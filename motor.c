/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Implementation of the inverter control core (see motor.h).
  ******************************************************************************
  */
#include "motor.h"
#include "main.h"          /* ERORR_Pin / ERORR_GPIO_Port (PA12) */
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Low-level output gating                                           */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/*  Hardware-break enable line  (PA12 = ERORR_Pin, open-drain)        */
/*  Wired on the board to PB12 = TIM1_BKIN (active-low break).        */
/*    HIGH (released) = healthy  -> BKIN high -> PWM allowed           */
/*    LOW  (asserted) = fault    -> BKIN low  -> TIM1 outputs cut (HW) */
/*  The external protection comparator shares this open-drain net and  */
/*  can pull it low independently of the CPU (true hardware trip).     */
/* ------------------------------------------------------------------ */
static inline void brk_release(void) { HAL_GPIO_WritePin(ERORR_GPIO_Port, ERORR_Pin, GPIO_PIN_SET);   }
static inline void brk_assert (void) { HAL_GPIO_WritePin(ERORR_GPIO_Port, ERORR_Pin, GPIO_PIN_RESET); }

static inline void out_enable (Motor_Handle_t *m)
{
    brk_release();                                   /* let BKIN go high first */
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_BREAK);   /* drop any stale latch    */
    __HAL_TIM_MOE_ENABLE(m->htim);
}
/* controlled (non-fault) disable: outputs off via MOE, break line stays
   released/high so a normal stop is NOT reported as a hardware fault. */
static inline void out_disable(Motor_Handle_t *m) { __HAL_TIM_MOE_DISABLE(m->htim); }

static inline void set_ccr(Motor_Handle_t *m, uint32_t a, uint32_t b, uint32_t c)
{
    __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_1, a);   /* HIN(U) PA8 + LIN(U) PB13 */
    __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_2, b);   /* HIN(V) PA9 + LIN(V) PB14 */
    __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_3, c);   /* HIN(W) PA10+ LIN(W) PB15 */
}

/* timer clock taking the APB2 prescaler doubling rule into account */
static float pwm_period_s(TIM_HandleTypeDef *htim)
{
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    uint32_t timclk = (ppre2 >= 4u) ? (pclk2 * 2u) : pclk2;   /* /1 -> x1, else x2 */

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t psc = htim->Instance->PSC;
    float f = (float)timclk / ((float)(psc + 1u) * (float)(arr + 1u));
    return (f > 1.0f) ? (1.0f / f) : (1.0f / 9142.0f);
}

/* recompute V/Hz characteristic from nameplate */
static void recompute_vf(Motor_Handle_t *m)
{
    m->f_base = (float)m->line_hz;
    if (m->f_base < 1.0f) m->f_base = 50.0f;
    /* peak phase-voltage demand the modulator can apply ~ vdc/sqrt3.
     * vf_ratio maps line frequency to that demand at base speed. */
    float v_peak = (m->v_rated > 1.0f) ? (m->v_rated * 1.41421356f / FOC_SQRT3)
                                       : 180.0f;
    m->vf_ratio = v_peak / m->f_base;     /* volts per Hz                     */
    m->vf_boost = v_peak * 0.04f;         /* ~4% low-speed boost              */
}

/* ------------------------------------------------------------------ */
/*  Init                                                              */
/* ------------------------------------------------------------------ */
void Motor_Init(Motor_Handle_t *m, TIM_HandleTypeDef *htim1, Meas_t *meas)
{
    m->state = MOTOR_STATE_IDLE;
    m->current_freq_hz_x10 = 0;
    m->target_freq_hz_x10  = MOTOR_DEFAULT_RUN_HZ * 10;
    m->ramp_rate_hz_per_sec_x10 = 100;     /* 10.0 Hz/s default               */
    m->is_reverse = false;
    m->amplitude  = 0;

    m->htim    = htim1;
    m->meas    = meas;
    m->pwm_arr = __HAL_TIM_GET_AUTORELOAD(htim1);
    m->dt      = pwm_period_s(htim1);
    m->theta   = 0.0f;

    m->on_start_callback = 0;
    m->on_stop_callback  = 0;
    m->on_direction_change_callback = 0;
    m->on_fault_callback = 0;

    m->mode    = MOTOR_MODE_VF;
    m->poles   = MOTOR_DEFAULT_POLES;
    m->line_hz = MOTOR_DEFAULT_LINE_HZ;
    m->v_rated = 220.0f;
    m->torque_ref_pct = 100.0f;

    /* protection defaults (trimmed by nameplate) */
    m->oc_amp  = 18.0f;                    /* < IGBT 20 A continuous          */
    m->ot_degc = 85.0f;
    m->uv_volt = 150.0f;
    m->ov_volt = 420.0f;

    m->rpm = m->torque_pct = 0.0f;
    m->ramp_frac = 0;
    m->last_fault = MOTOR_FAULT_NONE;

    recompute_vf(m);

    out_disable(m);
    brk_release();          /* idle = healthy line high, outputs off via MOE */
    set_ccr(m, 0, 0, 0);
}

/* ------------------------------------------------------------------ */
/*  Commands                                                          */
/* ------------------------------------------------------------------ */
static int clampi(int v, int lo, int hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

void Motor_Start(Motor_Handle_t *m, int hz, bool reverse)
{
    if (m->state == MOTOR_STATE_FAULT) return;       /* clear the fault first */

    m->is_reverse = reverse;
    m->target_freq_hz_x10 = clampi(hz, MOTOR_MIN_SPEED_HZ, MOTOR_MAX_SPEED_HZ) * 10;

    if (m->state == MOTOR_STATE_IDLE || m->state == MOTOR_STATE_STOPPING)
    {
        m->theta = 0.0f;
        m->current_freq_hz_x10 = 0;
        m->ramp_frac = 0;

        /* bootstrap precharge: enable outputs with all low-side ON (ccr=0) */
        set_ccr(m, 0, 0, 0);
        out_enable(m);
        m->phase_t0_ms = HAL_GetTick();
        m->state = MOTOR_STATE_PRECHARGE;

        if (m->on_start_callback) m->on_start_callback();
    }
}

void Motor_Stop(Motor_Handle_t *m, bool emergency)
{
    if (emergency)
    {
        out_disable(m);
        set_ccr(m, 0, 0, 0);
        m->current_freq_hz_x10 = 0;
        m->amplitude = 0;
        if (m->state != MOTOR_STATE_FAULT) m->state = MOTOR_STATE_IDLE;
        if (m->on_stop_callback) m->on_stop_callback();
        return;
    }

    if (m->state == MOTOR_STATE_RUN ||
        m->state == MOTOR_STATE_PRECHARGE ||
        m->state == MOTOR_STATE_CALIB)
    {
        m->target_freq_hz_x10 = 0;
        m->state = MOTOR_STATE_STOPPING;        /* soft stop via the ramp     */
    }
}

void Motor_SetFrequency(Motor_Handle_t *m, int hz)
{
    m->target_freq_hz_x10 = clampi(hz, MOTOR_MIN_SPEED_HZ, MOTOR_MAX_SPEED_HZ) * 10;
}

void Motor_SetDirection(Motor_Handle_t *m, bool reverse)
{
    if (m->is_reverse == reverse) return;

    /* if running, force a controlled stop before reversing */
    if (m->state == MOTOR_STATE_RUN)
        m->state = MOTOR_STATE_STOPPING, m->target_freq_hz_x10 = 0;

    m->is_reverse = reverse;
    if (m->on_direction_change_callback) m->on_direction_change_callback(reverse);
}

void Motor_SetRampRate(Motor_Handle_t *m, int hz_per_sec)
{
    m->ramp_rate_hz_per_sec_x10 = clampi(hz_per_sec, 1, 100) * 10;
}

void Motor_ClearFault(Motor_Handle_t *m)
{
    if (m->state == MOTOR_STATE_FAULT)
    {
        out_disable(m);
        brk_release();                                   /* release PA12 -> BKIN high */
        __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_BREAK);   /* drop the latched break    */
        set_ccr(m, 0, 0, 0);
        m->last_fault = MOTOR_FAULT_NONE;
        m->state = MOTOR_STATE_IDLE;
    }
}

void Motor_SetTorque(Motor_Handle_t *m, int pct)
{
    m->torque_ref_pct = (float)clampi(pct, 0, 100);
}

void Motor_SetPoles(Motor_Handle_t *m, int poles)
{
    if (poles >= 2 && poles <= 64) m->poles = poles;
}

void Motor_SetLineFreq(Motor_Handle_t *m, int hz)
{
    m->line_hz = (hz >= 55) ? 60 : 50;
    recompute_vf(m);
}

void Motor_SetMode(Motor_Handle_t *m, Motor_Mode_t mode)
{
    if (m->state == MOTOR_STATE_RUN) return;     /* only switch while stopped */
    m->mode = mode;
}

void Motor_ConfigNameplate(Motor_Handle_t *m, float volts, float amps,
                           float freq, float rpm)
{
    if (volts > 1.0f) m->v_rated = volts;
    if (freq  > 1.0f) m->line_hz = (freq >= 55.0f) ? 60 : 50;
    if (amps  > 0.1f)
    {
        m->oc_amp = amps * 1.20f;                 /* 120% rated -> trip       */
        if (m->oc_amp > 19.5f) m->oc_amp = 19.5f; /* IGBT ceiling             */
    }
    /* infer pole count from rated speed if a sane value was given */
    if (rpm > 100.0f && freq > 1.0f)
    {
        int p = (int)((120.0f * freq / rpm) + 0.5f);
        if (p >= 2 && p <= 64) m->poles = p;
    }
    recompute_vf(m);
}

void Motor_OnFault(Motor_Handle_t *m, Motor_Fault_t f)
{
    if (m->state == MOTOR_STATE_FAULT) return;
    m->state     = MOTOR_STATE_FAULT;   /* set first: the break IRQ then no-ops */
    m->last_fault = f;
    out_disable(m);                      /* MOE off                              */
    brk_assert();                        /* PA12=0 -> PB12=0 -> latch HW break    */
    set_ccr(m, 0, 0, 0);
    m->current_freq_hz_x10 = 0;
    m->amplitude = 0;
    if (m->on_fault_callback) m->on_fault_callback(f);
}

/* ------------------------------------------------------------------ */
/*  Control ISR  (TIM1 update, PWM rate)                              */
/* ------------------------------------------------------------------ */
void Motor_ControlISR(Motor_Handle_t *m)
{
    Meas_UpdateFast(m->meas);                /* refresh phase currents        */

    if (m->state == MOTOR_STATE_RUN || m->state == MOTOR_STATE_STOPPING)
    {
        float f_hz = (float)m->current_freq_hz_x10 * 0.1f;
        float we   = FOC_TWO_PI * f_hz;                 /* elec. rad/s         */
        float dir  = m->is_reverse ? -1.0f : 1.0f;

        /* advance electrical angle (open-loop / forced commutation) */
        m->theta += dir * we * m->dt;
        if (m->theta >= FOC_TWO_PI) m->theta -= FOC_TWO_PI;
        if (m->theta < 0.0f)        m->theta += FOC_TWO_PI;

        float s = sinf(m->theta);
        float c = cosf(m->theta);

        float vdc = m->meas->vdc_f;
        if (vdc < 50.0f) vdc = 50.0f;        /* guard before bus is measured  */

        float valpha, vbeta;

        /* --- V/F scalar -------------------------------------------------- */
        float vmag = m->vf_ratio * f_hz + m->vf_boost;
        float vmax = vdc / FOC_SQRT3;          /* SVPWM linear limit       */
        if (vmag > vmax) vmag = vmax;
        /* torque slider trims applied volts in V/F (acts like a limiter) */
        vmag *= (0.30f + 0.70f * (m->torque_ref_pct * 0.01f));
        valpha = vmag * c;
        vbeta  = vmag * s;

        uint32_t a, b, cc;
        float pk = SVPWM(valpha, vbeta, vdc, m->pwm_arr, &a, &b, &cc);
        set_ccr(m, a, b, cc);
        m->amplitude = (uint32_t)(pk * (float)m->pwm_arr);
    }
    else
    {
        /* IDLE / PRECHARGE / CALIB / FAULT -> low side held (ccr=0) */
        set_ccr(m, 0, 0, 0);
        m->amplitude = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  1 kHz housekeeping (TIM10)                                        */
/* ------------------------------------------------------------------ */
static void ramp_step(Motor_Handle_t *m)
{
    /* move current_freq_hz_x10 toward target by ramp_rate per second.
     * called at 1 kHz: per-tick delta (in x10 units) = ramp_x10 / 1000,
     * fractional remainder carried in ramp_frac (units of x10/1000). */
    int delta_milli = m->ramp_rate_hz_per_sec_x10;   /* x10 per 1000 ms       */
    m->ramp_frac += delta_milli;
    int step = m->ramp_frac / 1000;
    m->ramp_frac -= step * 1000;

    if (m->current_freq_hz_x10 < m->target_freq_hz_x10)
    {
        m->current_freq_hz_x10 += step;
        if (m->current_freq_hz_x10 > m->target_freq_hz_x10)
            m->current_freq_hz_x10 = m->target_freq_hz_x10;
    }
    else if (m->current_freq_hz_x10 > m->target_freq_hz_x10)
    {
        m->current_freq_hz_x10 -= step;
        if (m->current_freq_hz_x10 < m->target_freq_hz_x10)
            m->current_freq_hz_x10 = m->target_freq_hz_x10;
    }
}

void Motor_Tick1ms(Motor_Handle_t *m)
{

    Meas_UpdateSlow(m->meas);
    uint32_t now = HAL_GetTick();

    /* ---- NO software protection -----------------------------------------
     * Over-current / over-voltage / over-temp / under-voltage tripping is
     * delegated ENTIRELY to the hardware break: the protection comparator
     * pulls the ERROR net (PA12<->PB12=TIM1_BKIN) low, which cuts the six
     * gate outputs in hardware (MOE->0) with no CPU involvement.
     * Meas_UpdateSlow() above still runs, but only to feed the HMI display;
     * no threshold here can trip the drive. */

    /* ---- state sequencing ----------------------------------------------- */
    switch (m->state)
    {
    case MOTOR_STATE_PRECHARGE:
        if ((now - m->phase_t0_ms) >= MOTOR_PRECHARGE_MS)
        {
            if (!m->meas->calibrated) { Meas_StartCalib(m->meas);
                                        m->state = MOTOR_STATE_CALIB; }
            else                        m->state = MOTOR_STATE_RUN;
        }
        break;

    case MOTOR_STATE_CALIB:
        if (Meas_CalibTask(m->meas)) m->state = MOTOR_STATE_RUN;
        break;

    case MOTOR_STATE_RUN:
        ramp_step(m);
        break;

    case MOTOR_STATE_STOPPING:
        ramp_step(m);
        if (m->current_freq_hz_x10 <= 0)
        {
            out_disable(m);
            set_ccr(m, 0, 0, 0);
            m->state = MOTOR_STATE_IDLE;
            if (m->on_stop_callback) m->on_stop_callback();
        }
        break;

    default: /* IDLE, FAULT */
        break;
    }

    /* ---- derived telemetry (synchronous speed + torque estimate) -------- */
    float f_hz = (float)m->current_freq_hz_x10 * 0.1f;
    m->rpm = (m->poles > 0) ? (120.0f * f_hz / (float)m->poles) : 0.0f;

    /* torque estimate: approximate, proportional to |idc| referred to trip current */
    float t = (m->oc_amp > 0.1f) ? (fabsf(m->meas->idc) / m->oc_amp * 100.0f) : 0.0f;
    if (t > 150.0f) t = 150.0f;
    m->torque_pct += (t - m->torque_pct) * 0.10f;
    if (m->state != MOTOR_STATE_RUN && m->state != MOTOR_STATE_STOPPING)
        m->torque_pct *= 0.90f;
}
