/**
  ******************************************************************************
  * @file    motor.h
  * @brief   3-phase inverter control core for the STM32F411 / L6388 / IGBT
  *          board. Provides the exact handle/API that bt_motor.c expects,
  *          plus extensions (mode, poles, line frequency, torque) used by the
  *          HMI. Implements soft start / soft stop, V/F (scalar) and
  *          current-regulated FOC modulation. No blocking calls.
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "measure.h"
#include "foc.h"

/* ---- limits / defaults (Hz are *electrical* line frequency) --------------- */
#define MOTOR_MIN_SPEED_HZ      1
#define MOTOR_MAX_SPEED_HZ      120
#define MOTOR_DEFAULT_RUN_HZ    20

#define MOTOR_DEFAULT_POLES     4        /* pole COUNT (not pairs)             */
#define MOTOR_DEFAULT_LINE_HZ   50       /* 50 or 60 Hz base system            */

/* timed phases of the start sequence (ms) */
#define MOTOR_PRECHARGE_MS      25u      /* bootstrap-cap charge (low side on) */

/* ---- enums shared with bt_motor.c ----------------------------------------- */
typedef enum
{
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_PRECHARGE,
    MOTOR_STATE_CALIB,
    MOTOR_STATE_RUN,
    MOTOR_STATE_STOPPING,
    MOTOR_STATE_FAULT
} Motor_State_t;

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_OVERCURRENT,
    MOTOR_FAULT_OVERTEMP,
    MOTOR_FAULT_UNDERVOLTAGE,
    MOTOR_FAULT_OVERVOLTAGE,
    MOTOR_FAULT_BOOTSTRAP,
    MOTOR_FAULT_ESTOP,
    MOTOR_FAULT_UNKNOWN
} Motor_Fault_t;

typedef enum { MOTOR_MODE_VF = 0, MOTOR_MODE_FOC = 1 } Motor_Mode_t;

/* event callbacks (signatures used by bt_motor.c) */
typedef void (*Motor_VoidCb)(void);
typedef void (*Motor_DirCb)(bool reverse);
typedef void (*Motor_FaultCb)(Motor_Fault_t f);

typedef struct
{
    /* ---- fields read/written by bt_motor.c (names are part of the API) --- */
    Motor_State_t state;
    int           current_freq_hz_x10;     /* live ramped frequency  (x10)   */
    int           target_freq_hz_x10;      /* commanded frequency    (x10)   */
    int           ramp_rate_hz_per_sec_x10;/* ramp slope             (x10)   */
    bool          is_reverse;
    uint32_t      amplitude;               /* peak compare applied (0..arr)  */
    uint32_t      pwm_arr;                 /* timer auto-reload              */

    Motor_VoidCb  on_start_callback;
    Motor_VoidCb  on_stop_callback;
    Motor_DirCb   on_direction_change_callback;
    Motor_FaultCb on_fault_callback;

    /* ---- protection thresholds (set from nameplate / defaults) ----------- */
    float         oc_amp;                  /* phase over-current trip (A)    */
    float         ot_degc;                 /* over-temperature trip (deg C)  */
    float         uv_volt;                 /* DC under-voltage trip (V)      */
    float         ov_volt;                 /* DC over-voltage  trip (V)      */

    /* ---- nameplate / system ---------------------------------------------- */
    Motor_Mode_t  mode;
    int           poles;                   /* pole count                     */
    int           line_hz;                 /* 50 or 60                       */
    float         v_rated;                 /* nameplate volts                */
    float         f_base;                  /* base electrical freq (= line_hz)*/
    float         torque_ref_pct;          /* 0..100 (% of rated current)    */

    /* ---- modulation internals -------------------------------------------- */
    Meas_t       *meas;
    TIM_HandleTypeDef *htim;               /* TIM1                            */
    float         dt;                      /* control period (s)             */
    float         theta;                   /* electrical angle (rad)         */
    float         vf_ratio;                /* volts per Hz                   */
    float         vf_boost;                /* low-speed boost (V)            */

    /* ---- derived telemetry ----------------------------------------------- */
    float         rpm;                     /* synchronous shaft speed        */
    float         torque_pct;              /* estimated torque (% rated)     */

    /* ---- timing helpers --------------------------------------------------- */
    uint32_t      phase_t0_ms;
    int32_t       ramp_frac;               /* sub-0.1Hz ramp accumulator     */
    Motor_Fault_t last_fault;
} Motor_Handle_t;

/* ---- lifecycle ------------------------------------------------------------ */
void Motor_Init(Motor_Handle_t *m, TIM_HandleTypeDef *htim1, Meas_t *meas);

/* ---- commands (used by bt_motor.c) --------------------------------------- */
void Motor_Start      (Motor_Handle_t *m, int hz, bool reverse);
void Motor_Stop       (Motor_Handle_t *m, bool emergency);
void Motor_SetFrequency(Motor_Handle_t *m, int hz);
void Motor_SetDirection(Motor_Handle_t *m, bool reverse);
void Motor_SetRampRate (Motor_Handle_t *m, int hz_per_sec);   /* clamps 1..100 */
void Motor_ClearFault  (Motor_Handle_t *m);

/* ---- extended commands (used by the HMI via bt_motor.c) ------------------ */
void Motor_SetTorque  (Motor_Handle_t *m, int pct);           /* 0..100        */
void Motor_SetPoles   (Motor_Handle_t *m, int poles);
void Motor_SetLineFreq(Motor_Handle_t *m, int hz);            /* 50 / 60       */
void Motor_SetMode    (Motor_Handle_t *m, Motor_Mode_t mode);
void Motor_ConfigNameplate(Motor_Handle_t *m, float volts, float amps,
                           float freq, float rpm);

/* ---- runtime hooks -------------------------------------------------------- */
void Motor_ControlISR(Motor_Handle_t *m);   /* call from TIM1 update (PWM rate) */
void Motor_Tick1ms   (Motor_Handle_t *m);   /* call from TIM10 (1 kHz)          */
void Motor_OnFault   (Motor_Handle_t *m, Motor_Fault_t f);

#endif /* MOTOR_H */
