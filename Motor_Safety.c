/**
  ******************************************************************************
  * @file    Motor_Safety.c
  * @brief   Implementation of Motor Safety Test module
  ******************************************************************************
  */

#include "Motor_Safety.h"
#include "SENSOR_ALL.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Internal Helper Functions                                                  */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* ADC raw → voltage (Vref=3.3V, 12-bit) */
static inline float adc_to_volt(uint16_t raw)
{
    return (float)raw * (3.3f / 4096.0f);
}

/* Current calculation: Shunt=0.02Ω, Gain=5 */
static float calc_phase_current(uint16_t raw, uint16_t offset)
{
    int32_t diff = (int32_t)raw - (int32_t)offset;
    if (diff <= 0) return 0.0f;
    float v_adc = (float)diff * (3.3f / 4096.0f);
    return v_adc / (5.0f * 0.02f);  /* Gain=5, Shunt=0.02Ω */
}

/* DC Voltage: divider 1361k÷10k → K=137.1 */
static float calc_dc_voltage(uint16_t raw)
{
    float v_adc = (float)raw * (3.3f / 4096.0f);
    return v_adc * 137.1f;
}

/* Temperature from NTC (R_pullup=47k, R25=50k, Beta=4092) */
static float calc_temperature(uint16_t raw)
{
    if (raw < 50 || raw > 4000) return 25.0f;  /* reasonable default on error */
    
    float v_ntc = adc_to_volt(raw);
    float denom = 3.3f - v_ntc;
    if (denom < 0.001f) return 125.0f;  /* max if open */
    
    float r_ntc = 47000.0f * v_ntc / denom;
    if (r_ntc <= 0.0f) return 125.0f;
    
    float ln_ratio = logf(r_ntc / 50000.0f);
    float inv_t = (1.0f / 298.15f) + (ln_ratio / 4092.0f);
    if (inv_t <= 0.0f) return -50.0f;
    
    return (1.0f / inv_t) - 273.15f;
}

/* Average calculation helper */
static float calc_average(float *arr, uint8_t count)
{
    if (count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++)
        sum += arr[i];
    return sum / (float)count;
}

/* Min/max helpers */
static void calc_minmax(float *arr, uint8_t count, float *out_min, float *out_max)
{
    if (count == 0) { *out_min = *out_max = 0.0f; return; }
    *out_min = *out_max = arr[0];
    for (uint8_t i = 1; i < count; i++) {
        if (arr[i] < *out_min) *out_min = arr[i];
        if (arr[i] > *out_max) *out_max = arr[i];
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Sample Collection from ADC DMA Buffer                                      */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void collect_sample(Motor_Safety_t *ms)
{
    if (ms->sample_index >= MOTOR_SAFETY_MAX_SAMPLES)
        return;  /* buffer full */
    
    const volatile uint16_t *adc = ms->adc_buf;
    Motor_Safety_Sample_t *s = &ms->samples[ms->sample_index];
    
    s->timestamp_ms = HAL_GetTick();
    
    /* ADC indices from SENSOR_ALL.h */
    s->temperature_c = calc_temperature(adc[0]);  /* IDX_NTC */
    s->dc_voltage = calc_dc_voltage(adc[1]);       /* IDX_DC_VOLT */
    
    /* Phase currents */
    s->current_u = calc_phase_current(adc[2], ms->offset_iu);  /* IDX_IU */
    s->current_v = calc_phase_current(adc[3], ms->offset_iv);  /* IDX_IV */
    s->current_w = calc_phase_current(adc[4], ms->offset_iw);  /* IDX_IW */
    
    /* Phase voltages (divider 1360k÷10k → K=137) */
    s->voltage_u = (float)adc[5] * (3.3f / 4096.0f) * 137.0f;  /* IDX_VU */
    s->voltage_v = (float)adc[6] * (3.3f / 4096.0f) * 137.0f;  /* IDX_VV */
    s->voltage_w = (float)adc[7] * (3.3f / 4096.0f) * 137.0f;  /* IDX_VW */
    
    /* DC current */
    s->dc_current = calc_phase_current(adc[8], ms->offset_dci);  /* IDX_DC_CURR */
    
    ms->sample_index++;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Statistics Calculation                                                     */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void calculate_statistics(Motor_Safety_t *ms)
{
    Motor_Safety_Stats_t *stats = &ms->stats;
    uint8_t n = ms->sample_index;
    
    if (n == 0) {
        memset(stats, 0, sizeof(Motor_Safety_Stats_t));
        return;
    }
    
    stats->sample_count = n;
    
    /* Temperature */
    float temps[MOTOR_SAFETY_MAX_SAMPLES];
    for (uint8_t i = 0; i < n; i++) temps[i] = ms->samples[i].temperature_c;
    stats->temp_avg = calc_average(temps, n);
    calc_minmax(temps, n, &stats->temp_min, &stats->temp_max);
    
    /* Voltages - Phase U */
    float voltages[MOTOR_SAFETY_MAX_SAMPLES];
    for (uint8_t i = 0; i < n; i++) voltages[i] = ms->samples[i].voltage_u;
    stats->vu_avg = calc_average(voltages, n);
    
    /* Phase V */
    for (uint8_t i = 0; i < n; i++) voltages[i] = ms->samples[i].voltage_v;
    stats->vv_avg = calc_average(voltages, n);
    
    /* Phase W */
    for (uint8_t i = 0; i < n; i++) voltages[i] = ms->samples[i].voltage_w;
    stats->vw_avg = calc_average(voltages, n);
    
    /* DC Voltage */
    for (uint8_t i = 0; i < n; i++) voltages[i] = ms->samples[i].dc_voltage;
    stats->dc_volt_avg = calc_average(voltages, n);
    calc_minmax(voltages, n, &stats->volt_min, &stats->volt_max);
    
    /* Currents */
    float currents[MOTOR_SAFETY_MAX_SAMPLES];
    for (uint8_t i = 0; i < n; i++) currents[i] = ms->samples[i].current_u;
    stats->iu_avg = calc_average(currents, n);
    
    for (uint8_t i = 0; i < n; i++) currents[i] = ms->samples[i].current_v;
    stats->iv_avg = calc_average(currents, n);
    
    for (uint8_t i = 0; i < n; i++) currents[i] = ms->samples[i].current_w;
    stats->iw_avg = calc_average(currents, n);
    
    for (uint8_t i = 0; i < n; i++) currents[i] = ms->samples[i].dc_current;
    stats->dc_curr_avg = calc_average(currents, n);
    
    /* Find max current */
    stats->curr_max = stats->iu_avg;
    if (stats->iv_avg > stats->curr_max) stats->curr_max = stats->iv_avg;
    if (stats->iw_avg > stats->curr_max) stats->curr_max = stats->iw_avg;
    if (stats->dc_curr_avg > stats->curr_max) stats->curr_max = stats->dc_curr_avg;
    
    stats->curr_min = stats->curr_max;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Fault Detection & Analysis                                                 */
/* ═══════════════════════════════════════════════════════════════════════════ */

static Motor_Safety_Fault_t analyze_results(Motor_Safety_t *ms)
{
    Motor_Safety_Stats_t *s = &ms->stats;
    
    /* Check Over Temperature */
    if (s->temp_max >= MOTOR_SAFETY_TEMP_CRITICAL) {
        ms->fault_message = "ERROR OVER temperature";
        return MOTOR_SAFETY_FAULT_OT;
    }
    
    /* Check Under Voltage */
    if (s->dc_volt_avg > 5.0f && s->dc_volt_avg < MOTOR_SAFETY_VOLT_UNDER) {
        ms->fault_message = "ERROR UNDER dc VOLTAGE";
        return MOTOR_SAFETY_FAULT_UV;
    }
    
    /* Check Over Voltage */
    if (s->dc_volt_avg >= MOTOR_SAFETY_VOLT_OVER) {
        ms->fault_message = "ERROR OVER dc VOLTAGE";
        return MOTOR_SAFETY_FAULT_OV;
    }
    
    /* Check Over Current */
    if (s->curr_max >= MOTOR_SAFETY_CURRENT_OVER) {
        ms->fault_message = "ERROR OVER Idc Current";
        return MOTOR_SAFETY_FAULT_OC;
    }
    
    /* All checks passed */
    ms->fault_message = "SELF-TEST PASSED - READY TO RUN";
    return MOTOR_SAFETY_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Error Pin & Message Sending                                               */
/* ═══════════════════════════════════════════════════════════════════════════ */

static void latch_error(Motor_Safety_t *ms, Motor_Safety_Fault_t fault)
{
    ms->fault_code = fault;
    ms->error_latched = 1U;
    ms->error_latch_time_ms = HAL_GetTick();
    
    /* Write ERORR_Pin LOW (fault) */
    if (ms->write_error_pin != NULL) {
        ms->write_error_pin(0);  /* GPIO_PIN_RESET */
    }
    
    /* Send message to HMI */
    if (ms->send_message != NULL && ms->fault_message != NULL) {
        ms->send_message(ms->fault_message);
    }
}

static void clear_error(Motor_Safety_t *ms)
{
    ms->error_latched = 0U;
    ms->fault_code = MOTOR_SAFETY_OK;
    
    /* Write ERORR_Pin HIGH (healthy) */
    if (ms->write_error_pin != NULL) {
        ms->write_error_pin(1);  /* GPIO_PIN_SET */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Public API Implementation                                                  */
/* ═══════════════════════════════════════════════════════════════════════════ */

bool Motor_Safety_Init(Motor_Safety_t *ms, const volatile uint16_t *adc_buf)
{
    if (ms == NULL || adc_buf == NULL) return false;
    
    memset(ms, 0, sizeof(Motor_Safety_t));
    
    ms->adc_buf = adc_buf;
    ms->state = MOTOR_SAFETY_IDLE;
    ms->fault_code = MOTOR_SAFETY_OK;
    ms->fault_message = "Initialized";
    ms->error_latched = 0U;
    
    /* Default sensor offsets (can be updated via Motor_Safety_SetCurrentOffsets) */
    ms->offset_iu = 0;
    ms->offset_iv = 0;
    ms->offset_iw = 0;
    ms->offset_dci = 0;
    
    return true;
}

bool Motor_Safety_StartTest(Motor_Safety_t *ms)
{
    if (ms == NULL) return false;
    if (ms->state != MOTOR_SAFETY_IDLE && ms->state != MOTOR_SAFETY_COMPLETE)
        return false;  /* test already in progress */
    
    ms->state = MOTOR_SAFETY_TESTING;
    ms->start_time_ms = HAL_GetTick();
    ms->last_sample_ms = ms->start_time_ms;
    ms->sample_index = 0;
    ms->fault_code = MOTOR_SAFETY_OK;
    ms->fault_message = "Test in progress...";
    
    memset(&ms->stats, 0, sizeof(Motor_Safety_Stats_t));
    
    return true;
}

bool Motor_Safety_Task(Motor_Safety_t *ms)
{
    if (ms == NULL) return false;
    if (ms->state == MOTOR_SAFETY_IDLE) return false;  /* not testing */
    
    uint32_t now = HAL_GetTick();
    
    /* Check if test duration exceeded */
    if ((now - ms->start_time_ms) >= MOTOR_SAFETY_TEST_DURATION_MS) {
        /* Test complete — analyze results */
        calculate_statistics(ms);
        Motor_Safety_Fault_t fault = analyze_results(ms);
        
        if (fault != MOTOR_SAFETY_OK) {
            ms->state = MOTOR_SAFETY_FAULT;
            latch_error(ms, fault);
        } else {
            ms->state = MOTOR_SAFETY_COMPLETE;
            clear_error(ms);
        }
        
        return false;  /* test complete */
    }
    
    /* Collect sample if interval elapsed */
    if ((now - ms->last_sample_ms) >= MOTOR_SAFETY_SAMPLE_INTERVAL_MS) {
        if (ms->sample_index < MOTOR_SAFETY_MAX_SAMPLES) {
            collect_sample(ms);
            ms->last_sample_ms = now;
        }
    }
    
    return true;  /* test still running */
}

Motor_Safety_State_t Motor_Safety_GetState(const Motor_Safety_t *ms)
{
    if (ms == NULL) return MOTOR_SAFETY_IDLE;
    return ms->state;
}

Motor_Safety_Fault_t Motor_Safety_GetFault(const Motor_Safety_t *ms)
{
    if (ms == NULL) return MOTOR_SAFETY_FAULT_UNKNOWN;
    return ms->fault_code;
}

const char *Motor_Safety_GetMessage(const Motor_Safety_t *ms)
{
    if (ms == NULL) return "NULL";
    return (ms->fault_message != NULL) ? ms->fault_message : "Unknown";
}

void Motor_Safety_GetStats(const Motor_Safety_t *ms, Motor_Safety_Stats_t *out)
{
    if (ms == NULL || out == NULL) return;
    memcpy(out, &ms->stats, sizeof(Motor_Safety_Stats_t));
}

void Motor_Safety_SetErrorPinCallback(Motor_Safety_t *ms,
                                       void (*callback)(uint8_t))
{
    if (ms != NULL)
        ms->write_error_pin = callback;
}

void Motor_Safety_SetMessageCallback(Motor_Safety_t *ms,
                                      void (*callback)(const char *))
{
    if (ms != NULL)
        ms->send_message = callback;
}

void Motor_Safety_SetCurrentOffsets(Motor_Safety_t *ms,
                                     uint16_t iu, uint16_t iv,
                                     uint16_t iw, uint16_t dci)
{
    if (ms != NULL) {
        ms->offset_iu = iu;
        ms->offset_iv = iv;
        ms->offset_iw = iw;
        ms->offset_dci = dci;
    }
}

bool Motor_Safety_IsErrorLatched(const Motor_Safety_t *ms)
{
    if (ms == NULL) return false;
    return (ms->error_latched != 0U);
}

void Motor_Safety_ClearError(Motor_Safety_t *ms)
{
    if (ms == NULL) return;
    clear_error(ms);
    if (ms->state == MOTOR_SAFETY_FAULT)
        ms->state = MOTOR_SAFETY_IDLE;
}

uint32_t Motor_Safety_GetErrorAge(const Motor_Safety_t *ms)
{
    if (ms == NULL || !Motor_Safety_IsErrorLatched(ms))
        return 0U;
    return HAL_GetTick() - ms->error_latch_time_ms;
}

int Motor_Safety_BuildJSON(const Motor_Safety_t *ms, char *buf, size_t len)
{
    if (ms == NULL || buf == NULL || len < 64) return 0;
    
    const Motor_Safety_Stats_t *s = &ms->stats;
    
    return snprintf(buf, len,
        "{\"test\":%d,\"fault\":%d,\"msg\":\"%s\","
        "\"temp_c\":%.1f,\"uv\":%.0f,\"vv\":%.0f,\"wv\":%.0f,"
        "\"dcv\":%.1f,\"ui\":%.2f,\"vi\":%.2f,\"wi\":%.2f,\"dci\":%.2f}",
        (int)ms->state,
        (int)ms->fault_code,
        (ms->fault_message != NULL) ? ms->fault_message : "",
        (double)s->temp_avg,
        (double)s->vu_avg, (double)s->vv_avg, (double)s->vw_avg,
        (double)s->dc_volt_avg,
        (double)s->iu_avg, (double)s->iv_avg, (double)s->iw_avg,
        (double)s->dc_curr_avg);
}
