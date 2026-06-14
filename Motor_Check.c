/**
  ******************************************************************************
  * @file    Motor_Check.c
  * @brief   Phase sequence detector implementation
  *          Uses current injection method with 3 test pulses
  *          
  *          HARDWARE CALIBRATION:
  *          - Shunt: 20 mΩ
  *          - Op-amp gain: 1.2
  *          - Sensitivity: 24 mV/A
  *          - Conversion: I(A) = (ADC_raw - offset) × 0.80566 / 24.0
  *          - Simplified: I(A) = (ADC_raw - offset) × 0.03357
  ******************************************************************************
  */
#include "Motor_Check.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* External handles (from CubeMX) */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

/* ========================== PRIVATE HELPERS ========================== */

/**
  * @brief Convert raw ADC value to Amperes for phase current
  *        Based on schematic analysis:
  *        - Shunt resistor: 0.02 Ohm (20 mΩ)
  *        - Op-amp gain: 1.2 (MCP601, R18=3k, R15=15k)
  *        - Sensitivity = 0.02 × 1.2 = 0.024 V/A = 24 mV/A
  *        - ADC to mV: raw × (3300/4095) = raw × 0.80566
  *        - Final: I = (ADC_raw - offset) × 0.80566 / 24.0
  *        - Simplified: I = (ADC_raw - offset) × 0.03357
  * @param mc Pointer to Motor_Check structure
  * @param adc_raw Raw ADC value (0-4095)
  * @param phase_idx 0=U, 1=V, 2=W
  * @return Current in Amperes
  */
float MC_AdcToAmps(Motor_Check_t *mc, uint16_t adc_raw, uint8_t phase_idx)
{
    float current;
    float voltage_mv;
    float offset_mv;
    
    /* Step 1: Convert ADC raw to millivolts */
    voltage_mv = (float)adc_raw * MC_ADC_TO_MV;
    
    /* Step 2: Subtract zero offset (calibrated value) */
    offset_mv = (float)mc->cal.offset_raw[phase_idx] * MC_ADC_TO_MV;
    voltage_mv -= offset_mv;
    
    /* Step 3: Convert mV to Amperes using sensitivity (24 mV/A) */
    current = voltage_mv / MC_SENSITIVITY_MV_PER_A;
    
    /* Step 4: Apply noise floor (ignore very small readings) */
    if (fabsf(current) < MC_CURRENT_THRESHOLD_A) {
        current = 0.0f;
    }
    
    /* Step 5: Clamp to safe range */
    if (current > MC_MAX_TEST_CURRENT_A) {
        current = MC_MAX_TEST_CURRENT_A;
    }
    if (current < -MC_MAX_TEST_CURRENT_A) {
        current = -MC_MAX_TEST_CURRENT_A;
    }
    
    return current;
}

/**
  * @brief Get raw ADC values for phase currents
  * @param meas Pointer to Meas_t (contains DMA buffer)
  * @param raw_u Output raw U phase ADC
  * @param raw_v Output raw V phase ADC
  * @param raw_w Output raw W phase ADC
  */
static void MC_GetRawCurrents(Meas_t *meas, uint16_t *raw_u, uint16_t *raw_v, uint16_t *raw_w)
{
    /* ADC channel mapping from adc.c scan order */
    *raw_u = meas->dma[MC_IDX_IU];   /* Index 2 = CH2 = U_Current */
    *raw_v = meas->dma[MC_IDX_IV];   /* Index 3 = CH3 = V_Current */
    *raw_w = meas->dma[MC_IDX_IW];   /* Index 4 = CH4 = W_Current */
}

/**
  * @brief Inject a test pulse on a specific phase
  * @param mc Pointer to Motor_Check structure
  * @param phase Phase to inject pulse on (U, V, or W)
  * @param duration_ms Pulse duration in milliseconds
  * @param duty_percent Duty cycle percentage (0-100)
  * @return true if pulse was applied successfully
  */
static bool MC_InjectPulse(Motor_Check_t *mc, MC_Phase_t phase, 
                           uint32_t duration_ms, uint8_t duty_percent)
{
    TIM_HandleTypeDef *htim = mc->htim;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr_value = (uint32_t)((float)arr * (float)duty_percent / 100.0f);
    
    /* Store current CCR values to restore later */
    uint32_t old_ccr1 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
    uint32_t old_ccr2 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
    uint32_t old_ccr3 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_3);
    
    /* Set all phases to 0 first */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
    
    /* Apply pulse to selected phase */
    switch (phase) {
        case MC_PHASE_U:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr_value);
            break;
        case MC_PHASE_V:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, ccr_value);
            break;
        case MC_PHASE_W:
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr_value);
            break;
        default:
            return false;
    }
    
    /* Wait for duration */
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms) {
        /* Small delay to allow current to stabilize */
        for (volatile int i = 0; i < 100; i++);
    }
    
    /* Turn off all phases */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, 0);
    
    /* Restore old values */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, old_ccr1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, old_ccr2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, old_ccr3);
    
    return true;
}

/**
  * @brief Read current response after pulse injection
  * @param mc Pointer to Motor_Check structure
  * @param result Output structure to store results
  */
static void MC_ReadPulseResponse(Motor_Check_t *mc, MC_PulseResult_t *result)
{
    uint16_t raw_u, raw_v, raw_w;
    
    /* Give ADC time to settle */
    HAL_Delay(1);
    
    /* Read raw ADC values */
    MC_GetRawCurrents(mc->meas, &raw_u, &raw_v, &raw_w);
    
    /* Convert to Amperes */
    result->current_u_a = MC_AdcToAmps(mc, raw_u, 0);
    result->current_v_a = MC_AdcToAmps(mc, raw_v, 1);
    result->current_w_a = MC_AdcToAmps(mc, raw_w, 2);
    
    /* Determine which phase responded */
    float max_current = 0;
    result->detected_phase = MC_PHASE_NONE;
    
    if (result->current_u_a > MC_CURRENT_THRESHOLD_A && 
        result->current_u_a > max_current) {
        max_current = result->current_u_a;
        result->detected_phase = MC_PHASE_U;
    }
    
    if (result->current_v_a > MC_CURRENT_THRESHOLD_A && 
        result->current_v_a > max_current) {
        max_current = result->current_v_a;
        result->detected_phase = MC_PHASE_V;
    }
    
    if (result->current_w_a > MC_CURRENT_THRESHOLD_A && 
        result->current_w_a > max_current) {
        max_current = result->current_w_a;
        result->detected_phase = MC_PHASE_W;
    }
    
    result->valid = (result->detected_phase != MC_PHASE_NONE);
    
    /* Safety check: prevent overcurrent */
    if (result->current_u_a > MC_MAX_TEST_CURRENT_A ||
        result->current_v_a > MC_MAX_TEST_CURRENT_A ||
        result->current_w_a > MC_MAX_TEST_CURRENT_A) {
        result->valid = false;
    }
}

/**
  * @brief Enable PWM outputs for test (with MOE)
  * @param mc Pointer to Motor_Check structure
  */
static void MC_EnableOutputs(Motor_Check_t *mc)
{
    TIM_HandleTypeDef *htim = mc->htim;
    
    /* Clear break flag */
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_BREAK);
    
    /* Enable MOE (main output enable) */
    __HAL_TIM_MOE_ENABLE(htim);
}

/**
  * @brief Disable PWM outputs
  * @param mc Pointer to Motor_Check structure
  */
static void MC_DisableOutputs(Motor_Check_t *mc)
{
    TIM_HandleTypeDef *htim = mc->htim;
    __HAL_TIM_MOE_DISABLE(htim);
}

/**
  * @brief Wait for cooldown period between tests
  */
static void MC_Cooldown(void)
{
    HAL_Delay(MC_COOLDOWN_MS);
}

/**
  * @brief Send debug message via UART
  */
static void MC_SendDebug(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

/* ========================== PUBLIC FUNCTIONS ========================== */

/**
  * @brief Initialize phase sequence detector
  */
void MC_Init(Motor_Check_t *mc, 
             Motor_Handle_t *motor,
             TIM_HandleTypeDef *htim,
             Meas_t *meas)
{
    memset(mc, 0, sizeof(Motor_Check_t));
    
    mc->motor = motor;
    mc->htim = htim;
    mc->meas = meas;
    
    mc->status = MC_SEQUENCE_UNKNOWN;
    mc->method = MC_METHOD_CURRENT_ONLY;
    mc->compensation_active = false;
    mc->check_count = 0;
    mc->check_passed = false;
    mc->pulse_active = false;
    mc->last_error_ms = 0;
    
    /* Initialize current filters */
    mc->current_u_filtered = 0;
    mc->current_v_filtered = 0;
    mc->current_w_filtered = 0;
    
    /* Initialize calibration */
    mc->cal.calibrated = false;
    mc->cal.offset_raw[0] = 0;
    mc->cal.offset_raw[1] = 0;
    mc->cal.offset_raw[2] = 0;
    mc->cal.offset_amps[0] = 0;
    mc->cal.offset_amps[1] = 0;
    mc->cal.offset_amps[2] = 0;
}

/**
  * @brief Calibrate current sensors
  */
bool MC_CalibrateCurrentSensors(Motor_Check_t *mc)
{
    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    uint16_t raw_u, raw_v, raw_w;
    
    /* Verify motor is in IDLE state and outputs are disabled */
    if (mc->motor->state != MOTOR_STATE_IDLE) {
        mc->last_error_ms = HAL_GetTick();
        MC_SendDebug("LOG:CALIB_ERROR_MOTOR_NOT_IDLE\r\n");
        return false;
    }
    
    MC_SendDebug("LOG:CALIB_START\r\n");
    
    /* Take multiple samples and average */
    for (uint32_t i = 0; i < MC_CALIBRATION_SAMPLES; i++) {
        MC_GetRawCurrents(mc->meas, &raw_u, &raw_v, &raw_w);
        sum_u += raw_u;
        sum_v += raw_v;
        sum_w += raw_w;
        
        /* Small delay between samples */
        for (volatile int d = 0; d < 100; d++);
    }
    
    /* Calculate average raw values (zero current offset) */
    mc->cal.offset_raw[0] = (uint16_t)(sum_u / MC_CALIBRATION_SAMPLES);
    mc->cal.offset_raw[1] = (uint16_t)(sum_v / MC_CALIBRATION_SAMPLES);
    mc->cal.offset_raw[2] = (uint16_t)(sum_w / MC_CALIBRATION_SAMPLES);
    
    /* Convert offset to Amperes using sensitivity (24 mV/A) */
    for (int i = 0; i < 3; i++) {
        float offset_mv = (float)mc->cal.offset_raw[i] * MC_ADC_TO_MV;
        mc->cal.offset_amps[i] = offset_mv / MC_SENSITIVITY_MV_PER_A;
    }
    
    mc->cal.calibrated = true;
    mc->cal.cal_timestamp_ms = HAL_GetTick();
    
    /* Send calibration results via UART */
    char buf[128];
    snprintf(buf, sizeof(buf), 
             "LOG:CALIB_COMPLETE U=%u(%.2fmV/%.3fA) V=%u(%.2fmV/%.3fA) W=%u(%.2fmV/%.3fA)\r\n",
             mc->cal.offset_raw[0], 
             (float)mc->cal.offset_raw[0] * MC_ADC_TO_MV,
             mc->cal.offset_amps[0],
             mc->cal.offset_raw[1],
             (float)mc->cal.offset_raw[1] * MC_ADC_TO_MV,
             mc->cal.offset_amps[1],
             mc->cal.offset_raw[2],
             (float)mc->cal.offset_raw[2] * MC_ADC_TO_MV,
             mc->cal.offset_amps[2]);
    MC_SendDebug(buf);
    
    return true;
}

/**
  * @brief Perform complete phase sequence check
  */
MC_Sequence_Status_t MC_CheckSequence(Motor_Check_t *mc)
{
    TIM_HandleTypeDef *htim = mc->htim;
    uint32_t old_ccr1, old_ccr2, old_ccr3;
    MC_PulseResult_t results[3];
    MC_Phase_t injected[] = {MC_PHASE_U, MC_PHASE_V, MC_PHASE_W};
    MC_Phase_t detected[3] = {MC_PHASE_NONE, MC_PHASE_NONE, MC_PHASE_NONE};
    char buf[64];
    
    MC_SendDebug("LOG:PHASE_CHECK_START\r\n");
    
    /* Verify calibration */
    if (!mc->cal.calibrated) {
        MC_SendDebug("LOG:PHASE_CHECK_NO_CALIB\r\n");
        if (!MC_CalibrateCurrentSensors(mc)) {
            mc->status = MC_SEQUENCE_ERROR;
            return mc->status;
        }
    }
    
    /* Verify motor is in IDLE state */
    if (mc->motor->state != MOTOR_STATE_IDLE) {
        mc->last_error_ms = HAL_GetTick();
        mc->status = MC_SEQUENCE_ERROR;
        MC_SendDebug("LOG:PHASE_CHECK_MOTOR_BUSY\r\n");
        return mc->status;
    }
    
    /* Store original CCR values to restore later */
    old_ccr1 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
    old_ccr2 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
    old_ccr3 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_3);
    
    /* Enable PWM outputs for test */
    MC_EnableOutputs(mc);
    
    /* Test each phase */
    for (int i = 0; i < 3; i++) {
        results[i].injected_phase = injected[i];
        
        /* Send test start message */
        snprintf(buf, sizeof(buf), "LOG:TEST_PHASE_%c\r\n", 
                 (injected[i] == MC_PHASE_U) ? 'U' : 
                 (injected[i] == MC_PHASE_V) ? 'V' : 'W');
        MC_SendDebug(buf);
        
        /* Inject pulse on current phase */
        if (!MC_InjectPulse(mc, injected[i], MC_PULSE_DURATION_MS, MC_PULSE_DUTY_PERCENT)) {
            MC_DisableOutputs(mc);
            mc->status = MC_SEQUENCE_ERROR;
            MC_SendDebug("LOG:PULSE_INJECTION_FAILED\r\n");
            return mc->status;
        }
        
        /* Read response */
        MC_ReadPulseResponse(mc, &results[i]);
        detected[i] = results[i].detected_phase;
        
        /* Send response message */
        snprintf(buf, sizeof(buf), "LOG:RESPONSE_U=%.2fA V=%.2fA W=%.2fA DETECT=%c\r\n",
                 results[i].current_u_a, results[i].current_v_a, results[i].current_w_a,
                 (detected[i] == MC_PHASE_U) ? 'U' :
                 (detected[i] == MC_PHASE_V) ? 'V' :
                 (detected[i] == MC_PHASE_W) ? 'W' : '?');
        MC_SendDebug(buf);
        
        /* Cooldown before next test */
        MC_Cooldown();
    }
    
    /* Disable outputs after all tests */
    MC_DisableOutputs(mc);
    
    /* Restore original CCR values */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, old_ccr1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, old_ccr2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, old_ccr3);
    
    /* Analyze results */
    /* Expected mapping: Pulse on U -> detects U, Pulse on V -> detects V, Pulse on W -> detects W */
    bool correct = (detected[0] == MC_PHASE_U && 
                    detected[1] == MC_PHASE_V && 
                    detected[2] == MC_PHASE_W);
    
    bool reversed = (detected[0] == MC_PHASE_U && 
                     detected[1] == MC_PHASE_W && 
                     detected[2] == MC_PHASE_V);
    
    if (correct) {
        mc->status = MC_SEQUENCE_CORRECT;
        mc->check_passed = true;
        MC_SendDebug("LOG:PHASE_SEQUENCE=CORRECT (U->V->W)\r\n");
    } else if (reversed) {
        mc->status = MC_SEQUENCE_REVERSED;
        mc->check_passed = true;
        MC_SendDebug("LOG:PHASE_SEQUENCE=REVERSED (U->W->V)\r\n");
    } else {
        mc->status = MC_SEQUENCE_ERROR;
        mc->check_passed = false;
        mc->last_error_ms = HAL_GetTick();
        MC_SendDebug("LOG:PHASE_SEQUENCE=ERROR\r\n");
    }
    
    mc->last_check_ms = HAL_GetTick();
    mc->check_count++;
    
    return mc->status;
}

/**
  * @brief Get current phase sequence status
  */
MC_Sequence_Status_t MC_GetStatus(Motor_Check_t *mc)
{
    return mc->status;
}

/**
  * @brief Enable or disable compensation
  */
void MC_SetCompensation(Motor_Check_t *mc, bool enable)
{
    mc->compensation_active = enable;
    
    if (enable) {
        MC_SendDebug("LOG:PHASE_COMPENSATION=ON\r\n");
    } else {
        MC_SendDebug("LOG:PHASE_COMPENSATION=OFF\r\n");
    }
}

/**
  * @brief Get effective direction considering compensation
  */
bool MC_GetEffectiveDirection(Motor_Check_t *mc, bool requested_reverse)
{
    /* If sequence is correct or unknown, use requested direction */
    if (mc->status != MC_SEQUENCE_REVERSED) {
        return requested_reverse;
    }
    
    /* If sequence is reversed and compensation is active, invert direction */
    if (mc->compensation_active) {
        return !requested_reverse;
    }
    
    /* Reversed but no compensation, use requested direction (warning will be shown) */
    return requested_reverse;
}

/**
  * @brief Report status via UART
  */
void MC_ReportStatus(Motor_Check_t *mc)
{
    const char *status_str;
    char buf[128];
    
    switch (mc->status) {
        case MC_SEQUENCE_CORRECT:
            status_str = "OK (U->V->W)";
            break;
        case MC_SEQUENCE_REVERSED:
            status_str = "REVERSED (U->W->V)";
            break;
        case MC_SEQUENCE_ERROR:
            status_str = "ERROR";
            break;
        default:
            status_str = "UNKNOWN";
            break;
    }
    
    snprintf(buf, sizeof(buf), "LOG:PHASE_SEQ_STATUS=%s\r\n", status_str);
    MC_SendDebug(buf);
    
    if (mc->compensation_active && mc->status == MC_SEQUENCE_REVERSED) {
        MC_SendDebug("LOG:PHASE_COMP_ACTIVE=TRUE\r\n");
    }
    
    /* Send detailed calibration info */
    snprintf(buf, sizeof(buf), 
             "LOG:CALIB_OFFSETS U=%u(%.3fA) V=%u(%.3fA) W=%u(%.3fA)\r\n",
             mc->cal.offset_raw[0], mc->cal.offset_amps[0],
             mc->cal.offset_raw[1], mc->cal.offset_amps[1],
             mc->cal.offset_raw[2], mc->cal.offset_amps[2]);
    MC_SendDebug(buf);
}

/**
  * @brief Reset detection state
  */
void MC_Reset(Motor_Check_t *mc)
{
    mc->status = MC_SEQUENCE_UNKNOWN;
    mc->check_passed = false;
    mc->check_count = 0;
    mc->last_check_ms = 0;
    MC_SendDebug("LOG:PHASE_CHECK_RESET\r\n");
}

/**
  * @brief Update phase currents (call from fast control loop)
  */
void MC_UpdatePhaseCurrents(Motor_Check_t *mc, Meas_t *meas)
{
    uint16_t raw_u, raw_v, raw_w;
    
    MC_GetRawCurrents(meas, &raw_u, &raw_v, &raw_w);
    
    /* Simple IIR filter for smoother readings */
    const float alpha = 0.3f;
    
    float new_u = MC_AdcToAmps(mc, raw_u, 0);
    float new_v = MC_AdcToAmps(mc, raw_v, 1);
    float new_w = MC_AdcToAmps(mc, raw_w, 2);
    
    mc->current_u_raw = new_u;
    mc->current_v_raw = new_v;
    mc->current_w_raw = new_w;
    
    mc->current_u_filtered = mc->current_u_filtered + alpha * (new_u - mc->current_u_filtered);
    mc->current_v_filtered = mc->current_v_filtered + alpha * (new_v - mc->current_v_filtered);
    mc->current_w_filtered = mc->current_w_filtered + alpha * (new_w - mc->current_w_filtered);
}

/**
  * @brief Check if sequence is verified
  */
bool MC_IsVerified(Motor_Check_t *mc)
{
    return (mc->status == MC_SEQUENCE_CORRECT) ||
           (mc->status == MC_SEQUENCE_REVERSED && mc->compensation_active);
}

/**
  * @brief Get last error time
  */
uint32_t MC_GetLastErrorTime(Motor_Check_t *mc)
{
    return mc->last_error_ms;
}

/* ========================== END OF FILE ========================== */