/**
  ******************************************************************************
  * @file    Motor_Check.h
  * @brief   Phase sequence detector for 3-phase induction motor drive
  *          Uses current injection method (U, V, W phases sequentially)
  *          Compatible with STM32F411 / TIM1 PWM / ADC current sensing
  *          
  *          HARDWARE SPECIFICATIONS (based on schematic analysis):
  *          - Shunt resistor: 0.02 Ohm (20 mΩ)
  *          - Op-amp: MCP601 with gain = 1.2 (R18=3k, R15=15k)
  *          - Sensitivity: 24 mV/A
  *          - ADC reference: 3.3V, 12-bit (0-4095)
  *          - Maximum continuous current: 7A (limited by shunt)
  *          - Maximum peak current (pulse): 20A
  ******************************************************************************
  */
#ifndef MOTOR_CHECK_H
#define MOTOR_CHECK_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "measure.h"

/* ========================== HARDWARE CONSTANTS ========================== */

/**
  * @brief Current sensor parameters based on schematic analysis:
  *        - Shunt resistor: 0.02 Ohm (20 mΩ)
  *        - Op-amp gain: 1.2 (MCP601, R18=3k, R15=15k)
  *        - Sensitivity = 0.02 × 1.2 = 0.024 V/A = 24 mV/A
  */
#define MC_SHUNT_RESISTOR_OHM        0.02f      /* 20 mΩ current sense resistor */
#define MC_OPAMP_GAIN                1.2f       /* Non-inverting gain = 1 + 3k/15k */
#define MC_SENSITIVITY_V_PER_A       0.024f     /* Volts per Ampere */
#define MC_SENSITIVITY_MV_PER_A      24.0f      /* Millivolts per Ampere */

/**
  * @brief ADC conversion factor (3.3V / 4095) = 0.80566 mV per LSB
  */
#define MC_ADC_TO_MV                  0.80566f

/**
  * @brief Maximum current ratings
  */
#define MC_MAX_CONTINUOUS_CURRENT_A   7.0f       /* Limited by shunt resistor power */
#define MC_MAX_PEAK_CURRENT_A         20.0f      /* Short pulse only */

/* ========================== DETECTION PARAMETERS ========================== */

/**
  * @brief Pulse duration for current injection test (milliseconds)
  *        Range: 5-15 ms, default 8 ms
  *        Lower values may not generate detectable current
  *        Higher values may cause motor movement
  */
#define MC_PULSE_DURATION_MS          8u

/**
  * @brief Cooldown period between pulses (milliseconds)
  *        Allows current to decay before next test
  *        Minimum 20 ms for bootstrap capacitor recharge
  */
#define MC_COOLDOWN_MS                25u

/**
  * @brief Pulse duty cycle percentage (0-100)
  *        Start with 5%, increase if no current detected
  *        Maximum 15% to prevent motor rotation
  *        At 8% duty and 16kHz PWM, effective voltage ≈ 8% of VDC
  */
#define MC_PULSE_DUTY_PERCENT         8u

/**
  * @brief Current detection threshold (Amperes)
  *        Values below this are considered noise
  *        MCP601 offset error ≈ ±2mV → ±83mA
  *        Threshold set to 150mA for safe detection
  */
#define MC_CURRENT_THRESHOLD_A        0.15f

/**
  * @brief ADC sampling count for calibration
  *        Higher values give better averaging
  *        Default 256 samples
  */
#define MC_CALIBRATION_SAMPLES        256u

/**
  * @brief Maximum allowed phase current during test (Amperes)
  *        Safety limit to protect IGBTs and shunt resistor
  */
#define MC_MAX_TEST_CURRENT_A         5.0f

/* ========================== ADC CHANNEL MAPPING ========================== */

/**
  * @brief ADC DMA buffer indices for phase currents
  *        Based on adc.c scan order:
  *        Rank 1: CH0 (NTC)
  *        Rank 2: CH1 (DC_Volt)
  *        Rank 3: CH2 (U_Current)  <-- PHASE U
  *        Rank 4: CH3 (V_Current)  <-- PHASE V
  *        Rank 5: CH4 (W_Current)  <-- PHASE W
  *        Rank 6: CH5 (U_Voltage)
  *        Rank 7: CH6 (V_Voltage)
  *        Rank 8: CH7 (W_Voltage)
  *        Rank 9: CH8 (DC_Current)
  */
#define MC_IDX_IU                     2u
#define MC_IDX_IV                     3u
#define MC_IDX_IW                     4u

/* ========================== ENUMERATIONS ========================== */

/**
  * @brief Phase sequence detection status
  */
typedef enum {
    MC_SEQUENCE_UNKNOWN = 0,          /* Not yet checked */
    MC_SEQUENCE_CORRECT,              /* U -> V -> W (correct order) */
    MC_SEQUENCE_REVERSED,             /* U -> W -> V (reversed order) */
    MC_SEQUENCE_ERROR                 /* Wiring fault or sensor failure */
} MC_Sequence_Status_t;

/**
  * @brief Detection method
  */
typedef enum {
    MC_METHOD_CURRENT_ONLY = 0,       /* Use current injection only */
    MC_METHOD_VOLTAGE_ONLY,           /* Use voltage measurement only */
    MC_METHOD_AUTO                    /* Auto-select best method */
} MC_Detection_Method_t;

/**
  * @brief Phase identification
  */
typedef enum {
    MC_PHASE_NONE = 0,
    MC_PHASE_U,
    MC_PHASE_V,
    MC_PHASE_W
} MC_Phase_t;

/**
  * @brief Test result for a single phase injection
  */
typedef struct {
    MC_Phase_t injected_phase;        /* Which phase received the pulse */
    float current_u_a;                /* Measured current on phase U */
    float current_v_a;                /* Measured current on phase V */
    float current_w_a;                /* Measured current on phase W */
    MC_Phase_t detected_phase;        /* Which phase responded */
    bool valid;                       /* Test was successful */
} MC_PulseResult_t;

/* ========================== CALIBRATION STRUCTURES ========================== */

/**
  * @brief Current sensor calibration data for 3 phases
  */
typedef struct {
    uint16_t offset_raw[3];           /* Raw ADC value at zero current */
    float offset_amps[3];             /* Offset in Amperes */
    float slope_av;                   /* Slope A/V (from sensitivity) */
    bool calibrated;
    uint32_t cal_timestamp_ms;
} MC_CurrentCal_t;

/* ========================== MAIN DETECTOR STRUCTURE ========================== */

/**
  * @brief Main phase sequence detector structure
  */
typedef struct {
    /* External dependencies */
    Motor_Handle_t    *motor;
    TIM_HandleTypeDef *htim;
    Meas_t            *meas;
    
    /* State */
    MC_Sequence_Status_t   status;
    MC_Detection_Method_t  method;
    MC_CurrentCal_t        cal;
    
    /* Compensation */
    bool compensation_active;         /* Auto-compensate reversed sequence */
    
    /* Statistics */
    uint32_t last_check_ms;
    uint32_t check_count;
    bool     check_passed;
    uint32_t last_error_ms;
    
    /* Current measurement buffers */
    float current_u_raw;
    float current_v_raw;
    float current_w_raw;
    float current_u_filtered;
    float current_v_filtered;
    float current_w_filtered;
    
    /* Pulse test state */
    bool pulse_active;
    uint32_t pulse_start_ms;
    MC_Phase_t current_pulse_phase;
    
} Motor_Check_t;

/* ========================== PUBLIC API ========================== */

/**
  * @brief Initialize the phase sequence detector
  * @param mc Pointer to Motor_Check structure
  * @param motor Pointer to Motor_Handle_t
  * @param htim Pointer to TIM_HandleTypeDef (TIM1 for PWM)
  * @param meas Pointer to Meas_t structure
  */
void MC_Init(Motor_Check_t *mc, 
             Motor_Handle_t *motor,
             TIM_HandleTypeDef *htim,
             Meas_t *meas);

/**
  * @brief Calibrate current sensors (U, V, W phases)
  *        Must be called when motor is IDLE and no PWM output
  *        Measures zero-current offset for each phase
  * @param mc Pointer to Motor_Check structure
  * @return true if calibration successful
  */
bool MC_CalibrateCurrentSensors(Motor_Check_t *mc);

/**
  * @brief Perform complete phase sequence check
  *        Injects test pulses on U, V, W sequentially
  *        Compares injected phase with detected phase
  * @param mc Pointer to Motor_Check structure
  * @return Phase sequence status
  */
MC_Sequence_Status_t MC_CheckSequence(Motor_Check_t *mc);

/**
  * @brief Get current phase sequence status
  * @param mc Pointer to Motor_Check structure
  * @return Current sequence status
  */
MC_Sequence_Status_t MC_GetStatus(Motor_Check_t *mc);

/**
  * @brief Enable or disable automatic compensation for reversed sequence
  * @param mc Pointer to Motor_Check structure
  * @param enable true to enable compensation
  */
void MC_SetCompensation(Motor_Check_t *mc, bool enable);

/**
  * @brief Get effective direction based on sequence status
  *        If sequence is reversed and compensation is active,
  *        the effective direction is inverted
  * @param mc Pointer to Motor_Check structure
  * @param requested_reverse User's requested direction (FWD=false, REV=true)
  * @return Effective direction to apply to motor
  */
bool MC_GetEffectiveDirection(Motor_Check_t *mc, bool requested_reverse);

/**
  * @brief Report sequence status via UART (for Bluetooth)
  * @param mc Pointer to Motor_Check structure
  */
void MC_ReportStatus(Motor_Check_t *mc);

/**
  * @brief Reset detection state (force re-check on next start)
  * @param mc Pointer to Motor_Check structure
  */
void MC_Reset(Motor_Check_t *mc);

/**
  * @brief Get phase currents from ADC and convert to Amperes
  *        Call this in fast control loop for real-time monitoring
  * @param mc Pointer to Motor_Check structure
  * @param meas Pointer to Meas_t (for ADC DMA buffer access)
  */
void MC_UpdatePhaseCurrents(Motor_Check_t *mc, Meas_t *meas);

/**
  * @brief Check if phase sequence has been verified
  * @param mc Pointer to Motor_Check structure
  * @return true if sequence is verified (correct or compensated)
  */
bool MC_IsVerified(Motor_Check_t *mc);

/**
  * @brief Get last error timestamp (for debugging)
  * @param mc Pointer to Motor_Check structure
  * @return Milliseconds since last error or 0
  */
uint32_t MC_GetLastErrorTime(Motor_Check_t *mc);

/**
  * @brief Convert raw ADC value to Amperes
  *        Uses calibrated sensitivity (24 mV/A)
  * @param mc Pointer to Motor_Check structure
  * @param adc_raw Raw ADC value (0-4095)
  * @param phase_idx Phase index (0=U, 1=V, 2=W)
  * @return Current in Amperes
  */
float MC_AdcToAmps(Motor_Check_t *mc, uint16_t adc_raw, uint8_t phase_idx);

#endif /* MOTOR_CHECK_H */