/**
  ******************************************************************************
  * @file    Motor_Safety.h
  * @brief   Motor Safety Test & Health Diagnostics Module
  *          الاختبار الأمني عند بدء التشغيل والتشخيص الشامل
  *
  * @details عند بدء التشغيل:
  *          1. اختبر قراءات الجهد والتيار للأطوار الثلاثة
  *          2. تحقق من حرارة النظام
  *          3. افحص جهد DC Bus
  *          4. كرر الاختبار لمدة ~1 ثانية وحسب المتوسطات
  *          5. قيّم النتائج واتخذ القرار: تشغيل/إيقاف مع تقرير
  *
  * @version 2.0 — مع ERORR_Pin latching logic
  ******************************************************************************
  */

#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Test Configuration                                                         */
/* ═══════════════════════════════════════════════════════════════════════════ */

#define MOTOR_SAFETY_TEST_DURATION_MS    1000U   /* قراءات لمدة 1 ثانية */
#define MOTOR_SAFETY_SAMPLE_INTERVAL_MS  100U    /* فاصل 100 ملي بين العينات */
#define MOTOR_SAFETY_MAX_SAMPLES         10U     /* 10 عينات في الثانية */

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Thresholds (من motor.h + SENSOR_ALL.h)                                   */
/* ═══════════════════════════════════════════════════════════════════════════ */

#define MOTOR_SAFETY_TEMP_CRITICAL      75.0f   /* حد الخطأ — OT */
#define MOTOR_SAFETY_VOLT_UNDER         150.0f  /* حد الانخفاض — UV */
#define MOTOR_SAFETY_VOLT_OVER          420.0f  /* حد الارتفاع — OV */
#define MOTOR_SAFETY_CURRENT_OVER       10.0f   /* حد التيار — OC (A) */

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Fault Codes / Error Messages                                               */
/* ═══════════════════════════════════════════════════════════════════════════ */

typedef enum
{
    MOTOR_SAFETY_OK = 0,
    MOTOR_SAFETY_FAULT_OT,         /* Over Temperature */
    MOTOR_SAFETY_FAULT_UV,         /* Under Voltage */
    MOTOR_SAFETY_FAULT_OV,         /* Over Voltage */
    MOTOR_SAFETY_FAULT_OC,         /* Over Current */
    MOTOR_SAFETY_FAULT_UNKNOWN
} Motor_Safety_Fault_t;

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Test State Machine                                                         */
/* ═══════════════════════════════════════════════════════════════════════════ */

typedef enum
{
    MOTOR_SAFETY_IDLE = 0,
    MOTOR_SAFETY_TESTING,
    MOTOR_SAFETY_COMPLETE,
    MOTOR_SAFETY_FAULT
} Motor_Safety_State_t;

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Data Structures                                                            */
/* ═══════════════════════════════════════════════════════════════════════════ */

/* Sample container for a single test point */
typedef struct
{
    uint32_t timestamp_ms;
    float temperature_c;
    float voltage_u;
    float voltage_v;
    float voltage_w;
    float current_u;
    float current_v;
    float current_w;
    float dc_voltage;
    float dc_current;
} Motor_Safety_Sample_t;

/* Statistics / aggregated results */
typedef struct
{
    /* Averages */
    float temp_avg;
    float vu_avg, vv_avg, vw_avg;
    float iu_avg, iv_avg, iw_avg;
    float dc_volt_avg, dc_curr_avg;
    
    /* Max/Min */
    float temp_max, temp_min;
    float volt_max, volt_min;
    float curr_max, curr_min;
    
    /* Sample count */
    uint8_t sample_count;
} Motor_Safety_Stats_t;

/* Main handle structure */
typedef struct
{
    /* State machine */
    Motor_Safety_State_t state;
    uint32_t start_time_ms;
    uint32_t last_sample_ms;
    
    /* Sample buffer */
    Motor_Safety_Sample_t samples[MOTOR_SAFETY_MAX_SAMPLES];
    uint8_t sample_index;
    
    /* Statistics */
    Motor_Safety_Stats_t stats;
    
    /* Fault information */
    Motor_Safety_Fault_t fault_code;
    const char *fault_message;
    
    /* Latching error state (من ERORR_Pin) */
    uint8_t error_latched;      /* 1 = خطأ مكبوت إلى أن يمسح من HMI */
    uint32_t error_latch_time_ms;
    
    /* External data sources (pointers) */
    const volatile uint16_t *adc_buf;   /* من app.c: g_adc_dma */
    void (*write_error_pin)(uint8_t state);  /* callback لكتابة ERORR_Pin */
    void (*send_message)(const char *msg);   /* callback لإرسال رسائل للواجهة */
    
    /* Calibration (من SENSOR_ALL) */
    uint16_t offset_iu, offset_iv, offset_iw, offset_dci;
    
} Motor_Safety_t;

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  Public API                                                                 */
/* ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the motor safety module
 * @param ms Pointer to Motor_Safety_t structure
 * @param adc_buf Pointer to ADC DMA buffer (9 channels)
 * @return true on success
 */
bool Motor_Safety_Init(Motor_Safety_t *ms, const volatile uint16_t *adc_buf);

/**
 * @brief Start a safety test sequence
 * @param ms Pointer to Motor_Safety_t structure
 * @return true if test started successfully
 */
bool Motor_Safety_StartTest(Motor_Safety_t *ms);

/**
 * @brief Run the test state machine (call periodically from main loop)
 * @param ms Pointer to Motor_Safety_t structure
 * @return true if test is still in progress, false if complete/idle
 */
bool Motor_Safety_Task(Motor_Safety_t *ms);

/**
 * @brief Get current test state
 */
Motor_Safety_State_t Motor_Safety_GetState(const Motor_Safety_t *ms);

/**
 * @brief Get fault code if test failed
 */
Motor_Safety_Fault_t Motor_Safety_GetFault(const Motor_Safety_t *ms);

/**
 * @brief Get error message string
 */
const char *Motor_Safety_GetMessage(const Motor_Safety_t *ms);

/**
 * @brief Get statistics from last test
 */
void Motor_Safety_GetStats(const Motor_Safety_t *ms, Motor_Safety_Stats_t *out);

/**
 * @brief Set error pin write callback
 */
void Motor_Safety_SetErrorPinCallback(Motor_Safety_t *ms,
                                       void (*callback)(uint8_t));

/**
 * @brief Set message send callback (for HMI)
 */
void Motor_Safety_SetMessageCallback(Motor_Safety_t *ms,
                                      void (*callback)(const char *));

/**
 * @brief Set current sensor offsets (from calibration)
 */
void Motor_Safety_SetCurrentOffsets(Motor_Safety_t *ms,
                                     uint16_t iu, uint16_t iv,
                                     uint16_t iw, uint16_t dci);

/**
 * @brief Check if error is latched
 */
bool Motor_Safety_IsErrorLatched(const Motor_Safety_t *ms);

/**
 * @brief Clear latched error (called from HMI clear command)
 */
void Motor_Safety_ClearError(Motor_Safety_t *ms);

/**
 * @brief Get time since error was latched
 */
uint32_t Motor_Safety_GetErrorAge(const Motor_Safety_t *ms);

/**
 * @brief Build JSON report of last test
 */
int Motor_Safety_BuildJSON(const Motor_Safety_t *ms, char *buf, size_t len);

#endif /* MOTOR_SAFETY_H */
