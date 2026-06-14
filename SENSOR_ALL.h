/**
 ******************************************************************************
 * @file    SENSOR_ALL.h
 * @brief   مكتبة قياس الحساسات الكاملة — PLDC Project
 *          تدعم: الحرارة (NTC) | التيار (U,V,W) | الجهد (U,V,W)
 *                جهد DC Bus | تيار DC Bus
 *
 * @version 5.0  — إصلاحات بناءً على adc.c (9 قنوات حقيقية)
 *
 *  [v5.0] إضافات في SENSOR_Handle_t:
 *    curr_gain_factor : gain تصحيحي للتيار [0.5..2.0] — 1.0=بدون تصحيح
 *    cal_gain_done    : 1 = gain معيَّن — يُفعِّل calc_current_gain في Update
 *
 *  [v5.0] SENSOR_CalibrateCurrentGain:
 *    مُطبَّقة فعلياً في SENSOR_Update عبر calc_current_gain()
 *
 *  ERORR_Pin (PA12) = إخراج فقط: LOW عند خطأ → تفعيل TIM1_BKIN الخارجي
 *
 * خريطة ADC (من adc.c):
 *   DMA[0]=NTC  DMA[1]=IU  DMA[2]=IV  DMA[3]=IW
 *   DMA[4]=VU   DMA[5]=VV  DMA[6]=VW
 *   DMA[7]=DC_VOLT  DMA[8]=DC_CURR
 ******************************************************************************
 */

#ifndef SENSOR_ALL_H
#define SENSOR_ALL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 0 — تفعيل/تعطيل مراقبة العتبات (compile-time)
 * ══════════════════════════════════════════════════════════════════════════════ */

#ifndef SENSOR_MONITOR_TEMP_EN
  #define SENSOR_MONITOR_TEMP_EN           1
#endif
#ifndef SENSOR_MONITOR_OC_U_EN
  #define SENSOR_MONITOR_OC_U_EN           1
#endif
#ifndef SENSOR_MONITOR_OC_V_EN
  #define SENSOR_MONITOR_OC_V_EN           1
#endif
#ifndef SENSOR_MONITOR_OC_W_EN
  #define SENSOR_MONITOR_OC_W_EN           1
#endif
#ifndef SENSOR_MONITOR_OVERVOLT_EN
  #define SENSOR_MONITOR_OVERVOLT_EN       1
#endif
#ifndef SENSOR_MONITOR_UNDERVOLT_EN
  #define SENSOR_MONITOR_UNDERVOLT_EN      1
#endif

/* HW_PIN = 0 نهائياً — ERORR_Pin إخراج فقط، لا يُقرأ داخل SENSOR */
#ifndef SENSOR_MONITOR_HW_PIN_EN
  #define SENSOR_MONITOR_HW_PIN_EN         0
#endif

#ifndef SENSOR_MONITOR_DC_OV_EN
  #define SENSOR_MONITOR_DC_OV_EN          1
#endif
#ifndef SENSOR_MONITOR_DC_UV_EN
  #define SENSOR_MONITOR_DC_UV_EN          1
#endif
#ifndef SENSOR_MONITOR_DC_OC_EN
  #define SENSOR_MONITOR_DC_OC_EN          1
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 1 — تعريفات الأجهزة (من adc.c)
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_ADC_RESOLUTION       4096U   /* 12-bit right-aligned */
#define SENSOR_VREF_V               3.3f    /* Vref = 3.3V */
#define SENSOR_ADC_CH_COUNT         9U      /* 9 قنوات: NTC+IU+IV+IW+VU+VV+VW+DCV+DCI */

/* فهارس DMA Buffer — مُصحَّحة لتطابق التوصيل الفعلي على PCB (صفحة 1 من المخطط)
 *
 *  الخريطة الفعلية (MCU pin → إشارة → buf index):
 *    PA0=CH0=Rank1=buf[0] → NTC
 *    PA1=CH1=Rank2=buf[1] → DC_Volt      ← الخطأ القديم: كان IDX=7
 *    PA2=CH2=Rank3=buf[2] → U_Current    ← الخطأ القديم: كان IDX=1
 *    PA3=CH3=Rank4=buf[3] → V_Current    ← الخطأ القديم: كان IDX=2
 *    PA4=CH4=Rank5=buf[4] → W_Current    ← الخطأ القديم: كان IDX=3
 *    PA5=CH5=Rank6=buf[5] → U_Voltage    ← الخطأ القديم: كان IDX=4
 *    PA6=CH6=Rank7=buf[6] → V_Voltage    ← الخطأ القديم: كان IDX=5
 *    PA7=CH7=Rank8=buf[7] → W_Voltage    ← الخطأ القديم: كان IDX=6
 *    PB0=CH8=Rank9=buf[8] → DC_Current
 */
#define SENSOR_IDX_NTC              0U      /* CH0 PA0 — Rank 1 — NTC */
#define SENSOR_IDX_DC_VOLT          1U      /* CH1 PA1 — Rank 2 — DC_Volt */
#define SENSOR_IDX_IU               2U      /* CH2 PA2 — Rank 3 — U_Current */
#define SENSOR_IDX_IV               3U      /* CH3 PA3 — Rank 4 — V_Current */
#define SENSOR_IDX_IW               4U      /* CH4 PA4 — Rank 5 — W_Current */
#define SENSOR_IDX_VU               5U      /* CH5 PA5 — Rank 6 — U_Voltage */
#define SENSOR_IDX_VV               6U      /* CH6 PA6 — Rank 7 — V_Voltage */
#define SENSOR_IDX_VW               7U      /* CH7 PA7 — Rank 8 — W_Voltage */
#define SENSOR_IDX_DC_CURR          8U      /* CH8 PB0 — Rank 9 — DC_Current */

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 2 — معاملات NTC Thermistor (PA0, R29=47kΩ pull-up → 3.3V)
 *
 *  [UPDATED] NTC = 50kΩ @ 25°C  |  R_pullup = 47kΩ (R29 على PCB — لم يتغير)
 *
 *  تشخيص الخطأ (التعديل السابق):
 *    الكود كان يستخدم R_pullup=4.7kΩ بينما الهاردوير الفعلي R29=47kΩ
 *    → ADC_raw عند 28.6°C مع HW=47k يساوي ≈1944
 *    → calc_temperature بـ R_pullup=4.7k تعطي ≈90°C (خطأ)
 *    → الإصلاح: R_pullup يعكس قيمة R29 الفعلية على PCB = 47kΩ
 *
 *  ADC_WD HighThreshold = 2730 ≈ 66°C (محسوب بـ NTC=50k, Rp=47k):
 *    R_NTC@66°C = 50000 × exp(4092×(1/339.15 − 1/298.15)) ≈  9520 Ω
 *    V_NTC@66°C = 3.3 × 9520 / (47000 + 9520)             ≈  0.556 V
 *    ADC_raw    = 0.556 × 4096 / 3.3                       ≈   690
 *  ملاحظة: threshold في adc.c يبقى كما هو (ملف CubeMX)
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_NTC_R_PULLUP_OHM     47000.0f   /* R29 = 47kΩ على PCB → 3.3V */
#define SENSOR_NTC_R25_OHM          50000.0f   /* مقاومة NTC عند 25°C = 50kΩ */
#define SENSOR_NTC_B_CONST          4092.0f    /* معامل Beta (بدون تغيير) */
#define SENSOR_NTC_T0_K             298.15f    /* 25°C بالكلفن (بدون تغيير) */

/* ADC Watchdog في adc.c: HighThreshold=2730 (للمعلومية — يُضبط في adc.c) */
#define SENSOR_ADC_WD_THRESHOLD     2730U      /* للمعلومية — يُضبط في adc.c */
#define SENSOR_ADC_WD_TEMP_C        66.0f      /* درجة حرارة موافقة للـ Watchdog */

#ifndef SENSOR_TEMP_WARN_C
  #define SENSOR_TEMP_WARN_C        75.0f
#endif
#ifndef SENSOR_TEMP_FAULT_C
  #define SENSOR_TEMP_FAULT_C       85.0f      /* حد الإيقاف البرمجي (> ADC WD) */
#endif
#ifndef SENSOR_TEMP_MAX_C
  #define SENSOR_TEMP_MAX_C         125.0f
#endif
#ifndef SENSOR_TEMP_HYST_C
  #define SENSOR_TEMP_HYST_C        10.0f
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 3 — معاملات التيار (PA1/PA2/PA3 — R26/R27/R28=0.02Ω + MCP601 Gain=5)
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_SHUNT_OHM            0.02f
#define SENSOR_CURRENT_GAIN         5.0f
#define SENSOR_CURRENT_SCALE_A_LSB  (SENSOR_VREF_V / \
                                    ((float)SENSOR_ADC_RESOLUTION * \
                                     SENSOR_SHUNT_OHM * SENSOR_CURRENT_GAIN))
#define SENSOR_CURRENT_OFFSET_LSB   0U

#ifndef SENSOR_CURR_PHASE_DEFAULT_A
  #define SENSOR_CURR_PHASE_DEFAULT_A  15.0f
#endif
#ifndef SENSOR_CURR_DC_DEFAULT_A
  #define SENSOR_CURR_DC_DEFAULT_A     20.0f
#endif
#ifndef SENSOR_CURR_ABS_MAX_A
  #define SENSOR_CURR_ABS_MAX_A        25.0f
#endif
#ifndef SENSOR_CURR_WARN_RATIO
  #define SENSOR_CURR_WARN_RATIO       0.80f
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 4 — معاملات جهد الأطوار (PA4/PA5/PA6 — 1360kΩ÷10kΩ + MCP601T)
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_VOLT_R_TOP_OHM       1360000.0f
#define SENSOR_VOLT_R_BOT_OHM       10000.0f
#define SENSOR_VOLT_K               ((SENSOR_VOLT_R_TOP_OHM + \
                                      SENSOR_VOLT_R_BOT_OHM) / \
                                      SENSOR_VOLT_R_BOT_OHM)
#define SENSOR_VOLT_SCALE_V_LSB     (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION \
                                     * SENSOR_VOLT_K)

#ifndef SENSOR_VOLT_FAULT_V
  #define SENSOR_VOLT_FAULT_V       450.0f
#endif
#ifndef SENSOR_VOLT_WARN_V
  #define SENSOR_VOLT_WARN_V        430.0f
#endif
#ifndef SENSOR_VOLT_UNDER_V
  #define SENSOR_VOLT_UNDER_V       200.0f
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 4B — معاملات جهد DC Bus (PA7 — U12/MCP601T voltage follower)
 *
 *  دائرة المقسم الفعلية (من المخطط صفحة 4):
 *    B+_340V → R54(680kΩ) → R53(680kΩ) → R52(1kΩ) → [نقطة A] → R55(10kΩ) → GND
 *    U12 IN+ ← نقطة A  |  U12 Unity-Gain Follower → DC_Volt → PA7
 *
 *    R_top = R54 + R53 + R52 = 680k + 680k + 1k = 1361 kΩ
 *    R_bot = R55               =  10 kΩ
 *    K     = (1361k + 10k) / 10k = 137.100
 *
 *  [FIX] الكود القديم كان: R_top=1360kΩ, R_bot=11kΩ → K=124.64 (خاطئ)
 *        عند 316V الحقيقي كان يعطي ≈287V — تم التصحيح بقيم المخطط الفعلية
 *
 *  تحقق: عند V_real=316V → V_ADC=2.305V → ADC_raw≈2861
 *         V_calc = 2861 × (3.3/4096) × 137.1 = 316.0V ✓
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_DC_VOLT_R_TOP_OHM    1361000.0f  /* R54+R53+R52 = 680k+680k+1k */
#define SENSOR_DC_VOLT_R_BOT_OHM    10000.0f    /* R55 = 10kΩ */
#define SENSOR_DC_VOLT_K            ((SENSOR_DC_VOLT_R_TOP_OHM + \
                                      SENSOR_DC_VOLT_R_BOT_OHM) / \
                                      SENSOR_DC_VOLT_R_BOT_OHM)
#define SENSOR_DC_VOLT_SCALE_V_LSB  (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION \
                                     * SENSOR_DC_VOLT_K)

#ifndef SENSOR_DC_VOLT_FAULT_V
  #define SENSOR_DC_VOLT_FAULT_V    450.0f
#endif
#ifndef SENSOR_DC_VOLT_WARN_V
  #define SENSOR_DC_VOLT_WARN_V     430.0f
#endif
#ifndef SENSOR_DC_VOLT_UNDER_V
  #define SENSOR_DC_VOLT_UNDER_V    200.0f
#endif

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 4C — معاملات تيار DC Bus (PB0 — R46=0.02Ω + MCP602 Gain=5)
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_DC_CURR_SHUNT_OHM    0.02f
#define SENSOR_DC_CURR_GAIN         5.0f
#define SENSOR_DC_CURR_SCALE_A_LSB  (SENSOR_VREF_V / \
                                    ((float)SENSOR_ADC_RESOLUTION * \
                                     SENSOR_DC_CURR_SHUNT_OHM * SENSOR_DC_CURR_GAIN))

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 5 — إشارة الخطأ ERORR_Pin (PA12)
 *  إخراج فقط — يُكتب في main.c بناءً على threshold_exceeded
 * ══════════════════════════════════════════════════════════════════════════════ */

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 6 — فلترة Moving Average
 * ══════════════════════════════════════════════════════════════════════════════ */

#define SENSOR_FILTER_SIZE          8U

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 7 — هياكل البيانات
 * ══════════════════════════════════════════════════════════════════════════════ */

typedef enum
{
    SENSOR_MASK_TEMP         = 0x001U,
    SENSOR_MASK_OC_U         = 0x002U,
    SENSOR_MASK_OC_V         = 0x004U,
    SENSOR_MASK_OC_W         = 0x008U,
    SENSOR_MASK_OVERVOLT     = 0x010U,
    SENSOR_MASK_UNDERVOLT    = 0x020U,
    SENSOR_MASK_HW_PIN       = 0x040U,  /* محجوز — معطَّل دائماً */
    SENSOR_MASK_DC_OV        = 0x200U,
    SENSOR_MASK_DC_UV        = 0x400U,
    SENSOR_MASK_DC_OC        = 0x800U,
    SENSOR_MASK_ALL          = 0xFFFU,
    SENSOR_MASK_NONE         = 0x000U,
} SENSOR_MonitorMask_t;

typedef struct
{
    float    temperature_C;
    float    current_U_A;
    float    current_V_A;
    float    current_W_A;
    float    voltage_U_V;
    float    voltage_V_V;
    float    voltage_W_V;
    float    current_max_A;     /* أعلى تيار بين U/V/W */
    float    voltage_avg_V;     /* متوسط جهد U/V/W */
    float    dc_voltage_V;
    float    dc_current_A;
    uint8_t  threshold_exceeded;   /* 1 = تجاوز عتبة → main.c يُوقف ويكتب ERORR_Pin */
} SENSOR_Data_t;

typedef struct
{
    /* ── مصدر البيانات ── */
    volatile uint16_t *adc_buf;     /* مؤشر لـ DMA buffer (9 عناصر) */
    SENSOR_Data_t      data;        /* القيم المحسوبة والمفلترة */

    /* ── فلتر Moving Average ── */
    uint16_t filt_buf[SENSOR_ADC_CH_COUNT][SENSOR_FILTER_SIZE];
    uint8_t  filt_idx;              /* مؤشر الكتابة الحالي */
    uint8_t  filt_ready;            /* 1 = دورة كاملة أُنجزت */

    /* ── حدود الحماية (قابلة للضبط من BT) ── */
    float    limit_temp_fault;
    float    limit_curr_fault;
    float    limit_dc_curr_fault;
    float    limit_volt_fault;
    float    limit_volt_under;
    float    limit_dc_volt_fault;
    float    limit_dc_volt_under;

    /* ── قناع المراقبة ── */
    uint16_t monitor_mask;
    uint8_t  initialized;

    /* ── معايرة التيار (offset الصفر) ── */
    uint16_t offset_iu;
    uint16_t offset_iv;
    uint16_t offset_iw;
    uint16_t offset_dci;
    uint8_t  cal_done;              /* 1 = تمت معايرة offset */

    /* ── [v5.0] gain التيار التصحيحي ── */
    float    curr_gain_factor;      /* gain كلي للأطوار [0.5..2.0] — 1.0=بدون تصحيح */
    uint8_t  cal_gain_done;         /* 1 = gain مُعيَّن → SENSOR_Update يستخدم calc_current_gain */

    /* ── معايرة NTC ── */
    float    ntc_offset_c;          /* offset حراري [°C] */
    float    ntc_gain;              /* gain حراري [1.0=بدون تصحيح] */
    uint8_t  ntc_cal_done;

    /* ── حالة المعايرة الكلية ── */
    uint8_t  cal_source;            /* 0=لا شيء 1=تلقائي 2=يدوي */

} SENSOR_Handle_t;

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 8 — Core API
 * ══════════════════════════════════════════════════════════════════════════════ */

bool     SENSOR_Init           (SENSOR_Handle_t *hsen, volatile uint16_t *adc_buf);
void     SENSOR_Update         (SENSOR_Handle_t *hsen);
void     SENSOR_GetData        (const SENSOR_Handle_t *hsen, SENSOR_Data_t *out);
void     SENSOR_UpdateHWProtect(SENSOR_Handle_t *hsen);

void     SENSOR_SetLimits      (SENSOR_Handle_t *hsen,
                                float temp_fault, float curr_fault,
                                float volt_fault, float volt_under);

/**
 * @brief  ضبط حدود التيار
 * @note   -1.0f = "لا تغيير" لهذا الطرف
 *         0.0f  = صفر (حماية وقائية)
 */
void     SENSOR_SetCurrentLimits(SENSOR_Handle_t *hsen,
                                 float phase_max_a,
                                 float dc_max_a);

void     SENSOR_SetVoltageLimits  (SENSOR_Handle_t *hsen,
                                   float volt_under_v, float volt_fault_v);
void     SENSOR_SetDCVoltageLimits(SENSOR_Handle_t *hsen,
                                   float dc_under_v,  float dc_fault_v);
void     SENSOR_SetTempFaultLimit (SENSOR_Handle_t *hsen, float temp_max_c);

void     SENSOR_MonitorEnable    (SENSOR_Handle_t *hsen, uint16_t mask);
void     SENSOR_MonitorDisable   (SENSOR_Handle_t *hsen, uint16_t mask);
void     SENSOR_MonitorEnableAll (SENSOR_Handle_t *hsen);
void     SENSOR_MonitorDisableAll(SENSOR_Handle_t *hsen);
uint16_t SENSOR_MonitorGetMask   (const SENSOR_Handle_t *hsen);

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 9 — Calibration API
 * ══════════════════════════════════════════════════════════════════════════════ */

void SENSOR_CalibrateCurrentOffset(SENSOR_Handle_t *hsen);
void SENSOR_CalibrateCurrentManual(SENSOR_Handle_t *hsen, float ref_current_a);

/**
 * @brief  [v5.0 FIXED] معايرة gain التيار بنقطتَين — مُطبَّقة في SENSOR_Update
 * @param  measured_a   التيار المقاس حالياً بعد offset [A]
 * @param  reference_a  التيار الحقيقي من الأمبيرمتر [A]
 * @note   يُعيِّن curr_gain_factor و cal_gain_done=1
 *         SENSOR_Update سيستخدم calc_current_gain() تلقائياً
 */
void SENSOR_CalibrateCurrentGain  (SENSOR_Handle_t *hsen,
                                   float measured_a, float reference_a);

void SENSOR_CalibrateNTC          (SENSOR_Handle_t *hsen, float ref_temp_c);
void SENSOR_CalibrationReset      (SENSOR_Handle_t *hsen);
void SENSOR_GetCalibrationStatus  (const SENSOR_Handle_t *hsen,
                                   float *out_curr_off,
                                   float *out_ntc_off,
                                   float *out_ntc_gain);

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 10 — JSON Builder (مُستخدَمة من UPGRADE_FillTelemetry)
 * ══════════════════════════════════════════════════════════════════════════════ */

int SENSOR_BuildJSON  (const SENSOR_Handle_t *hsen, char *buf, uint16_t len);
int SENSOR_BuildJSONEx(const SENSOR_Handle_t *hsen, char *buf, uint16_t len);

/* ══════════════════════════════════════════════════════════════════════════════
 *  SECTION 11 — Inline helpers (للتوافق)
 * ══════════════════════════════════════════════════════════════════════════════ */

static inline void SENSOR_ClearFaults(SENSOR_Handle_t *hsen)
{
    if (hsen != NULL) {
        hsen->data.threshold_exceeded = 0U;
        SENSOR_UpdateHWProtect(hsen);
    }
}

static inline bool SENSOR_HasFault(const SENSOR_Handle_t *hsen)
{
    if (hsen == NULL) return false;
    return (hsen->data.threshold_exceeded != 0U);
}

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_ALL_H */
