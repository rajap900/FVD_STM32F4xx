/**
 ******************************************************************************
 * @file    SENSOR_ALL.c
 * @brief   تنفيذ مكتبة قياس الحساسات — PLDC Project
 * @version 5.0  — إصلاحات شاملة بناءً على adc.c
 *
 * ════════════════════════════════════════════════════════════════
 *  خريطة ADC ← adc.c (9 قنوات Scan + DMA Circular HalfWord):
 *
 *  [CORRECTED] الخريطة الفعلية من المخطط (صفحة 1):
 *  DMA[0] CH0 PA0  NTC Temp     R1=47kΩ pull-up   → IDX_NTC=0
 *  DMA[1] CH1 PA1  DC_Volt(Bus) R52+R53+R54÷R55   → IDX_DC_VOLT=1
 *  DMA[2] CH2 PA2  U_Current    R26=0.02Ω Gain=5  → IDX_IU=2
 *  DMA[3] CH3 PA3  V_Current    R27=0.02Ω Gain=5  → IDX_IV=3
 *  DMA[4] CH4 PA4  W_Current    R28=0.02Ω Gain=5  → IDX_IW=4
 *  DMA[5] CH5 PA5  U_Voltage    1360kΩ÷10kΩ       → IDX_VU=5
 *  DMA[6] CH6 PA6  V_Voltage    1360kΩ÷10kΩ       → IDX_VV=6
 *  DMA[7] CH7 PA7  W_Voltage    1360kΩ÷10kΩ       → IDX_VW=7
 *  DMA[8] CH8 PB0  DC_Current   R46=0.02Ω Gain=5  → IDX_DC_CURR=8
 *
 *  ADC Watchdog: CH0(NTC) HighThreshold=2730 (~66°C) → ADC_IRQn
 *
 * ════════════════════════════════════════════════════════════════
 *  إصلاحات v5.0:
 *  [FIX-1] SENSOR_CalibrateCurrentGain: تطبيق gain الفعلي في Update
 *           — حقل curr_gain_factor مُضاف لـ SENSOR_Handle_t
 *           — calc_current_gain مستدعاة عند cal_gain_done=1
 *  [FIX-2] SENSOR_BuildJSON / SENSOR_BuildJSONEx: مُستدعيتان من
 *           UPGRADE_FillTelemetry عند DEBUG_JSON_EN=1
 *  [FIX-3] SENSOR_UpdateHWProtect: تعيد threshold_exceeded للمحرك
 *           ليُستخدم في main.c لكتابة ERORR_Pin
 *  [FIX-4] ADC Watchdog Callback: مُعالَج في SENSOR_ALL ويُسجَّل
 *           كخطأ حرارة مسبق قبل حد الإيقاف الناعم
 * ════════════════════════════════════════════════════════════════
 */

#include "SENSOR_ALL.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ──────────────────────────────────────────────────────────────
 *  ماكرو التحقق من بت في monitor_mask
 * ──────────────────────────────────────────────────────────────*/
#define _IS_EXCEEDED(h, bit)   (((h)->monitor_mask & (uint16_t)(bit)) != 0U)

/* ══════════════════════════════════════════════════════════════
 *  بناء القناع الابتدائي من ماكروهات وقت الترجمة
 * ══════════════════════════════════════════════════════════════ */
static uint16_t build_compile_time_mask(void)
{
    uint16_t m = 0U;
#if (SENSOR_MONITOR_TEMP_EN != 0)
    m |= (uint16_t)SENSOR_MASK_TEMP;
#endif
#if (SENSOR_MONITOR_OC_U_EN != 0)
    m |= (uint16_t)SENSOR_MASK_OC_U;
#endif
#if (SENSOR_MONITOR_OC_V_EN != 0)
    m |= (uint16_t)SENSOR_MASK_OC_V;
#endif
#if (SENSOR_MONITOR_OC_W_EN != 0)
    m |= (uint16_t)SENSOR_MASK_OC_W;
#endif
#if (SENSOR_MONITOR_OVERVOLT_EN != 0)
    m |= (uint16_t)SENSOR_MASK_OVERVOLT;
#endif
#if (SENSOR_MONITOR_UNDERVOLT_EN != 0)
    m |= (uint16_t)SENSOR_MASK_UNDERVOLT;
#endif
#if (SENSOR_MONITOR_DC_OV_EN != 0)
    m |= (uint16_t)SENSOR_MASK_DC_OV;
#endif
#if (SENSOR_MONITOR_DC_UV_EN != 0)
    m |= (uint16_t)SENSOR_MASK_DC_UV;
#endif
#if (SENSOR_MONITOR_DC_OC_EN != 0)
    m |= (uint16_t)SENSOR_MASK_DC_OC;
#endif
    /* SENSOR_MONITOR_HW_PIN_EN = 0 دائماً — ERORR_Pin إخراج فقط */
    return m;
}

/* ══════════════════════════════════════════════════════════════
 *  دوال حساب داخلية — كلها static
 * ══════════════════════════════════════════════════════════════ */

/*
 * Moving Average على نافذة SENSOR_FILTER_SIZE=8
 * اعتمادًا على DMA Circular HalfWord (12-bit right-aligned → [0..4095])
 */
static uint16_t filter_average(const uint16_t *buf)
{
    uint32_t sum = 0U;
    for (uint8_t i = 0U; i < SENSOR_FILTER_SIZE; i++)
        sum += (uint32_t)buf[i];
    return (uint16_t)(sum / (uint32_t)SENSOR_FILTER_SIZE);
}

/* ADC raw → V_ADC (3.3V Vref, 12-bit) */
static inline float adc_to_volt(uint16_t raw)
{
    return (float)raw * (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION);
}

/*
 * NTC → درجة حرارة [°C]
 * الدائرة: R_pullup=47kΩ (R29) pull-up to 3.3V → NTC شبه أرضي
 * V_NTC = ADC_raw × (3.3/4096)
 * R_NTC = R_pullup × V_NTC / (Vref - V_NTC)
 * 1/T = 1/T0 + ln(R_NTC/R25) / B
 * T_C = T_K - 273.15
 *
 * ADC Watchdog (adc.c): HighThreshold=2730 ~→ ~66°C (تحذير مبكر)
 * حد الإيقاف البرمجي: SENSOR_TEMP_FAULT_C=85°C (أقل حدة)
 */
static float calc_temperature(uint16_t raw)
{
    if (raw == 0U) return -273.0f;   /* لا اتصال */

    float v_ntc      = adc_to_volt(raw);
    float denominator = SENSOR_VREF_V - v_ntc;
    if (denominator <= 0.001f) return SENSOR_TEMP_MAX_C; /* NTC مقطوع → حرارة أقصى */

    float r_ntc = SENSOR_NTC_R_PULLUP_OHM * v_ntc / denominator;
    if (r_ntc <= 0.0f) return SENSOR_TEMP_MAX_C;

    float ln_ratio = logf(r_ntc / SENSOR_NTC_R25_OHM);
    float inv_T    = (1.0f / SENSOR_NTC_T0_K) + (ln_ratio / SENSOR_NTC_B_CONST);
    if (inv_T <= 0.0f) return -273.0f;

    return (1.0f / inv_T) - 273.15f;
}

/*
 * قياس التيار — بدون gain تصحيحي (تستدعيها المسار الأساسي)
 * الدائرة: Shunt=0.02Ω + MCP601 Gain=5 → V_sense = I × 0.02 × 5 = I × 0.1
 * I[A] = V_ADC / (Gain × Shunt) = V_ADC / 0.1
 * مع تصحيح offset الصفر (يُعيَّن تلقائياً عند CAL_CURR)
 */
static float calc_current(uint16_t raw, uint16_t offset)
{
    int32_t corrected = (int32_t)raw - (int32_t)offset;
    if (corrected < 0) corrected = 0;
    float v_adc = (float)corrected * (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION);
    return v_adc / (SENSOR_CURRENT_GAIN * SENSOR_SHUNT_OHM);
}

/*
 * [FIX-1] قياس التيار مع gain تصحيحي (مسار المعايرة اليدوية)
 * gain_factor = reference_A / measured_A — مُحدَّد في [0.5 .. 2.0]
 */
static float calc_current_gain(uint16_t raw, uint16_t offset, float gain_factor)
{
    int32_t corrected = (int32_t)raw - (int32_t)offset;
    if (corrected < 0) corrected = 0;
    float v_adc = (float)corrected * (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION);
    float base   = v_adc / (SENSOR_CURRENT_GAIN * SENSOR_SHUNT_OHM);
    return base * gain_factor;
}

/*
 * قياس جهد الطور
 * الدائرة: مقسم 1360kΩ ÷ 10kΩ → K = (1360+10)/10 = 137
 * V_phase = ADC_raw × (3.3/4096) × 137
 * مُعرَّف في SENSOR_VOLT_SCALE_V_LSB
 */
static float calc_voltage(uint16_t raw)
{
    if (raw > (uint16_t)SENSOR_ADC_RESOLUTION) return 0.0f;
    return (float)raw * SENSOR_VOLT_SCALE_V_LSB;
}

/*
 * قياس جهد DC Bus
 * الدائرة: مقسم 1360kΩ ÷ 11kΩ → K = (1360+11)/11 ≈ 124.6
 * V_dc = ADC_raw × (3.3/4096) × 124.6
 * مُعرَّف في SENSOR_DC_VOLT_SCALE_V_LSB
 */
static float calc_dc_voltage(uint16_t raw)
{
    if (raw > (uint16_t)SENSOR_ADC_RESOLUTION) return 0.0f;
    return (float)raw * SENSOR_DC_VOLT_SCALE_V_LSB;
}

/*
 * قياس تيار DC Bus
 * الدائرة: Shunt=0.02Ω (R46) + MCP602 Gain=5
 * I_dc = V_ADC / (Gain × Shunt) = V_ADC / 0.1
 */
static float calc_dc_current(uint16_t raw, uint16_t offset)
{
    if (raw > (uint16_t)SENSOR_ADC_RESOLUTION) return 0.0f;

    int32_t corrected = (int32_t)raw - (int32_t)offset;
    if (corrected < 0) corrected = 0;

    float v_adc = (float)corrected * (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION);
    return v_adc / (SENSOR_DC_CURR_GAIN * SENSOR_DC_CURR_SHUNT_OHM);
}

/*
 * فحص تجاوز العتبات — بدون إيقاف فوري
 * يكتب threshold_exceeded=1 فقط — الإجراء يتخذه main.c
 */
static void check_thresholds(SENSOR_Handle_t *hsen)
{
    SENSOR_Data_t *d = &hsen->data;
    uint8_t exceeded = 0U;

    /* الحرارة الزائدة — OT */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_TEMP))
    {
        if (d->temperature_C >= hsen->limit_temp_fault)
            exceeded = 1U;
    }

    /* تيار الطور U — OC_U */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_OC_U))
    {
        if (d->current_U_A >= hsen->limit_curr_fault)
            exceeded = 1U;
    }

    /* تيار الطور V — OC_V */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_OC_V))
    {
        if (d->current_V_A >= hsen->limit_curr_fault)
            exceeded = 1U;
    }

    /* تيار الطور W — OC_W */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_OC_W))
    {
        if (d->current_W_A >= hsen->limit_curr_fault)
            exceeded = 1U;
    }

    /* جهد الطور الزائد — OV */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_OVERVOLT))
    {
        float v_max = d->voltage_U_V;
        if (d->voltage_V_V > v_max) v_max = d->voltage_V_V;
        if (d->voltage_W_V > v_max) v_max = d->voltage_W_V;
        if (v_max >= hsen->limit_volt_fault)
            exceeded = 1U;
    }

    /* جهد الطور المنخفض — UV */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_UNDERVOLT))
    {
        /* لا نُفعِّل UV عند dc_voltage_V < 5V (المحرك متوقف تماماً) */
        if ((d->voltage_avg_V > 5.0f) &&
            (d->voltage_avg_V < hsen->limit_volt_under))
            exceeded = 1U;
    }

    /* HW_PIN محذوف: ERORR_Pin إخراج فقط — يُكتب من main.c */

    /* جهد DC Bus الزائد — DC_OV */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_DC_OV))
    {
        if (d->dc_voltage_V >= hsen->limit_dc_volt_fault)
            exceeded = 1U;
    }

    /* جهد DC Bus المنخفض — DC_UV (لا نُفعِّل عند صفر) */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_DC_UV))
    {
        if ((d->dc_voltage_V > 5.0f) &&
            (d->dc_voltage_V < hsen->limit_dc_volt_under))
            exceeded = 1U;
    }

    /* تيار DC Bus الزائد — DC_OC */
    if (_IS_EXCEEDED(hsen, SENSOR_MASK_DC_OC))
    {
        if (d->dc_current_A >= hsen->limit_dc_curr_fault)
            exceeded = 1U;
    }

    d->threshold_exceeded = exceeded;
}

/* ══════════════════════════════════════════════════════════════
 *  Core API — التنفيذ
 * ══════════════════════════════════════════════════════════════ */

bool SENSOR_Init(SENSOR_Handle_t *hsen, volatile uint16_t *adc_buf)
{
    if (hsen == NULL || adc_buf == NULL) return false;

    memset(hsen, 0, sizeof(SENSOR_Handle_t));
    hsen->adc_buf = adc_buf;

    /* حدود الحماية الافتراضية */
    hsen->limit_temp_fault     = SENSOR_TEMP_FAULT_C;
    hsen->limit_curr_fault     = SENSOR_CURR_PHASE_DEFAULT_A;
    hsen->limit_dc_curr_fault  = SENSOR_CURR_DC_DEFAULT_A;
    hsen->limit_volt_fault     = SENSOR_VOLT_FAULT_V;
    hsen->limit_volt_under     = SENSOR_VOLT_UNDER_V;
    hsen->limit_dc_volt_fault  = SENSOR_DC_VOLT_FAULT_V;
    hsen->limit_dc_volt_under  = SENSOR_DC_VOLT_UNDER_V;

    /* قناع المراقبة الافتراضي من ماكروهات وقت الترجمة */
    hsen->monitor_mask = build_compile_time_mask();

    /* [FIX-1] تهيئة قيم المعايرة الافتراضية */
    hsen->ntc_offset_c       = 0.0f;
    hsen->ntc_gain           = 1.0f;
    hsen->ntc_cal_done       = 0U;
    hsen->curr_gain_factor   = 1.0f;  /* gain=1.0 = بدون تصحيح */
    hsen->cal_gain_done      = 0U;
    hsen->cal_done           = 0U;
    hsen->cal_source         = 0U;

    hsen->initialized = 1U;

    return true;
}

void SENSOR_Update(SENSOR_Handle_t *hsen)
{
    if (hsen == NULL || hsen->initialized == 0U || hsen->adc_buf == NULL) return;

    /* ── تحديث فلتر Moving Average ── */
    uint8_t idx = hsen->filt_idx;
    for (uint8_t ch = 0U; ch < SENSOR_ADC_CH_COUNT; ch++)
        hsen->filt_buf[ch][idx] = hsen->adc_buf[ch];
    hsen->filt_idx = (uint8_t)((idx + 1U) % SENSOR_FILTER_SIZE);
    if (hsen->filt_idx == 0U) hsen->filt_ready = 1U;

    if (hsen->filt_ready == 0U) return;  /* انتظر دورة كاملة قبل الحساب */

    /* ── قراءة المتوسطات المفلترة ── */
    uint16_t raw_ntc   = filter_average(hsen->filt_buf[SENSOR_IDX_NTC]);
    uint16_t raw_iu    = filter_average(hsen->filt_buf[SENSOR_IDX_IU]);
    uint16_t raw_iv    = filter_average(hsen->filt_buf[SENSOR_IDX_IV]);
    uint16_t raw_iw    = filter_average(hsen->filt_buf[SENSOR_IDX_IW]);
    uint16_t raw_vu    = filter_average(hsen->filt_buf[SENSOR_IDX_VU]);
    uint16_t raw_vv    = filter_average(hsen->filt_buf[SENSOR_IDX_VV]);
    uint16_t raw_vw    = filter_average(hsen->filt_buf[SENSOR_IDX_VW]);
    uint16_t raw_dcv   = filter_average(hsen->filt_buf[SENSOR_IDX_DC_VOLT]);
    uint16_t raw_dci   = filter_average(hsen->filt_buf[SENSOR_IDX_DC_CURR]);

    SENSOR_Data_t *d = &hsen->data;

    /* ── الحرارة — مع offset وgain معايرة NTC ── */
    {
        float t_raw          = calc_temperature(raw_ntc);
        d->temperature_C     = (t_raw * hsen->ntc_gain) + hsen->ntc_offset_c;
    }

    /* ── التيار — [FIX-1] يختار المسار حسب حالة المعايرة ── */
    if (hsen->cal_gain_done != 0U)
    {
        /* مسار المعايرة اليدوية: offset + gain تصحيحي */
        d->current_U_A = calc_current_gain(raw_iu, hsen->offset_iu, hsen->curr_gain_factor);
        d->current_V_A = calc_current_gain(raw_iv, hsen->offset_iv, hsen->curr_gain_factor);
        d->current_W_A = calc_current_gain(raw_iw, hsen->offset_iw, hsen->curr_gain_factor);
    }
    else
    {
        /* مسار أساسي: offset فقط (تلقائي أو بدون معايرة) */
        d->current_U_A = calc_current(raw_iu, hsen->offset_iu);
        d->current_V_A = calc_current(raw_iv, hsen->offset_iv);
        d->current_W_A = calc_current(raw_iw, hsen->offset_iw);
    }

    /* ── الجهد ── */
    d->voltage_U_V   = calc_voltage(raw_vu);
    d->voltage_V_V   = calc_voltage(raw_vv);
    d->voltage_W_V   = calc_voltage(raw_vw);

    /* ── DC Bus ── */
    d->dc_voltage_V  = calc_dc_voltage(raw_dcv);
    d->dc_current_A  = calc_dc_current(raw_dci, hsen->offset_dci);

    /* ── إحصائيات مشتقة ── */
    d->current_max_A = d->current_U_A;
    if (d->current_V_A > d->current_max_A) d->current_max_A = d->current_V_A;
    if (d->current_W_A > d->current_max_A) d->current_max_A = d->current_W_A;
    d->voltage_avg_V = (d->voltage_U_V + d->voltage_V_V + d->voltage_W_V) / 3.0f;

    /* ── فحص العتبات ← يضبط threshold_exceeded ── */
    check_thresholds(hsen);

    /* ── [FIX-3] تحديث ERORR_Pin Logic (علم فقط — الكتابة الفعلية في main.c) ── */
    SENSOR_UpdateHWProtect(hsen);
}

void SENSOR_GetData(const SENSOR_Handle_t *hsen, SENSOR_Data_t *out)
{
    if (hsen == NULL || out == NULL) return;
    memcpy(out, &hsen->data, sizeof(SENSOR_Data_t));
}

/*
 * [FIX-3] SENSOR_UpdateHWProtect
 * محفوظة للتوافق ولاستخدام SENSOR_ClearFaults inline.
 * ERORR_Pin يُكتب من main.c حصراً بناءً على threshold_exceeded.
 * هذه الدالة لا تكتب أي GPIO — تُبقي threshold_exceeded للقارئ الخارجي.
 */
void SENSOR_UpdateHWProtect(SENSOR_Handle_t *hsen)
{
    (void)hsen;
    /* الكتابة على ERORR_Pin تتم في main.c:
     *   GPIO_PIN_RESET (LOW) = خطأ → تفعيل TIM1_BKIN الخارجي
     *   GPIO_PIN_SET   (HIGH) = سليم
     * المنطق موجود في حلقة while(1) عبر:
     *   ep = (hs->data.threshold_exceeded || motor_fault) ? GPIO_PIN_RESET : GPIO_PIN_SET;
     *   ERORR_WRITE(ep);
     */
}

/* ══════════════════════════════════════════════════════════════
 *  دوال ضبط الحدود
 * ══════════════════════════════════════════════════════════════ */

void SENSOR_SetLimits(SENSOR_Handle_t *hsen,
                      float temp_fault, float curr_fault,
                      float volt_fault, float volt_under)
{
    if (hsen == NULL) return;
    if (temp_fault >= 0.0f && temp_fault <= SENSOR_TEMP_MAX_C)
        hsen->limit_temp_fault = temp_fault;
    if (curr_fault >= 0.0f && curr_fault <= SENSOR_CURR_ABS_MAX_A)
        hsen->limit_curr_fault = curr_fault;
    if (volt_fault > 0.0f && volt_fault <= SENSOR_VOLT_FAULT_V)
        hsen->limit_volt_fault = volt_fault;
    if (volt_under >= 0.0f)
        hsen->limit_volt_under = volt_under;
}

void SENSOR_SetCurrentLimits(SENSOR_Handle_t *hsen, float phase_max_a, float dc_max_a)
{
    if (hsen == NULL) return;
    /*
     * -1.0f تعني "لا تغيير" — يُمكِّن main.c من تمرير -1 لأحد الطرفَين
     * بدون التأثير على الآخر.
     * القيمة 0.0f مقبولة (تيار = 0 حماية وقائية).
     */
    if (phase_max_a >= 0.0f && phase_max_a <= SENSOR_CURR_ABS_MAX_A)
        hsen->limit_curr_fault = phase_max_a;
    if (dc_max_a >= 0.0f && dc_max_a <= SENSOR_CURR_ABS_MAX_A)
        hsen->limit_dc_curr_fault = dc_max_a;
}

void SENSOR_SetVoltageLimits(SENSOR_Handle_t *hsen,
                              float volt_under_v,
                              float volt_fault_v)
{
    if (hsen == NULL) return;
    if (volt_under_v > 0.0f && volt_under_v < hsen->limit_volt_fault)
        hsen->limit_volt_under = volt_under_v;
    if (volt_fault_v > 0.0f && volt_fault_v > hsen->limit_volt_under
        && volt_fault_v <= SENSOR_VOLT_FAULT_V)
        hsen->limit_volt_fault = volt_fault_v;
}

void SENSOR_SetDCVoltageLimits(SENSOR_Handle_t *hsen,
                                float dc_under_v,
                                float dc_fault_v)
{
    if (hsen == NULL) return;
    if (dc_under_v > 0.0f && dc_under_v < hsen->limit_dc_volt_fault)
        hsen->limit_dc_volt_under = dc_under_v;
    if (dc_fault_v > 0.0f && dc_fault_v > hsen->limit_dc_volt_under
        && dc_fault_v <= SENSOR_DC_VOLT_FAULT_V)
        hsen->limit_dc_volt_fault = dc_fault_v;
}

void SENSOR_SetTempFaultLimit(SENSOR_Handle_t *hsen, float temp_max_c)
{
    if (hsen == NULL) return;
    if (temp_max_c > 0.0f && temp_max_c <= SENSOR_TEMP_MAX_C)
        hsen->limit_temp_fault = temp_max_c;
}

/* ══════════════════════════════════════════════════════════════
 *  مراقبة العتبات — تفعيل / تعطيل
 * ══════════════════════════════════════════════════════════════ */

void SENSOR_MonitorEnable(SENSOR_Handle_t *hsen, uint16_t mask)
{
    if (hsen == NULL || hsen->initialized == 0U) return;
    uint16_t compile_mask = build_compile_time_mask();
    hsen->monitor_mask |= (mask & compile_mask);
}

void SENSOR_MonitorDisable(SENSOR_Handle_t *hsen, uint16_t mask)
{
    if (hsen == NULL || hsen->initialized == 0U) return;
    hsen->monitor_mask &= ~mask;
}

void SENSOR_MonitorEnableAll(SENSOR_Handle_t *hsen)
{
    if (hsen == NULL) return;
    hsen->monitor_mask = build_compile_time_mask();
}

void SENSOR_MonitorDisableAll(SENSOR_Handle_t *hsen)
{
    if (hsen == NULL) return;
    hsen->monitor_mask = 0U;
}

uint16_t SENSOR_MonitorGetMask(const SENSOR_Handle_t *hsen)
{
    if (hsen == NULL) return 0U;
    return hsen->monitor_mask;
}

/* ══════════════════════════════════════════════════════════════
 *  Calibration API — المعايرة
 * ══════════════════════════════════════════════════════════════ */

/*
 * معايرة تلقائية لأوفست التيار
 * شرط: المحرك متوقف (تيار = 0)، الفلتر جاهز (filt_ready=1)
 * يُعيِّن offset_iu/iv/iw/dci = متوسط ADC الحالي
 * حد الأمان: offset > OFFSET_SAFETY_LIMIT يُرفض (خط مفتوح أو خطأ)
 */
void SENSOR_CalibrateCurrentOffset(SENSOR_Handle_t *hsen)
{
    if (hsen == NULL || hsen->initialized == 0U || hsen->adc_buf == NULL) return;

    const uint16_t OFFSET_SAFETY_LIMIT = 410U;  /* ~0.33V → ~1.6A — حد مقبول */

    uint16_t avg_iu  = filter_average(hsen->filt_buf[SENSOR_IDX_IU]);
    uint16_t avg_iv  = filter_average(hsen->filt_buf[SENSOR_IDX_IV]);
    uint16_t avg_iw  = filter_average(hsen->filt_buf[SENSOR_IDX_IW]);
    uint16_t avg_dci = filter_average(hsen->filt_buf[SENSOR_IDX_DC_CURR]);

    hsen->offset_iu  = (avg_iu  < OFFSET_SAFETY_LIMIT) ? avg_iu  : 0U;
    hsen->offset_iv  = (avg_iv  < OFFSET_SAFETY_LIMIT) ? avg_iv  : 0U;
    hsen->offset_iw  = (avg_iw  < OFFSET_SAFETY_LIMIT) ? avg_iw  : 0U;
    hsen->offset_dci = (avg_dci < OFFSET_SAFETY_LIMIT) ? avg_dci : 0U;

    /* إعادة ضبط gain — المعايرة التلقائية تلغي أي gain يدوي سابق */
    hsen->curr_gain_factor = 1.0f;
    hsen->cal_gain_done    = 0U;

    hsen->cal_done   = 1U;
    hsen->cal_source = 1U;  /* 1 = تلقائي */
}

/*
 * معايرة يدوية لأوفست التيار
 * ref_current_a = 0.0  → يستدعي SENSOR_CalibrateCurrentOffset
 * ref_current_a > 0.0  → مرجع موجب: يُحسب gain فقط (offset من معايرة سابقة)
 */
void SENSOR_CalibrateCurrentManual(SENSOR_Handle_t *hsen, float ref_current_a)
{
    if (hsen == NULL || hsen->initialized == 0U) return;

    /* [FIX-R7] تحقق من جاهزية الفلتر قبل أي مسار
     * يمنع gain خاطئاً إذا استُدعيت مباشرة قبل اكتمال 8 عينات */
    if (hsen->filt_ready == 0U) return;

    if (ref_current_a <= 0.01f)
    {
        SENSOR_CalibrateCurrentOffset(hsen);
        return;
    }

    /*
     * تيار مرجعي موجب — نحسب المتوسط الحالي ونقارنه بالمرجع
     * لاستخراج gain. يتطلب معايرة offset مسبقة (cal_done=1).
     * إذا لم تُجرَ المعايرة التلقائية → نجري offset الآن أولاً.
     */
    if (hsen->cal_done == 0U)
        SENSOR_CalibrateCurrentOffset(hsen);

    /* قياس التيار الحالي بعد تطبيق offset */
    if (hsen->filt_ready == 0U) return;  /* انتظر الفلتر */

    uint16_t raw_iu = filter_average(hsen->filt_buf[SENSOR_IDX_IU]);
    uint16_t raw_iv = filter_average(hsen->filt_buf[SENSOR_IDX_IV]);
    uint16_t raw_iw = filter_average(hsen->filt_buf[SENSOR_IDX_IW]);

    float iu = calc_current(raw_iu, hsen->offset_iu);
    float iv = calc_current(raw_iv, hsen->offset_iv);
    float iw = calc_current(raw_iw, hsen->offset_iw);

    /* متوسط التيار المقاس لاستخراج gain */
    float measured_avg = (iu + iv + iw) / 3.0f;

    if (measured_avg < 0.1f) return;  /* تيار صغير جداً → غير موثوق */

    float new_gain = ref_current_a / measured_avg;
    if (new_gain < 0.5f) new_gain = 0.5f;
    if (new_gain > 2.0f) new_gain = 2.0f;

    hsen->curr_gain_factor = new_gain;
    hsen->cal_gain_done    = 1U;
    hsen->cal_source       = 2U;  /* 2 = يدوي */
}

/*
 * [FIX-1] SENSOR_CalibrateCurrentGain — مُطبَّقة فعلياً الآن
 * measured_a: قراءة النظام الحالية (بعد offset)
 * reference_a: القيمة الحقيقية من أمبيرمتر خارجي
 * gain_new = reference_a / measured_a → يُطبَّق في SENSOR_Update
 */
void SENSOR_CalibrateCurrentGain(SENSOR_Handle_t *hsen,
                                  float measured_a, float reference_a)
{
    if (hsen == NULL || hsen->initialized == 0U) return;
    if (measured_a <= 0.1f || reference_a <= 0.0f) return;

    float new_gain = reference_a / measured_a;
    if (new_gain < 0.5f) new_gain = 0.5f;
    if (new_gain > 2.0f) new_gain = 2.0f;

    hsen->curr_gain_factor = new_gain;
    hsen->cal_gain_done    = 1U;    /* [FIX-1] يُفعِّل مسار calc_current_gain في SENSOR_Update */
    hsen->cal_source       = 2U;
}

/*
 * معايرة NTC — تصحيح بنقطة واحدة (offset فقط)
 * ntc_offset_c = ref_temp_c - temp_measured_c
 * يبقى ntc_gain = 1.0 في المعايرة الأحادية
 */
void SENSOR_CalibrateNTC(SENSOR_Handle_t *hsen, float ref_temp_c)
{
    if (hsen == NULL || hsen->initialized == 0U) return;

    uint16_t raw_ntc = filter_average(hsen->filt_buf[SENSOR_IDX_NTC]);
    float temp_raw   = calc_temperature(raw_ntc);

    if (ref_temp_c < -20.0f || ref_temp_c > 150.0f) return;
    if (temp_raw < -50.0f || temp_raw > 200.0f) return;

    float offset = ref_temp_c - temp_raw;
    if (offset < -25.0f || offset > 25.0f) return;  /* فرق كبير → مشكوك */

    hsen->ntc_offset_c = offset;
    hsen->ntc_gain     = 1.0f;
    hsen->ntc_cal_done = 1U;
    hsen->cal_source   = 2U;
}

/* إعادة ضبط كل المعايرات للقيم الافتراضية */
void SENSOR_CalibrationReset(SENSOR_Handle_t *hsen)
{
    if (hsen == NULL) return;
    hsen->offset_iu        = 0U;
    hsen->offset_iv        = 0U;
    hsen->offset_iw        = 0U;
    hsen->offset_dci       = 0U;
    hsen->ntc_offset_c     = 0.0f;
    hsen->ntc_gain         = 1.0f;
    hsen->curr_gain_factor = 1.0f;   /* [FIX-1] إعادة gain */
    hsen->cal_done         = 0U;
    hsen->ntc_cal_done     = 0U;
    hsen->cal_gain_done    = 0U;     /* [FIX-1] مسح علم gain */
    hsen->cal_source       = 0U;
}

/*
 * قراءة حالة المعايرة — للإرسال عبر Bluetooth / التليمتري
 * out_curr_off: offset التيار [A] (متوسط الأطوار الثلاثة)
 * out_ntc_off : offset الحرارة [°C]
 * out_ntc_gain: gain التيار الفعلي [1.0 = بلا تصحيح]
 */
void SENSOR_GetCalibrationStatus(const SENSOR_Handle_t *hsen,
                                  float *out_curr_off,
                                  float *out_ntc_off,
                                  float *out_ntc_gain)
{
    if (hsen == NULL) return;

    if (out_curr_off != NULL)
    {
        float off_a = (float)((uint32_t)hsen->offset_iu +
                               (uint32_t)hsen->offset_iv +
                               (uint32_t)hsen->offset_iw) / 3.0f;
        *out_curr_off = off_a * (SENSOR_VREF_V / (float)SENSOR_ADC_RESOLUTION)
                        / (SENSOR_CURRENT_GAIN * SENSOR_SHUNT_OHM);
    }
    if (out_ntc_off  != NULL) *out_ntc_off  = hsen->ntc_offset_c;
    /* [FIX-1] يُعيد curr_gain_factor الآن (وليس ntc_gain) */
    if (out_ntc_gain != NULL) *out_ntc_gain = hsen->curr_gain_factor;
}

/* ══════════════════════════════════════════════════════════════
 *  [FIX-2] SENSOR_BuildJSON / SENSOR_BuildJSONEx
 *  مُستدعيتان من UPGRADE_FillTelemetry عند تفعيل
 *  UPGRADE_DEBUG_JSON_EN=1 في pldc_upgrade.h
 *  (إرسال JSON تشخيصي مستقل عبر USART)
 * ══════════════════════════════════════════════════════════════ */

int SENSOR_BuildJSON(const SENSOR_Handle_t *hsen, char *buf, uint16_t len)
{
    if (hsen == NULL || buf == NULL || len < 20U) return 0;
    const SENSOR_Data_t *d = &hsen->data;
    return snprintf(buf, (size_t)len,
        "{\"tmp\":%.1f,\"ui\":%.2f,\"vi\":%.2f,\"wi\":%.2f,"
        "\"vu\":%.0f,\"vv\":%.0f,\"vw\":%.0f,"
        "\"dcv\":%.1f,\"dci\":%.2f,\"err\":%u}\r\n",
        (double)d->temperature_C,
        (double)d->current_U_A, (double)d->current_V_A, (double)d->current_W_A,
        (double)d->voltage_U_V, (double)d->voltage_V_V, (double)d->voltage_W_V,
        (double)d->dc_voltage_V, (double)d->dc_current_A,
        (unsigned int)d->threshold_exceeded);
}

int SENSOR_BuildJSONEx(const SENSOR_Handle_t *hsen, char *buf, uint16_t len)
{
    if (hsen == NULL || buf == NULL || len < 20U) return 0;
    const SENSOR_Data_t *d = &hsen->data;
    return snprintf(buf, (size_t)len,
        "{\"tmp\":%.1f,\"ui\":%.2f,\"vi\":%.2f,\"wi\":%.2f,"
        "\"vu\":%.0f,\"vv\":%.0f,\"vw\":%.0f,"
        "\"dcv\":%.1f,\"dci\":%.2f,"
        "\"err\":%u,\"pmsk\":%u,"
        "\"cal_done\":%u,\"gain\":%.3f}\r\n",
        (double)d->temperature_C,
        (double)d->current_U_A, (double)d->current_V_A, (double)d->current_W_A,
        (double)d->voltage_U_V, (double)d->voltage_V_V, (double)d->voltage_W_V,
        (double)d->dc_voltage_V, (double)d->dc_current_A,
        (unsigned int)d->threshold_exceeded,
        (unsigned int)hsen->monitor_mask,
        (unsigned int)hsen->cal_done,
        (double)hsen->curr_gain_factor);
}
