/**
  ******************************************************************************
  * @file    measure.c
  * @brief   Implementation with IDC working correctly.
  *
  *  KEY POINTS:
  *    - IDC uses lab calibration (145.6 mV/A)
  ******************************************************************************
  */
#include "measure.h"
#include <math.h>
#include <string.h>

/* ===========================================================================
 *  PRIVATE HELPERS
 * =========================================================================== */

static inline float adc_to_mv(uint16_t raw)
{
    return (float)raw * MEAS_ADC_TO_MV;
}

static inline float iir_filter(float new_val, float prev, float alpha)
{
    return prev + alpha * (new_val - prev);
}

static float moving_average_filter(Meas_MAFilter_t *f, float input)
{
    if (f->count < MEAS_MOVING_AVG_SIZE) {
        f->buffer[f->index] = input;
        f->sum += input;
        f->count++;
        f->value = f->sum / (float)f->count;
    } else {
        f->sum -= f->buffer[f->index];
        f->buffer[f->index] = input;
        f->sum += input;
        f->value = f->sum / (float)MEAS_MOVING_AVG_SIZE;
    }
    f->index = (f->index + 1) % MEAS_MOVING_AVG_SIZE;
    return f->value;
}

static inline float apply_hysteresis(float new_val, float last_val, float hysteresis)
{
    if (fabsf(new_val - last_val) < hysteresis) return last_val;
    return new_val;
}

static inline float adc_to_voltage(uint16_t raw, float rtop, float rbot)
{
    float v_adc = (float)raw * (MEAS_VREF / MEAS_ADC_FULL);
    return v_adc * ((rtop + rbot) / rbot);
}

static void reset_ma_filter(Meas_MAFilter_t *f)
{
    f->index = 0;
    f->count = 0;
    f->sum = 0.0f;
    f->value = 0.0f;
    memset(f->buffer, 0, sizeof(f->buffer));
}

/* ===========================================================================
 *  CONVERSION FUNCTIONS
 * =========================================================================== */

/**
 * @brief Convert ADC difference to IDC current (WORKING - lab calibrated)
 * 
 * Formula: I = (V_mV - 2.5) / 145.6
 * Example: 152mV → (152-2.5)/145.6 = 1.027A
 */
float Meas_IDC_RawToCurrent(int32_t adc_diff)
{
    /* Step 1: ADC difference to millivolts */
    float voltage_mv = (float)adc_diff * MEAS_ADC_TO_MV;
    
    /* Step 2: Apply zero offset correction */
    voltage_mv -= IDC_ZERO_OFFSET_MV;
    
    /* Step 3: Convert to current using slope */
    float current = voltage_mv / IDC_SLOPE_MV_PER_A;
    
    /* Step 4: Apply noise floor */
    if (fabsf(current) < IDC_NOISE_FLOOR_A) {
        current = 0.0f;
    }
    
    return current;
}

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

void Meas_Init(Meas_t *m, volatile uint16_t *dma_buf)
{
    m->dma = dma_buf;
    
    /* Default offset (zero for DC) */
    m->off_idc = 0;
    
    /* Clear calibration */
    m->cal_acc = 0;
    m->cal_count = 0;
    m->calibrated = false;
    
    /* Reset filters */
    m->idc_filt = 0.0f;
    m->idc_ma = 0.0f;
    m->idc_last = 0.0f;
    m->idc = 0.0f;
    
    reset_ma_filter(&m->ma_idc);
    
    m->vdc = m->vu = m->vv = m->vw = 0.0f;
    m->temp_c = 25.0f;
    m->vdc_f = m->idc_f = m->temp_f = 0.0f;
}

void Meas_StartCalib(Meas_t *m)
{
    m->cal_acc = 0;
    m->cal_count = 0;
    m->calibrated = false;
}

bool Meas_CalibTask(Meas_t *m)
{
    if (m->calibrated) return true;
    
    m->cal_acc += m->dma[MEAS_IDX_IDC];
    m->cal_count++;
    
    if (m->cal_count >= 256) {
        m->off_idc = (uint16_t)(m->cal_acc / m->cal_count);
        m->calibrated = true;
        Meas_ResetFilters(m);
        return true;
    }
    return false;
}

void Meas_Update(Meas_t *m)
{
    if (!m->calibrated) {
        m->idc = 0.0f;
        return;
    }
    
    /* Calculate ADC difference */
    int32_t diff_idc = (int32_t)m->dma[MEAS_IDX_IDC] - (int32_t)m->off_idc;
    
    /* ---- IDC: Use lab calibration ---- */
    float idc_raw = Meas_IDC_RawToCurrent(diff_idc);
    m->idc_filt = iir_filter(idc_raw, m->idc_filt, MEAS_IIR_ALPHA);
    m->idc_ma = moving_average_filter(&m->ma_idc, m->idc_filt);
    m->idc = apply_hysteresis(m->idc_ma, m->idc_last, MEAS_HYSTERESIS);
    m->idc_last = m->idc;
    
    /* ---- Voltage readings ---- */
    m->vdc = adc_to_voltage(m->dma[MEAS_IDX_VDC], MEAS_VDC_RTOP, MEAS_VDC_RBOT);
    m->vu  = adc_to_voltage(m->dma[MEAS_IDX_VU],  MEAS_VPH_RTOP, MEAS_VPH_RBOT);
    m->vv  = adc_to_voltage(m->dma[MEAS_IDX_VV],  MEAS_VPH_RTOP, MEAS_VPH_RBOT);
    m->vw  = adc_to_voltage(m->dma[MEAS_IDX_VW],  MEAS_VPH_RTOP, MEAS_VPH_RBOT);
    
    /* ---- Temperature ---- */
    float v_ntc = adc_to_mv(m->dma[MEAS_IDX_NTC]) / 1000.0f;
    if (v_ntc < 0.01f) v_ntc = 0.01f;
    if (v_ntc > MEAS_VREF - 0.01f) v_ntc = MEAS_VREF - 0.01f;
    float rntc = MEAS_NTC_RFIXED * (v_ntc / (MEAS_VREF - v_ntc));
    float t_kelvin = 1.0f / (1.0f / MEAS_NTC_T25_K + 
                     logf(rntc / MEAS_NTC_R25) / MEAS_NTC_BETA);
    m->temp_c = t_kelvin - 273.15f;
    
    /* ---- Display filters ---- */
    m->vdc_f  = iir_filter(m->vdc,  m->vdc_f,  MEAS_DISP_FILT_ALPHA);
    m->idc_f  = iir_filter(m->idc,  m->idc_f,  MEAS_DISP_FILT_ALPHA);
    m->temp_f = iir_filter(m->temp_c, m->temp_f, MEAS_DISP_FILT_ALPHA);
}

void Meas_UpdateFast(Meas_t *m)
{
    if (!m->calibrated) return;
    
    int32_t diff_idc = (int32_t)m->dma[MEAS_IDX_IDC] - (int32_t)m->off_idc;
    
    static float fidc = 0;
    const float alpha_fast = 0.3f;
    
    fidc = fidc + alpha_fast * (Meas_IDC_RawToCurrent(diff_idc) - fidc);
    
    m->idc = fidc;
}

void Meas_UpdateSlow(Meas_t *m)
{
    Meas_Update(m);
}

void Meas_ResetFilters(Meas_t *m)
{
    m->idc_filt = 0.0f;
    m->idc_ma = 0.0f;
    m->idc_last = 0.0f;
    
    reset_ma_filter(&m->ma_idc);
}