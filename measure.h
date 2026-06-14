/**
  ******************************************************************************
  * @file    measure.h
  * @brief   Analog front-end with per-channel calibration.
  *
  *  IDC (DC bus current) is calibrated from lab measurements:
  *    I(A) = (V_mV - 2.5) / 145.6
  *    or simplified: I = (ADC_diff) × 0.005534 - 0.01717
  ******************************************************************************
  */
#ifndef MEASURE_H
#define MEASURE_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * ADC DMA buffer layout
 * ------------------------------------------------------------------------- */
/* The ADC still scans 9 ranks in hardware; buffer slots 2..4 are sampled
 * but no longer read in software, so the IDC slot (8) stays in place. */
#define MEAS_NCH        9u
#define MEAS_IDX_NTC    0u
#define MEAS_IDX_VDC    1u
#define MEAS_IDX_VU     5u
#define MEAS_IDX_VV     6u
#define MEAS_IDX_VW     7u
#define MEAS_IDX_IDC    8u

/* ---- ADC hardware parameters ---------------------------------------------- */
#define MEAS_VREF       3.30f           /* ADC reference voltage (volts)      */
#define MEAS_ADC_FULL   4095.0f         /* 12-bit ADC maximum raw value       */
#define MEAS_VREF_MV    3300.0f         /* ADC reference voltage (millivolts) */
#define MEAS_ADC_TO_MV  0.80566f        /* (VREF_MV / ADC_FULL)               */

/* ===========================================================================
 *  IDC CALIBRATION (from lab measurements - WORKING)
 * ===========================================================================
 *  Formula: I = (V_mV - 2.5) / 145.6
 */
#define IDC_SLOPE_MV_PER_A     145.6f   /* mV per Ampere (from lab)           */
#define IDC_ZERO_OFFSET_MV     2.5f     /* Zero offset in mV                  */
#define IDC_NOISE_FLOOR_A      0.03f    /* Readings below this = 0            */

/* Pre-calculated coefficients for IDC */
#define IDC_RAW_TO_A_COEFF     (MEAS_ADC_TO_MV / IDC_SLOPE_MV_PER_A)  /* 0.005534 */
#define IDC_RAW_TO_A_OFFSET    (IDC_ZERO_OFFSET_MV / IDC_SLOPE_MV_PER_A) /* 0.01717 */

/* ===========================================================================
 *  FILTERING CONFIGURATION
 * =========================================================================== */
#define MEAS_IIR_ALPHA          0.20f   /* IIR filter coefficient (0.1-0.3)  */
#define MEAS_MOVING_AVG_SIZE    8       /* Moving average window size        */
#define MEAS_HYSTERESIS         0.02f   /* Output hysteresis (amps)          */

/* ===========================================================================
 *  VOLTAGE DIVIDERS
 * =========================================================================== */
#define MEAS_VDC_RTOP   1360000.0f      /* R53 + R54 (680k + 680k)           */
#define MEAS_VDC_RBOT     10000.0f      /* R55                               */
#define MEAS_VPH_RTOP   1360000.0f      /* Phase voltage divider top         */
#define MEAS_VPH_RBOT     10000.0f      /* Phase voltage divider bottom      */

/* ===========================================================================
 *  NTC THERMISTOR
 * =========================================================================== */
#define MEAS_NTC_RFIXED 47000.0f
#define MEAS_NTC_R25    50000.0f
#define MEAS_NTC_BETA   3950.0f
#define MEAS_NTC_T25_K  298.15f

/* ===========================================================================
 *  DISPLAY FILTERING
 * =========================================================================== */
#define MEAS_DISP_FILT_ALPHA 0.10f

/* ===========================================================================
 *  DATA STRUCTURES
 * =========================================================================== */

/* Moving average filter */
typedef struct {
    float buffer[MEAS_MOVING_AVG_SIZE];
    uint8_t index;
    uint8_t count;
    float sum;
    float value;
} Meas_MAFilter_t;

/* Main measurement structure */
typedef struct
{
    /* DMA buffer pointer */
    volatile uint16_t *dma;
    
    /* Calibration offset (raw ADC counts at zero current) */
    uint16_t off_idc;
    uint32_t cal_acc;
    uint16_t cal_count;
    bool     calibrated;
    
    /* Filter states */
    float idc_filt;
    float idc_ma;
    float idc_last;
    
    /* Moving average buffer */
    Meas_MAFilter_t ma_idc;
    
    /* FINAL OUTPUT (use this) */
    float idc;          /* DC bus current (A) */
    
    /* Voltage and temperature */
    float vdc, vu, vv, vw;
    float temp_c;
    
    /* Display-filtered values */
    float vdc_f, idc_f, temp_f;
    
} Meas_t;

/* ===========================================================================
 *  PUBLIC API
 * =========================================================================== */

void Meas_Init(Meas_t *m, volatile uint16_t *dma_buf);
void Meas_StartCalib(Meas_t *m);
bool Meas_CalibTask(Meas_t *m);
void Meas_Update(Meas_t *m);
void Meas_UpdateFast(Meas_t *m);
void Meas_UpdateSlow(Meas_t *m);
void Meas_ResetFilters(Meas_t *m);

/* Conversion function */
float Meas_IDC_RawToCurrent(int32_t adc_diff);

#endif /* MEASURE_H */