/**
  ******************************************************************************
  * @file    foc.h
  * @brief   Field-oriented-control math primitives: PI regulator, Clarke /
  *          Park transforms and a space-vector modulator (min/max common-mode
  *          injection). Self-contained, float, FPU-friendly, no HAL.
  ******************************************************************************
  */
#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#ifndef FOC_TWO_PI
#define FOC_TWO_PI   6.28318530718f
#endif
#define FOC_SQRT3    1.73205080757f
#define FOC_SQRT3_2  0.86602540378f

typedef struct
{
    float kp, ki;        /* gains                                  */
    float integ;         /* integrator state                       */
    float out_min;       /* output clamp                           */
    float out_max;
} PI_t;

void  PI_Init (PI_t *p, float kp, float ki, float out_min, float out_max);
void  PI_Reset(PI_t *p);
float PI_Step (PI_t *p, float err, float dt);   /* anti-windup clamp included */

/* ia + ib assumed; ic = -(ia+ib). */
void Clarke  (float ia, float ib, float *alpha, float *beta);
void Park    (float alpha, float beta, float s, float c, float *d, float *q);
void InvPark (float d, float q, float s, float c, float *alpha, float *beta);

/* alpha/beta voltages (volts) + bus voltage -> three timer compare values.
 * Returns the peak duty (0..1) actually applied for telemetry. */
float SVPWM(float valpha, float vbeta, float vdc, uint32_t arr,
            uint32_t *ca, uint32_t *cb, uint32_t *cc);

#endif /* FOC_H */
