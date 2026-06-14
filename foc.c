/**
  ******************************************************************************
  * @file    foc.c
  * @brief   Implementation of the FOC math primitives (see foc.h).
  ******************************************************************************
  */
#include "foc.h"

void PI_Init(PI_t *p, float kp, float ki, float out_min, float out_max)
{
    p->kp = kp; p->ki = ki;
    p->integ = 0.0f;
    p->out_min = out_min; p->out_max = out_max;
}

void PI_Reset(PI_t *p) { p->integ = 0.0f; }

float PI_Step(PI_t *p, float err, float dt)
{
    p->integ += p->ki * err * dt;

    /* clamp integrator to the output range (anti-windup) */
    if (p->integ > p->out_max) p->integ = p->out_max;
    if (p->integ < p->out_min) p->integ = p->out_min;

    float out = p->kp * err + p->integ;
    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;
    return out;
}

void Clarke(float ia, float ib, float *alpha, float *beta)
{
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * (1.0f / FOC_SQRT3);
}

void Park(float alpha, float beta, float s, float c, float *d, float *q)
{
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

void InvPark(float d, float q, float s, float c, float *alpha, float *beta)
{
    *alpha = d * c - q * s;
    *beta  = d * s + q * c;
}

static inline float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

float SVPWM(float valpha, float vbeta, float vdc, uint32_t arr,
            uint32_t *ca, uint32_t *cb, uint32_t *cc)
{
    if (vdc < 1.0f) vdc = 1.0f;

    /* inverse Clarke -> phase voltages */
    float va = valpha;
    float vb = -0.5f * valpha + FOC_SQRT3_2 * vbeta;
    float vc = -0.5f * valpha - FOC_SQRT3_2 * vbeta;

    /* common-mode (min/max) injection -> extends linear range to vdc/sqrt3 */
    float vmax = va, vmin = va;
    if (vb > vmax) vmax = vb;
    if (vb < vmin) vmin = vb;
    if (vc > vmax) vmax = vc;
    if (vc < vmin) vmin = vc;
    float vcom = 0.5f * (vmax + vmin);
    va -= vcom; vb -= vcom; vc -= vcom;

    /* volts -> duty (0..1), centred at 0.5 */
    float inv = 1.0f / vdc;
    float da = clamp01(0.5f + va * inv);
    float db = clamp01(0.5f + vb * inv);
    float dc = clamp01(0.5f + vc * inv);

    *ca = (uint32_t)(da * (float)arr);
    *cb = (uint32_t)(db * (float)arr);
    *cc = (uint32_t)(dc * (float)arr);

    /* peak duty for telemetry */
    float pk = da; if (db > pk) pk = db; if (dc > pk) pk = dc;
    return pk;
}
