/**
  ******************************************************************************
  * @file    app.h
  * @brief   Application glue layer. Owns the global module instances, starts
  *          the peripherals at runtime (without editing any CubeMX file) and
  *          routes every interrupt to the right module via the HAL weak
  *          callbacks. Call App_Init() once after the MX_*_Init() calls, then
  *          App_Task() continuously from the main super-loop.
  ******************************************************************************
  */
#ifndef APP_H
#define APP_H

#include "measure.h"
#include "motor.h"
#include "bt_motor.h"
#include "Motor_Safety.h"

extern Meas_t          g_meas;
extern Motor_Handle_t  g_motor;
extern BtMotor_t       g_bt;
extern Motor_Safety_t  g_motor_safety;  /* NEW */

void App_Init(void);    /* call after all MX_*_Init() in main()           */
void App_Task(void);    /* call from the while(1) loop (non-blocking)     */

#endif /* APP_H */
