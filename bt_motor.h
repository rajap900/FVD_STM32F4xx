/**
  ******************************************************************************
  * @file    bt_motor.h
  * @brief   Bluetooth (HC-05 / UART1) command + telemetry bridge to the motor
  *          core. Line protocol, newline-terminated, 9600 8N1.
  *
  *  Commands in  : START | STOP | FWD | REV | CLR_FLT
  *                 FREQ=x.y | RAMP=n | TORQUE=n | POLES=n | HZSYS=50|60
  *                 MODE=VF|FOC
  *                 NAMEPLATE=volt,curr,freq,rpm,acc,fmin[,poles]
  *  Telemetry out: one JSON object per line (see bt_send_telemetry)
  ******************************************************************************
  */
#ifndef BT_MOTOR_H
#define BT_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "motor.h"

#define BT_LINE_MAX          96
#define BT_TELEM_PERIOD_MS   400u     /* ~2.5 Hz; a full frame is ~200 B @9600 */

typedef struct
{
    Motor_Handle_t    *motor;
    UART_HandleTypeDef *huart;

    /* command shadow state */
    int   cmd_freq_hz_x10;
    int   fmin_hz;
    int   fmax_hz;
    int   oc_x10;
    bool  reverse;

    /* telemetry pacing */
    uint32_t telem_period_ms;
    uint32_t last_telem_ms;

    /* RX line assembly */
    uint8_t rx_byte;
    char    line[BT_LINE_MAX];
    char    ready_line[BT_LINE_MAX];
    int     line_len;
    bool    line_ready;
} BtMotor_t;

void BtMotor_Init   (BtMotor_t *bt, Motor_Handle_t *motor, UART_HandleTypeDef *huart);
void BtMotor_OnRxByte(BtMotor_t *bt);     /* call from HAL_UART_RxCpltCallback */
void BtMotor_Task    (BtMotor_t *bt);     /* call from the main super-loop     */

#endif /* BT_MOTOR_H */
