/**
  ******************************************************************************
  * @file    app.c
  * @brief   Application glue (see app.h). Nothing here edits a CubeMX file:
  *          runtime-only configuration (enabling the TIM1 update IRQ, gating
  *          MOE, disabling the spurious cold-start analog-watchdog IRQ) lives
  *          in user code.
  ******************************************************************************
  */
#include "app.h"
#include "main.h"          /* pin macros: FAN_12V_Pin, SOFT_START_Relay_Pin,
                              ERORR_Pin/Port, STOP_Pin, NTC_Pin ...           */
#include "adc.h"
#include "tim.h"
#include "usart.h"

/* CubeMX-owned handles */
extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim10;
extern UART_HandleTypeDef huart1;

/* ---- global module instances --------------------------------------------- */
Meas_t          g_meas;
Motor_Handle_t  g_motor;
BtMotor_t       g_bt;
Motor_Safety_t  g_motor_safety;  /* NEW */

static volatile uint16_t g_adc_dma[MEAS_NCH];   /* circular ADC DMA target    */

/* soft-start relay / fan / LED state */
static uint32_t s_relay_ok_ms;
static bool     s_relay_on;
static uint32_t s_led_ms;

/* SYS heartbeat LED is on PC13 (raw, as configured by gpio.c) */
#define SYS_LED_PORT   GPIOC
#define SYS_LED_PIN    GPIO_PIN_13

/* Error pin callbacks for Motor_Safety */
static void app_error_pin_callback(uint8_t state)
{
    HAL_GPIO_WritePin(ERORR_GPIO_Port, ERORR_Pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void app_message_callback(const char *msg)
{
    if (msg != NULL) {
        uint16_t len = 0;
        while (msg[len] != '\0' && len < 256) len++;
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
        HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 50);
    }
}

/* ------------------------------------------------------------------ */
/*  Init                                                              */
/* ------------------------------------------------------------------ */
void App_Init(void)
{
    /* 1) start the free-running ADC into the DMA buffer */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma, MEAS_NCH);

    /* 2) bring up the measurement module */
    Meas_Init(&g_meas, g_adc_dma);

    /* 3) initialize motor control */
    Motor_Init(&g_motor, &htim1, &g_meas);
    
    /* 3b) NEW: Initialize motor safety test module */
    Motor_Safety_Init(&g_motor_safety, g_adc_dma);
    Motor_Safety_SetErrorPinCallback(&g_motor_safety, app_error_pin_callback);
    Motor_Safety_SetMessageCallback(&g_motor_safety, app_message_callback);

    /* 4) start all six PWM outputs, then force MOE off so idle = safe
     *    (OCxIdleState/OCNxIdleState = RESET -> HIN=LIN=0 -> both IGBTs off) */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_DISABLE(&htim1);

    /* 5) enable the TIM1 update interrupt -> control ISR at the PWM rate
     *    (the NVIC line is already enabled by the CubeMX MspInit) */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    /* 5b) HARDWARE-ONLY protection: the BKIN(PB12) break cuts the outputs in
     *     hardware. Enable its interrupt so firmware mirrors the trip (FAULT
     *     state + HMI log) once the ERROR net (PA12<->PB12) is pulled low. */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);

    /* 6) 1 kHz housekeeping tick */
    HAL_TIM_Base_Start_IT(&htim10);

    /* 7) Bluetooth bridge (also arms UART RX) */
    BtMotor_Init(&g_bt, &g_motor, &huart1);

    s_relay_ok_ms = 0;
    s_relay_on    = false;
    s_led_ms      = HAL_GetTick();
}

/* ------------------------------------------------------------------ */
/*  Super-loop task                                                   */
/* ------------------------------------------------------------------ */
void App_Task(void)
{
    BtMotor_Task(&g_bt);

    uint32_t now = HAL_GetTick();
    
    /* NEW: Run motor safety test machine if active */
    if (Motor_Safety_Task(&g_motor_safety)) {
        /* Test still running — don't transition to normal operation */
        return;
    }

    /* ---- soft-start (inrush bypass) relay: close once the bus is up ---- */
    if (g_meas.vdc_f > (g_motor.uv_volt + 40.0f))
    {
        if (s_relay_ok_ms == 0) s_relay_ok_ms = now;
        if (!s_relay_on && (now - s_relay_ok_ms) > 300u)   /* stable 300 ms */
        {
            HAL_GPIO_WritePin(SOFT_START_Relay_GPIO_Port, SOFT_START_Relay_Pin, GPIO_PIN_SET);
            s_relay_on = true;
        }
    }
    else
    {
        s_relay_ok_ms = 0;
        if (s_relay_on)
        {
            HAL_GPIO_WritePin(SOFT_START_Relay_GPIO_Port, SOFT_START_Relay_Pin, GPIO_PIN_RESET);
            s_relay_on = false;
        }
    }

    /* ---- cooling fan: on while energised or when warm ---- */
    bool fan = (g_motor.state == MOTOR_STATE_RUN) ||
               (g_motor.state == MOTOR_STATE_STOPPING) ||
               (g_meas.temp_c > 45.0f);
    HAL_GPIO_WritePin(FAN_12V_GPIO_Port, FAN_12V_Pin, fan ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* ---- ERORR_Pin management: controlled from motor.c + Motor_Safety ---- */
    uint8_t error_pin_state = GPIO_PIN_SET;  /* default: healthy */
    
    /* Check motor fault state */
    if (g_motor.state == MOTOR_STATE_FAULT || g_motor.error_latched) {
        error_pin_state = GPIO_PIN_RESET;
    }
    
    /* Check safety test error */
    if (Motor_Safety_IsErrorLatched(&g_motor_safety)) {
        error_pin_state = GPIO_PIN_RESET;
    }
    
    HAL_GPIO_WritePin(ERORR_GPIO_Port, ERORR_Pin, error_pin_state);

    /* ---- heartbeat: 1 Hz idle, 4 Hz running ---- */
    uint32_t blink = (g_motor.state == MOTOR_STATE_RUN) ? 125u : 500u;
    if ((now - s_led_ms) >= blink)
    {
        s_led_ms = now;
        HAL_GPIO_TogglePin(SYS_LED_PORT, SYS_LED_PIN);
    }
}

/* ================================================================== */
/*  HAL weak-callback overrides                                       */
/* ================================================================== */

/* TIM1 update == PWM-rate control ISR; TIM10 == 1 kHz housekeeping.
 * Both share TIM1_UP_TIM10_IRQn; HAL dispatches by instance. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
        Motor_ControlISR(&g_motor);
    else if (htim->Instance == TIM10)
        Motor_Tick1ms(&g_motor);
}

/* hardware over-current / fault on the BKIN pin (PB12, active low) */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
        Motor_OnFault(&g_motor, MOTOR_FAULT_OVERCURRENT);
}

/* ADC DMA full-sequence complete: data already sits in g_adc_dma, nothing to do */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { (void)hadc; }

/* analog watchdog (NTC, CH0): only fires on an out-of-window cold board.
 * We compute temperature in software, so silence the IRQ to avoid a storm. */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
        __HAL_ADC_DISABLE_IT(hadc, ADC_IT_AWD);
}

/* emergency-stop button (STOP_Pin, EXTI falling) */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == STOP_Pin)
        Motor_OnFault(&g_motor, MOTOR_FAULT_ESTOP);
}

/* HC-05 byte received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        BtMotor_OnRxByte(&g_bt);
}
