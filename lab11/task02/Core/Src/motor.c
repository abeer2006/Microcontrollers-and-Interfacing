# include "motor.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_uart.h"
#include <stdint.h>
#include <sys/types.h>
#include "stdarg.h"
#include "stdio.h"
#include "stm32f3xx_hal.h"
#include "string.h"
#include "math.h"
#include <stdint.h>

void RightMotor_Forward(uint16_t speed)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // D8 HIGH
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // D12 LOW

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed); // PWM speed
}

void RightMotor_Backward(uint16_t speed)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET); // D8 LOW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // D12 HIGH

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}

void RightMotor_Stop()
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}

void LeftMotor_Forward(uint16_t speed)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // D7 HIGH
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); // D6 LOW

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed); // PWM speed
}

void LeftMotor_Backward(uint16_t speed)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // D7 LOW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); // D6 HIGH

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

void LeftMotor_Stop(void)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
