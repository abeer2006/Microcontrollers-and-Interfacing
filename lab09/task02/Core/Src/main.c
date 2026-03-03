/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_uart.h"
#include <stdint.h>
#include <sys/types.h>
#include "stdarg.h"
#include "stdio.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include "string.h"
#include "math.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
u_int8_t dev_addr = 0x33;
uint16_t mem_addr = 0x0f;
u_int8_t output;

void myPrintf(const char *fmt, ...);
void Init_LSM();
void Offset_LSM();
void Read_LSM();
void Print_LSM();
void gyro_init ();
void gyro_set_ctrl_reg4 ();
void spi_write(uint8_t reg, uint8_t value);
uint8_t spi_read(uint8_t reg);
void gyro_calibrate();
void Read_Gyro();

#define CTRL_REG1_A 0x20
#define CTRL_REG1_A_VAL 0x67
#define CTRL_REG4_A 0x23
#define CTRL_REG4_A_VAL 0x00
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define RAD_TO_DEG 57.2958
#define CTRL_REG1 0x20
#define CTRL_REG1_VAL 0b10001111 // Power on , enable X, Y, Z axes
#define CTRL_REG4 0x23
#define CTRL_REG4_VAL 0b00000000 // FS [1:0] = 00 => +/- 245 dps
#define CS_LOW()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)


typedef struct
{
  // Raw accelerometer values (16-bit signed)
  int16_t acc_raw_x;
  int16_t acc_raw_y;
  int16_t acc_raw_z;
  
  // Scaled accelerometer values (g units)
  float acc_x;
  float acc_y;
  float acc_z;
  
  // Angle estimation (degrees)
  float angle_x;
  float angle_y;
  float angle_z;
  
  // Offset values (calculated during calibration)
  int16_t acc_offset_x;
  int16_t acc_offset_y;
  int16_t acc_offset_z;
  

  // Gyroscope Values
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;

} LSM_Data_t;

LSM_Data_t lsm;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  
  Init_LSM();
  gyro_init();
  lsm.acc_offset_x = 0;
  lsm.acc_offset_y = 0;
  lsm.acc_offset_z = 0;
  lsm.gyro_offset_x = 0;
  lsm.gyro_offset_y = 0;
  lsm.gyro_offset_z = 0;
  Offset_LSM();
  gyro_set_ctrl_reg4();
  gyro_calibrate();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Read_LSM();
    Read_Gyro();
    Print_LSM();
    HAL_Delay(100) ;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void myPrintf(const char *fmt, ...)
{
  char buffer[128]; 
  va_list argument;
  va_start(argument, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, argument);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}

void Init_LSM(){
uint8_t data1 = 0x67;
uint8_t data2 = 0x00;
HAL_I2C_Mem_Write(&hi2c1, dev_addr, CTRL_REG1_A, 1, &data1, 1, 100);
HAL_I2C_Mem_Write(&hi2c1, dev_addr, CTRL_REG4_A, 1, &data2, 1, 100);
}

void Read_LSM(){
  uint8_t x_low, x_high, y_low, y_high, z_low, z_high;
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_X_L_A, 1, &x_low, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_X_H_A, 1, &x_high, 1, 100);
  lsm.acc_raw_x = (int16_t)(x_high << 8) | x_low;
  
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Y_L_A, 1, &y_low, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Y_H_A, 1, &y_high, 1, 100);
  lsm.acc_raw_y = (int16_t)(y_high << 8) | y_low;
  
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Z_L_A, 1, &z_low, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Z_H_A, 1, &z_high, 1, 100);
  lsm.acc_raw_z = (int16_t)(z_high << 8) | z_low;
  
  //dividing by 1000 to convert from mg to g
  lsm.acc_x = (lsm.acc_raw_x - lsm.acc_offset_x) / 1000.0f;
  lsm.acc_y = (lsm.acc_raw_y - lsm.acc_offset_y) / 1000.0f;
  lsm.acc_z = (lsm.acc_raw_z - lsm.acc_offset_z) / 1000.0f;

  //using atan2 to calculate angles in degrees
  lsm.angle_x  = atan2(lsm.acc_y, lsm.acc_z) * RAD_TO_DEG;
  lsm.angle_y = atan2(-lsm.acc_x, sqrt(lsm.acc_y*lsm.acc_y + lsm.acc_z*lsm.acc_z)) * RAD_TO_DEG;


}

void Print_LSM(){
  myPrintf(" Accelerometer X: %0.2f, Y: %0.2f, Z: %0.2f Angle X: %0.2f Angle Y: %0.2f \r\n Gyroscope X: %d Y: %d Z: %d\r\n",
     lsm.acc_x, lsm.acc_y, lsm.acc_z, lsm.angle_x, lsm.angle_y, lsm.gyro_x, lsm.gyro_y, lsm.gyro_z);
}

void Offset_LSM(){
  uint8_t x_low, x_high, y_low, y_high, z_low, z_high;

  for(int i = 0; i < 20; i++){
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_X_L_A, 1, &x_low, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_X_H_A, 1, &x_high, 1, 100);
    lsm.acc_offset_x += (int16_t)(x_high << 8) | x_low;
  
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Y_L_A, 1, &y_low, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Y_H_A, 1, &y_high, 1, 100);
    lsm.acc_offset_y += (int16_t)(y_high << 8) | y_low;
    
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Z_L_A, 1, &z_low, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Z_H_A, 1, &z_high, 1, 100);
    lsm.acc_offset_z += (int16_t)(z_high << 8) | z_low;

    //myPrintf("offset -> x: %d y: %d z: %d\r\n", lsm.acc_offset_x, lsm.acc_offset_y, lsm.acc_offset_z);
    HAL_Delay(100);
  }
  lsm.acc_offset_x /= 20; 
  lsm.acc_offset_y /= 20;
  lsm.acc_offset_z /= 20;
}

void gyro_init ()
{
 uint8_t tx [2] = { CTRL_REG1 , CTRL_REG1_VAL };
 // Pull CS low to begin communication
 HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_RESET );

 // Send control register configuration
 HAL_SPI_Transmit (&hspi1 , tx , 2, HAL_MAX_DELAY );

 // Pull CS high to end communication
 HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_SET );
}

void gyro_set_ctrl_reg4 ()
{
  uint8_t tx [2] = { CTRL_REG4 , CTRL_REG4_VAL };
  HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_RESET );
  HAL_SPI_Transmit (& hspi1 , tx , 2, 100 );
  HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_SET );
}

void gyro_calibrate()
{
    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    const int samples = 20;

    for(int i = 0; i < samples; i++)
    {
        int16_t x = (int16_t)(spi_read(OUT_X_H_A) << 8 | spi_read(OUT_X_L_A));
        int16_t y = (int16_t)(spi_read(OUT_Y_H_A) << 8 | spi_read(OUT_Y_L_A));
        int16_t z = (int16_t)(spi_read(OUT_Z_H_A) << 8 | spi_read(OUT_Z_L_A));

        x_sum += x;
        y_sum += y;
        z_sum += z;

        HAL_Delay(100);
    }

    lsm.gyro_offset_x = x_sum / samples;
    lsm.gyro_offset_y = y_sum / samples;
    lsm.gyro_offset_z = z_sum / samples;
}


uint8_t spi_read(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = reg | 0x80;  // Read command (MSB = 1)
    
    tx[1] = 0x00;

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_HIGH();

    return rx[1];
}

void spi_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg & 0x7F, value};  // Write command (MSB = 0)
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

// SPI read multiple bytes from L3GD20
// void spi_read_bytes(uint8_t reg, uint8_t *data, uint8_t len)
// {
//     // For L3GD20:
//     // Bit 7 = 1 (read), Bit 6 = 1 (auto-increment if reading multiple registers)
//     uint8_t addr = reg | 0x80;
//     if(len > 1) addr |= 0x40; // Enable auto-increment for multi-byte read

//     CS_LOW();
//     HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
//     HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
//     CS_HIGH();
// }

// // SPI write a single byte
// void spi_write(uint8_t reg, uint8_t value)
// {
//     uint8_t data[2] = { reg & 0x7F, value }; // MSB = 0 for write
//     CS_LOW();
//     HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
//     CS_HIGH();
// }
// void gyro_calibrate()
// {
//     int32_t x_sum = 0, y_sum = 0, z_sum = 0;
//     const int samples = 50; // Take more samples for better accuracy
//     uint8_t buffer[6];

//     for(int i = 0; i < samples; i++)
//     {
//         spi_read_bytes(OUT_X_L_A, buffer, 6);

//         int16_t x = (int16_t)(buffer[0] | (buffer[1] << 8));
//         int16_t y = (int16_t)(buffer[2] | (buffer[3] << 8));
//         int16_t z = (int16_t)(buffer[4] | (buffer[5] << 8));

//         x_sum += x;
//         y_sum += y;
//         z_sum += z;

//         HAL_Delay(20);
//     }

//     lsm.gyro_offset_x = x_sum / samples;
//     lsm.gyro_offset_y = y_sum / samples;
//     lsm.gyro_offset_z = z_sum / samples;
// }


void Read_Gyro()
{
    uint8_t xl, xh, yl, yh, zl, zh;

    // X-axis
    xl = spi_read(OUT_X_L_A);
    xh = spi_read(OUT_X_H_A);
    lsm.gyro_x = (int16_t)((xh << 8) | xl) - lsm.gyro_offset_x;

    // Y-axis
    yl = spi_read(OUT_Y_L_A);
    yh = spi_read(OUT_Y_H_A);
    lsm.gyro_y = (int16_t)((yh << 8) | yl) - lsm.gyro_offset_y;

    // Z-axis
    zl = spi_read(OUT_Z_L_A);
    zh = spi_read(OUT_Z_H_A);
    lsm.gyro_z = (int16_t)((zh << 8) | zl) - lsm.gyro_offset_z;
}
// void Read_Gyro()
// {
//     uint8_t buffer[6];
    
//     // Read 6 consecutive bytes: OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H
//     spi_read_bytes(OUT_X_L_A, buffer, 6);

//     // Combine low and high bytes (low first!)
//     lsm.gyro_x = (int16_t)(buffer[0] | (buffer[1] << 8)) - lsm.gyro_offset_x;
//     lsm.gyro_y = (int16_t)(buffer[2] | (buffer[3] << 8)) - lsm.gyro_offset_y;
//     lsm.gyro_z = (int16_t)(buffer[4] | (buffer[5] << 8)) - lsm.gyro_offset_z;
// }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
