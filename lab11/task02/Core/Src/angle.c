#include "angle.h"
#include "stm32f3xx_hal_def.h"
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

void Init_LSM(){
    uint8_t data1 = 0x67;
    uint8_t data2 = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, dev_addr, CTRL_REG1_A, 1, &data1, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, dev_addr, CTRL_REG4_A, 1, &data2, 1, 100);
}

void Read_LSM(LSM_Data_t* lsm){
    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, (OUT_X_L_A | 0x80), 1, raw, 6, 100) != HAL_OK) {
        return;
    }
    lsm->acc_raw_x = (int16_t)((raw[1] << 8) | raw[0]);
    lsm->acc_raw_y = (int16_t)((raw[3] << 8) | raw[2]);
    lsm->acc_raw_z = (int16_t)((raw[5] << 8) | raw[4]);

    float acc_x_scaled = lsm->acc_raw_x * 3.9f;
    // float acc_y_scaled = lsm->acc_raw_y * 3.9f;
    float acc_z_scaled = lsm->acc_raw_z * 3.9f;

    //dividing by 1000 to convert from mg to g
    lsm->acc_x = (acc_x_scaled - lsm->acc_offset_x) / 1000.0f;
    // lsm->acc_y = (acc_y_scaled - lsm->acc_offset_y) / 1000.0f;
    lsm->acc_z = (acc_z_scaled - lsm->acc_offset_z) / 1000.0f;

    //using atan2 to calculate angles in degrees
    lsm->angle_x  = atan2(lsm->acc_x, lsm->acc_z) * RAD_TO_DEG;
    //lsm->angle_y = atan2(-lsm->acc_x, sqrt(lsm->acc_y*lsm->acc_y + lsm->acc_z*lsm->acc_z)) * RAD_TO_DEG;
    //lsm->angle_y = atan2(lsm->acc_y, lsm->acc_z) * RAD_TO_DEG;
}

void Offset_LSM(LSM_Data_t* lsm)
{
    uint8_t x_low, x_high, y_low, y_high;
    uint8_t z_low, z_high;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    int samples = 200;

    for(int i = 0; i < samples; i++){
        // ---- READ RAW X ----
        HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_X_L_A, 1, &x_low, 1, 100);
        HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_X_H_A, 1, &x_high, 1, 100);
        int16_t raw_x = (int16_t)(x_high << 8) | x_low;

        // ---- READ RAW Y ----
        // HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Y_L_A, 1, &y_low, 1, 100);
        // HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Y_H_A, 1, &y_high, 1, 100);
        // int16_t raw_y = (int16_t)(y_high << 8) | y_low;

        // ---- READ RAW Z ----
        HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Z_L_A, 1, &z_low, 1, 100);
        HAL_I2C_Mem_Read(&hi2c1, dev_addr, OUT_Z_H_A, 1, &z_high, 1, 100);
        int16_t raw_z = (int16_t)(z_high << 8) | z_low;

        // ---- SCALE TO mg ----
        float scaled_x = raw_x * 3.9f;
        // float scaled_y = raw_y * 3.9f;
        float scaled_z = raw_z * 3.9f;

        // ---- ACCUMULATE ----
        sum_x += scaled_x;
        // sum_y += scaled_y;
        sum_z += scaled_z;

        HAL_Delay(5);  // small delay for stable readings
        }

  // ---- AVERAGE (FINAL OFFSET) ----
  lsm->acc_offset_x = sum_x / samples - 1000.0f;
  lsm->acc_offset_y = sum_y / samples - 1000.0f;
  lsm->acc_offset_z = sum_z / samples - 1000.0f;
}

void gyro_init ()
{
    uint8_t tx [2] = { CTRL_REG1 , CTRL_REG1_VAL };
    // Pull CS low to begin communication
    HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_RESET );

    // Send control register configuration
    HAL_SPI_Transmit (&hspi1 , tx , 2, HAL_MAX_DELAY);

    // Pull CS high to end communication
    HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_SET );
}

void gyro_set_ctrl_reg4 (){
    uint8_t tx [2] = { CTRL_REG4 , CTRL_REG4_VAL };
    HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_RESET );
    HAL_SPI_Transmit (& hspi1 , tx , 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (GPIOE , GPIO_PIN_3 , GPIO_PIN_SET );
}

void gyro_calibrate(LSM_Data_t* lsm){
  int32_t x_sum = 0, y_sum = 0, z_sum = 0;
  const int samples = 200;

  for(int i = 0; i < samples; i++)
  {
    // int16_t x = (int16_t)((spi_read(GYRO_OUT_X_H) << 8) | spi_read(GYRO_OUT_X_L));
    int16_t y = (int16_t)((spi_read(GYRO_OUT_Y_H) << 8) | spi_read(GYRO_OUT_Y_L));
    // int16_t z = (int16_t)((spi_read(GYRO_OUT_Z_H) << 8) | spi_read(GYRO_OUT_Z_L));

    // x_sum += x;
    y_sum += y;
    // z_sum += z;

    HAL_Delay(10);
  }

//   lsm->gyro_offset_x = x_sum / samples;
  lsm->gyro_offset_y = y_sum / samples;
//   lsm->gyro_offset_z = z_sum / samples;
}

uint8_t spi_read(uint8_t reg){
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = reg | 0x80;  // Read command (MSB = 1)
    
    tx[1] = 0x00;

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_HIGH();

    return rx[1];
}

void spi_write(uint8_t reg, uint8_t value){
    uint8_t data[2] = {reg & 0x7F, value};  // Write command (MSB = 0)
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

void Read_Gyro(LSM_Data_t* lsm){
    uint8_t xl, xh, yl, yh, zl, zh;

    // X axis
    // xl = spi_read(GYRO_OUT_X_L);
    // xh = spi_read(GYRO_OUT_X_H);
    // lsm->gyro_x = (int16_t)((xh << 8) | xl) - lsm->gyro_offset_x;

    // Y axis
    yl = spi_read(GYRO_OUT_Y_L);
    yh = spi_read(GYRO_OUT_Y_H);
    lsm->gyro_y = (int16_t)((yh << 8) | yl) - lsm->gyro_offset_y;

    // Z axis
    // zl = spi_read(GYRO_OUT_Z_L);
    // zh = spi_read(GYRO_OUT_Z_H);
    // lsm->gyro_z = (int16_t)((zh << 8) | zl) - lsm->gyro_offset_z;

    // lsm->gyro_x_dps = lsm->gyro_x * 0.00875f;
    lsm->gyro_y_dps = lsm->gyro_y * 0.00875f;
    // lsm->gyro_z_dps = lsm->gyro_z * 0.00875f;
}

float tilt_angle(float dt, float tilt_angle_x, LSM_Data_t* lsm){
    Read_LSM(lsm);
    Read_Gyro(lsm);

    tilt_angle_x = 0.94f * (tilt_angle_x + lsm->gyro_y_dps * dt) + 0.06f * -lsm->angle_x;
    return tilt_angle_x;
} 

