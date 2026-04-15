#ifndef __ANGLE_H
#define __ANGLE_H

#define CTRL_REG1_A 0x20
#define CTRL_REG1_A_VAL 0x67
#define CTRL_REG4_A 0x23
#define CTRL_REG4_A_VAL 0x00

uint8_t dev_addr = 0x33;
uint16_t mem_addr = 0x0f;
uint8_t output;

// Accelerometer output registers
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

// Gyroscope output registers
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D

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
  
  // Offset values (calculated during calibration)
  int16_t acc_offset_x;
  int16_t acc_offset_y;
  int16_t acc_offset_z;
  
  // Gyroscope Values
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;

  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;

  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;

} LSM_Data_t;

void Init_LSM();
void Read_LSM(LSM_Data_t* lsm);
void Offset_LSM(LSM_Data_t* lsm);
void gyro_init();
void gyro_set_ctrl_reg4();
void gyro_calibrate(LSM_Data_t* lsm);
void spi_write(uint8_t reg, uint8_t value);
void Read_Gyro(LSM_Data_t* lsm);
uint8_t spi_read(uint8_t reg);
float tilt_angle(float dt, float tilt_angle_x, LSM_Data_t* lsm);

#endif /* __ANGLE_H */