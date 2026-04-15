#ifndef MOTOR_H
#define MOTOR_H

void RightMotor_Forward(uint16_t speed);
void RightMotor_Backward(uint16_t speed);
void RightMotor_Stop();
void LeftMotor_Forward(uint16_t speed);
void LeftMotor_Backward(uint16_t speed);    
void LeftMotor_Stop(void);

#endif /* MOTOR_H */