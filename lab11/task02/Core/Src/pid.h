#ifndef INC_PID_H_
#define INC_PID_H_

/* Struct containing variables for the PID controller */
typedef struct
{
	// User-defined
	float sampling_time; // based on PWM signals and timers configured
	float kp;
	float ki;
	float kd;
	uint8_t is_first_run;

	float proportional;
	float integral;
	float derivative;
	float prev_error;
	float motor_output;

} PIDController;

/* Controller functions */
void PIDController_Init(PIDController * controller);
void PIDController_Update(PIDController * controller, float setpoint, float imu_reading);

#endif /* INC_PID_H_ */