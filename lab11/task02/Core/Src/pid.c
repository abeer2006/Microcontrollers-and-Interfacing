#include "pid.h"
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

/* Initialize PID controller variables */
void PIDController_Init(PIDController *controller)
{
    controller->proportional = 0.0f;
    controller->integral = 0.0f;
    controller->derivative = 0.0f;
    controller->prev_error = 0.0f;
    controller->motor_output = 0.0f;
    controller->is_first_run = 1;   // add this flag to your struct
}

/* Update PID controller */
// void PIDController_Update(PIDController *controller, float setpoint, float imu_reading)
// {
//     // Calculate error
//     float error = setpoint - imu_reading;

//     // Proportional term
//     controller->proportional = controller->kp * error;

//     // Integral term (accumulated over time)
//     controller->integral += controller->ki * error * controller->sampling_time;

//     // --- Optional: Anti-windup (clamping) ---
//     float integral_max = 100.0f;
//     float integral_min = -100.0f;

//     if (controller->integral > integral_max)
//         controller->integral = integral_max;
//     else if (controller->integral < integral_min)
//         controller->integral = integral_min;

//     // Derivative term (rate of change of error)
//     float error_diff = (error - controller->prev_error) / controller->sampling_time;
//     controller->derivative = controller->kd * error_diff;

//     // Compute total output
//     controller->motor_output = controller->proportional + controller->integral + controller->derivative;

//     // Store error for next iteration
//     controller->prev_error = error;
// }

void PIDController_Update(PIDController *controller, float setpoint, float imu_reading)
{
    float error = setpoint - imu_reading;

    controller->proportional = controller->kp * error;

    controller->integral += controller->ki * error * controller->sampling_time;
    if (controller->integral >  100.0f) controller->integral =  100.0f;
    if (controller->integral < -100.0f) controller->integral = -100.0f;

    // Skip derivative on first run to avoid the initial spike
    if (controller->is_first_run)
    {
        controller->derivative   = 0.0f;
        controller->prev_error   = error;
        controller->is_first_run = 0;
    }
    else
    {
        float error_diff = (error - controller->prev_error) / controller->sampling_time;
        controller->derivative = controller->kd * error_diff;
    }

    controller->motor_output = controller->proportional + controller->integral + controller->derivative;
    controller->prev_error = error;
}