#include "PID.h"
#include "main.h"
#include "stm32h7xx_hal_gpio.h"

PIDController current_controller =
{
    .Kp = KP_I,
    .Ki = KI_I,
    .Kd = KD_I,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prevError = 0.0f,
    .outputMin = OUTPUT_MIN_I,
    .outputMax = OUTPUT_MAX_I,
    .prev_setpoint = 0.0f,
    .min_error = 0.0f
};

float PID_Compute(PIDController *pid, float measurement, float dt)
{
#ifdef ENABLE_PID
    const float max_step = RATELIMIT * dt;
    float setpoint = pid->prev_setpoint;
    if (pid->setpoint > setpoint + max_step) {
        setpoint += max_step;
    } else if (pid->setpoint < setpoint - max_step) {
        setpoint -= max_step;
    } else {
        setpoint = pid->setpoint;
    }
    pid->prev_setpoint = setpoint;

    const float error = setpoint - measurement;

    pid->min_error = fminf(pid->min_error, error);

    const float integral = pid->integral + error * dt;
    const float output_raw = pid->Kp * error + pid->Ki * integral;
    float output = output_raw;

    if (output_raw > pid->outputMax) {
        if (error < 0.0f) {
            pid->integral = integral;
        }
        output = pid->outputMax;
    } else if (output_raw < pid->outputMin) {
        if (error > 0.0f) {
            pid->integral = integral;
        }
        output = pid->outputMin;
    } else {
        pid->integral = integral;
    }

    return output;
#else
    return 0.0f;
#endif
}

void PID_SetSetpoint(PIDController *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Reset(PIDController *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}
