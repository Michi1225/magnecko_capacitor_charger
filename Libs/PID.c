#include "PID.h"

PIDController current_controller =
{
    .Kp = KP_I,
    .Ki = KI_I,
    .Kd = KD_I,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prevError = 0.0f,
    .outputMin = OUTPUT_MIN_I,
    .outputMax = OUTPUT_MAX_I
};

float PID_Compute(PIDController *pid, float measurement, float dt)
{
#ifdef ENABLE_PID
    float error = pid->setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Clamp output to min/max
    if (output > pid->outputMax) {
        output = pid->outputMax;
        // Anti-windup: prevent integral from increasing further
        if (error > 0) {
            pid->integral -= error * dt; // Undo last integral addition
        }
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
        // Anti-windup: prevent integral from decreasing further
        if (error < 0) {
            pid->integral -= error * dt; // Undo last integral addition
        }
    }

    pid->prevError = error;
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
