#include "PID.h"


void PID_Init(PIDController *pid)
{
    pid->Kp = KP;
    pid->Ki = KI;
    pid->Kd = KD;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->outputMin = OUTPUT_MIN;
    pid->outputMax = OUTPUT_MAX;
}

float PID_Compute(PIDController *pid, float measurement)
{
#ifdef ENABLE_PID
    float error = pid->setpoint - measurement;
    pid->integral += error * PERIOD;
    float derivative = (error - pid->prevError) / PERIOD;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Clamp output to min/max
    if (output > pid->outputMax) {
        output = pid->outputMax;
        // Anti-windup: prevent integral from increasing further
        if (error > 0) {
            pid->integral -= error * PERIOD; // Undo last integral addition
        }
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
        // Anti-windup: prevent integral from decreasing further
        if (error < 0) {
            pid->integral -= error * PERIOD; // Undo last integral addition
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
