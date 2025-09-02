#pragma once

// #define ENABLE_PID

#define PERIOD 1E-5 //10us
#define KP 0.1f
#define KI 40.0f
#define KD 0.00f
#define OUTPUT_MIN 0.0f
#define OUTPUT_MAX 0.90f
#define CURRENT_SETPOINT 4.0f

typedef struct {
    float Kp;       // Proportional gain
    float Ki;       // Integral gain
    float Kd;       // Derivative gain
    float setpoint; // Desired target value
    float integral; // Integral term
    float prevError;// Previous error value
    float outputMin;// Minimum output limit
    float outputMax;// Maximum output limit
} PIDController;

void PID_Init(PIDController* pid);
float PID_Compute(PIDController* pid, float measurement);
void PID_SetSetpoint(PIDController* pid, float setpoint);
void PID_Reset(PIDController* pid);
