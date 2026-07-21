#pragma once

#define ENABLE_PID




// #define PERIOD 1E-5f //10us
// #define KP_I 0.1f
// #define KI_I 40.0f
// #define KD_I 0.00f
// #define OUTPUT_MIN_I 0.0f
// #define OUTPUT_MAX_I 0.90f
// #define CURRENT_SETPOINT 4.0f
// #define VOLTAGE_SETPOINT 200.0f


#define PERIOD 100E-6f //100us
#define KP_I 0.10f
#define KI_I 0.50f
#define OUTPUT_MIN_I 0.0f
#define OUTPUT_MAX_I 200.0f
#define CURRENT_SETPOINT 1.0f
#define VOLTAGE_SETPOINT 200.0f
#define RATELIMIT 1000.0f // A/s

typedef struct {
    float Kp;       // Proportional gain
    float Ki;       // Integral gain
    float setpoint; // Desired target value
    float integral; // Integral term
    float prevError;// Previous error value
    float outputMin;// Minimum output limit
    float outputMax;// Maximum output limit
    float prev_setpoint;
    float min_error;
} PIDController;

float PID_Compute(PIDController* pid, float measurement, float dt);
void PID_SetSetpoint(PIDController* pid, float setpoint);
void PID_Reset(PIDController* pid);


// extern PIDController current_controller;
