#pragma once

#include "main.h"
#include "PID.h"
#include <stdint.h>


#define VREFINT 1.21f
#define ADC_RES_16B 65536.0f
#define ADC_RES_12B 4096.0f
#define CURRENT_SENSE_RESISTOR 0.008f
#define VIN_GAIN 0.0544f // V/V
#define VOUT_GAIN 0.009756f // V/V
#define IL_GAIN 0.16f // V/A



typedef struct
{
    uint8_t ready      :   1;
    uint8_t active     :   1;
    uint8_t OC_fault   :   1;
    uint8_t OV_fault   :   1;
    uint8_t WD_fault   :   1;
    uint8_t            :   3;
    uint16_t vin_10mV;
    uint16_t vout_10mV;
    uint16_t imeas_mA;
}ChargerData;

typedef struct
{
    uint8_t enable       :   1;
    uint8_t clear_faults :   1;
    uint8_t              :   6;
}ReceiveData;

typedef struct 
{
    ChargerData charger_data;
    ReceiveData receive_data;
    PIDController current_controller;

    
    
}Controller;


void Controller_Init(Controller* ctrl);
void Controller_Update(Controller* ctrl);
void Controller_CommunicationHandler(Controller* ctrl);
