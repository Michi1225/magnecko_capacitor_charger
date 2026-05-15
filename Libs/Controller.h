#pragma once

#include "main.h"
#include "PID.h"
#include <stdint.h>


#define VREFINT 1.21f
#define ADC_RES_16B 65536.0f
#define INV_ADC_RES_16B 15.2587891E-6f
#define ADC_RES_12B 4096.0f
#define INV_ADC_RES_12B 244.140625E-6f
#define CURRENT_SENSE_RESISTOR 0.008f
#define VIN_GAIN 0.0544f // V/V
#define INV_VIN_GAIN 18.38235294f // V/V
#define VOUT_GAIN 0.009756f // V/V
#define INV_VOUT_GAIN 102.476f // V/V
#define IL_GAIN 0.16f // V/A
#define INV_IL_GAIN 6.25f // A/V

#define CHARGER_TIMEOUT_MS 500 //smaller than 655ms to fit in uint16_t



typedef struct __attribute((packed))
{
    uint8_t ready      :   1;
    uint8_t active     :   1;
    uint8_t OC_fault   :   1;
    uint8_t OV_fault   :   1;
    uint8_t WD_fault   :   1;
    uint8_t timeout   :   1;
    uint8_t            :   2;
    uint16_t vin_10mV;
    uint16_t vout_10mV;
    uint16_t imeas_mA;
}ChargerData;

typedef struct __attribute((packed))
{
    uint8_t enable       :   1;
    uint8_t clear_faults :   1;
    uint8_t              :   6;
    uint16_t reserved[3];
}ReceiveData;

typedef struct 
{
    ChargerData charger_data;
    ReceiveData receive_data;
    PIDController current_controller;

    uint8_t data_valid;

    float Vout;
    float Imeas;
    float Vin;

    uint16_t timeout_counter;

    uint8_t oc_sw, oc_hw;

    uint32_t max_counter;

    
    
}Controller;


void Controller_Init(Controller* ctrl);
void Controller_Update(Controller* ctrl);
void Controller_CommunicationHandler(Controller* ctrl);
