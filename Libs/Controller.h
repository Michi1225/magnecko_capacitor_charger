#pragma once

#include "main.h"
#include "PID.h"
#include <stdint.h>
#include "utils.h"

#define CLAMP(x, min, max) ((x < min) ? min : ((x > max) ? max : x))
#define PI 3.14159265f

#define n 5 // Turns ratio


#define VREFINT 1.21f
#define VREF_EXT 1.65f

#define ADC_RES_16B 65536.0f
#define INV_ADC_RES_16B 15.2587891E-6f
#define ADC_RES_12B 4096.0f
#define INV_ADC_RES_12B 244.140625E-6f


// Primary side Measurements
#define VIN_GAIN 0.053030303f // V/V
#define INV_VIN_GAIN 18.85714287f // V/V

#define IPRIM_GAIN 0.14f // V/A
#define INV_IPRIM_GAIN 7.14285714f // A/V

// Secondary side measurements
#define Pow23 8388608.0f //2^23, since DFSDM is in 24-bit mode but signed
#define DIV2Pow23 1.1920929E-7f //1/2^23
#define DIV2Pow24 5.96046448E-8f //1/2^24

#define VOUT_FS 313.125f // Full scale voltage: 1V *(5*499k +10k)/10k @ 80% FS
#define IOUT_FS 3.125f //Full scale current: 50mV / 20mOhm @ 80% FS

#define CHARGER_TIMEOUT_MS 500 //smaller than 655ms to fit in uint16_t

#define WD_TIMER &htim4
#define CTRL_TIMER &htim2

#define CTRL_SPI_HANDLE &hspi3



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
    uint32_t crc;
}ChargerData;

typedef struct __attribute((packed))
{
    uint8_t enable       :   1;
    uint8_t clear_faults :   1;
    uint8_t              :   6;
    uint16_t signature[3];
    uint32_t crc
}ReceiveData;

typedef struct 
{
    ChargerData charger_data;
    ChargerData tx_data;
    ReceiveData control_data;
    ReceiveData rx_data;
    PIDController current_controller;

    uint8_t data_valid;

    float Vout;
    float Imeas;
    float Vin;
    float Iout;

    uint16_t timeout_counter;

    uint8_t oc_sw, oc_hw;

    uint32_t max_counter;

    
    
}Controller;


void Controller_Init(Controller* ctrl);
void Controller_Update(Controller* ctrl);
void Controller_CommunicationHandler(Controller* ctrl);

void rx_data_validate(Controller* ctrl);

void clear_faults(Controller* ctrl);
void handle_faults(Controller* ctrl);
void set_phase_shift_rad(float rad);
