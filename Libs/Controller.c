#include "Controller.h"
#include "stm32h725xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"



__section(".RAM_D3") uint32_t ADC3_Buffer[2];
__section(".RAM") uint32_t Imeas;
__section(".RAM") uint32_t Vout;

static float getVref()
{
    uint32_t vref_adc = ADC3_Buffer[1];
    return ADC_RES_12B * VREFINT / (float)vref_adc;
}

void Controller_Init(Controller *ctrl)
{
    ctrl->charger_data.ready = 0;
    ctrl->charger_data.active = 0;
    ctrl->charger_data.OC_fault = 0;
    ctrl->charger_data.OV_fault = 0;
    ctrl->charger_data.WD_fault = 0;
    ctrl->charger_data.vin_10mV = 0;
    ctrl->charger_data.vout_10mV = 0;
    ctrl->charger_data.imeas_mA = 0;

    ctrl->receive_data.enable = 0;
    ctrl->receive_data.clear_faults = 0;

    ctrl->current_controller.Kp = KP_I;
    ctrl->current_controller.Ki = KI_I;
    ctrl->current_controller.Kd = KD_I;
    ctrl->current_controller.setpoint = CURRENT_SETPOINT;
    ctrl->current_controller.integral = 0.0f;
    ctrl->current_controller.prevError = 0.0f;
    ctrl->current_controller.outputMin = OUTPUT_MIN_I;
    ctrl->current_controller.outputMax = OUTPUT_MAX_I;

    HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WD_nRST_GPIO_Port, WD_nRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(0);
    HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WD_nRST_GPIO_Port, WD_nRST_Pin, GPIO_PIN_SET);


    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_DIFFERENTIAL_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc3, ADC3_Buffer, 2);
    HAL_ADC_Start_DMA(&hadc2, &Imeas, 1);
    HAL_ADC_Start_DMA(&hadc1, &Vout, 1);
    
    PID_SetSetpoint(&ctrl->current_controller, CURRENT_SETPOINT);

    // WD Timer
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    TIM1->CCR1 = 500;
    
    
    HAL_TIM_Base_Start_IT(&htim2);

    //DRV
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    TIM8->CCR4 = 0;
    HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_1); //TRGO for ADC
    TIM8->CCR1 = 250; //TRGO for ADC

    // Controller Timer
    HAL_TIM_Base_Start_IT(&htim4);

    
    ctrl->max_counter = 0;
    

    ctrl->charger_data.ready = 1;
}

void Controller_Update(Controller *ctrl)
{
    DWT->CYCCNT = 0; // Reset cycle counter for profiling

    const float VREF = getVref();
    const float vref_div_adc16 = VREF * INV_ADC_RES_16B;
    const float Input = (float)ADC3_Buffer[0] * INV_ADC_RES_12B * VREF * INV_VIN_GAIN;
    const float Output = (((float)Vout - (ADC_RES_16B * 0.5f)) * vref_div_adc16 * 2.0f) * INV_VOUT_GAIN;
    const float current = (VREF - ((float)Imeas * vref_div_adc16)) * INV_IL_GAIN;

    ChargerData *cdata = &ctrl->charger_data;
    ReceiveData *rx = &ctrl->receive_data;

    cdata->vin_10mV = (uint16_t)(Input * 100.0f);
    cdata->vout_10mV = (uint16_t)(Output * 100.0f);
    cdata->imeas_mA = (uint16_t)(current * 1000.0f);

    ctrl->Vin = Input;
    ctrl->Imeas = current;
    ctrl->Vout = Output;

    const uint8_t ov_fault = Output > 220.0f;
    const uint8_t oc_sw_fault = current > 10.0f;
    const uint8_t oc_hw_fault = HAL_GPIO_ReadPin(OC_GPIO_Port, OC_Pin);
    const uint8_t wd_fault = !HAL_GPIO_ReadPin(WD_SNS_GPIO_Port, WD_SNS_Pin);

    cdata->OV_fault |= ov_fault;
    if (oc_sw_fault) {
        cdata->OC_fault = 1;
        ctrl->oc_sw = 1;
    } else if (oc_hw_fault) {
        cdata->OC_fault = 1;
        ctrl->oc_hw = 1;
    }
    cdata->WD_fault |= wd_fault;
    if (ctrl->timeout_counter > 100 * CHARGER_TIMEOUT_MS) {
        cdata->timeout = 1;
    }

    const uint8_t has_fault = cdata->OV_fault | cdata->OC_fault | cdata->WD_fault | cdata->timeout;
    if (has_fault) {
        if (rx->clear_faults && !cdata->active) {
            cdata->OV_fault = 0;
            cdata->OC_fault = 0;
            cdata->WD_fault = 0;
            cdata->timeout = 0;

            HAL_GPIO_WritePin(WD_nRST_GPIO_Port, WD_nRST_Pin, GPIO_PIN_RESET);
            for (volatile int i = 0; i < 100000; i++);
            HAL_GPIO_WritePin(WD_nRST_GPIO_Port, WD_nRST_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_RESET);
            for (volatile int i = 0; i < 100000; i++);
            HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        } else {
            cdata->active = 0;
            ctrl->timeout_counter = 0;
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            TIM8->CCR4 = 0;
        }
    } else if (rx->enable) {
        const float voltage_threshold = 0.95f * VOLTAGE_SETPOINT;
        if (Output < voltage_threshold) {
            cdata->active = 1;
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }

        if (cdata->active) {
            ++ctrl->timeout_counter;
            const float d = PID_Compute(&ctrl->current_controller, current, PERIOD);
            TIM8->CCR4 = (uint32_t)(TIM8->ARR * d);
            if(TIM8->CCR4 < TIM8->ARR >> 1) TIM8->CCR1 = (uint32_t)(0.5f * (TIM8->ARR - TIM8->CCR4)); // Adjust ADC trigger based on duty cycle
            else TIM8->CCR1 = (uint32_t)(0.5f * TIM8->CCR4);

            if (Output > VOLTAGE_SETPOINT) {
                TIM8->CCR4 = 0;
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
                cdata->active = 0;
                PID_Reset(&ctrl->current_controller);
                ctrl->timeout_counter = 0;
            }
        }
    } else {
        TIM8->CCR4 = 0;
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        cdata->active = 0;
        PID_Reset(&ctrl->current_controller);
    }

    if (DWT->CYCCNT > ctrl->max_counter) {
        ctrl->max_counter = DWT->CYCCNT;
    }
}

void Controller_CommunicationHandler(Controller *ctrl) 
{
    // uint8_t txdata[8];
    // memcpy(txdata, &ctrl->charger_data, sizeof(ChargerData));
    // uint8_t rxdata[7];
    ctrl->data_valid = 0;
    HAL_SPI_TransmitReceive_IT(&hspi1, &ctrl->charger_data, &ctrl->receive_data, sizeof(ChargerData));
}
