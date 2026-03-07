#include "Controller.h"
#include "stm32h725xx.h"
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
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    TIM8->CCR4 = 0;

    // Controller Timer
    HAL_TIM_Base_Start_IT(&htim4);

    

    ctrl->charger_data.ready = 1;
}

void Controller_Update(Controller *ctrl)
{
    //Get Measurements
    float VREF = getVref();

    float Vadc = (float)(Imeas) / ADC_RES_16B * VREF;
    float current = (VREF - Vadc) / IL_GAIN; //A

    uint32_t Vin = ADC3_Buffer[0];
    Vadc = (float)(Vin) / ADC_RES_12B * VREF; //V
    float Input = Vadc / VIN_GAIN; //V

    Vadc = ((float)(Vout) - ADC_RES_16B / 2.0f) / (ADC_RES_16B / 2.0f) * VREF; //V
    float Output = Vadc / VOUT_GAIN; //V

    ctrl->charger_data.vin_10mV = (uint16_t)(Input * 100.0f);
    ctrl->charger_data.vout_10mV = (uint16_t)(Output * 100.0f);
    ctrl->charger_data.imeas_mA = (uint16_t)(current * 1000.0f);


    //Check Fault Conditions
    if(Output > 220.0f) ctrl->charger_data.OV_fault = 1;
    if(current > 10.0f) ctrl->charger_data.OC_fault = 1;
    if(ctrl->charger_data.OV_fault || ctrl->charger_data.OC_fault)
    {
        // Only clear faults if charger is inactive.
        if(ctrl->receive_data.clear_faults && !ctrl->charger_data.active)
        {
            ctrl->charger_data.OV_fault = 0;
            ctrl->charger_data.OC_fault = 0;

            // Reset WD
            HAL_GPIO_WritePin(WD_nRST_GPIO_Port, WD_nRST_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(WD_nRST_GPIO_Port, WD_nRST_Pin, GPIO_PIN_SET);
            
            // Clear OC Fault Latch in DRV
            HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_SET);
        }
        // Fault active, stop charging
        else
        {
            ctrl->charger_data.active = 0;
            TIM8->CCR4 = 0;
            return;
        }
    }

    //Run Controller
    if(ctrl->charger_data.active && ctrl->receive_data.enable)
    {
        //Run Current Current Controller at 100kHz
        float d = PID_Compute(&ctrl->current_controller, current, PERIOD);
        TIM8->CCR4 = TIM8->ARR * d;

        //Setpoint check
        if(Output > VOLTAGE_SETPOINT)//Setpoint reached
        {
            TIM8->CCR4 = 0;
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
            ctrl->charger_data.active = 0;
            PID_Reset(&ctrl->current_controller);
            return;
        }

    }
    else if(Vout < 0.95f * VOLTAGE_SETPOINT && ctrl->receive_data.enable)
    {
        ctrl->charger_data.active = 1;
    }
}

void Controller_CommunicationHandler(Controller *ctrl) 
{
    uint8_t txdata[8];
    memcpy(txdata, &ctrl->charger_data, sizeof(ChargerData));
    uint8_t rxdata[7];
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, txdata, rxdata, 7) == HAL_OK)
    {
        while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY 
        || HAL_DMA_GetState(((&hspi1)->hdmarx)) != HAL_DMA_STATE_READY
        || HAL_DMA_GetState(((&hspi1)->hdmatx)) != HAL_DMA_STATE_READY) __NOP();
        memcpy(&ctrl->receive_data, rxdata, sizeof(ReceiveData));
    }
}
