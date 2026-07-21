#include "Controller.h"
#include "dfsdm.h"
#include "main.h"
#include "stm32h725xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dfsdm.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"
#include "utils.h"



__section(".RAM_D3") uint32_t Vrefint;
__section(".RAM_D2") volatile uint32_t Imeas;
__section(".RAM_D2") volatile uint32_t Vin;
__section(".RAM_D2") int32_t Iout;
__section(".RAM_D2") int32_t Vout;

static float getVref()
{
    uint32_t vref_adc = Vrefint & 0xFFFF; // Mask to get the lower 16 bits
    if (vref_adc == 0) {
        return 3.3f; // Avoid division by zero, return 0 or handle error as appropriate
    }
    return ADC_RES_12B * VREFINT / (float)vref_adc;
}

void Controller_Init(Controller *ctrl)
{
    HAL_StatusTypeDef status = HAL_OK;

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
    ctrl->current_controller.setpoint = CURRENT_SETPOINT;
    ctrl->current_controller.integral = 0.0f;
    ctrl->current_controller.prevError = 0.0f;
    ctrl->current_controller.outputMin = OUTPUT_MIN_I;
    ctrl->current_controller.outputMax = OUTPUT_MAX_I;

    HAL_GPIO_WritePin(nCLR_OC_GPIO_Port, nCLR_OC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(nWD_RST_GPIO_Port, nWD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(0);
    HAL_GPIO_WritePin(nCLR_OC_GPIO_Port, nCLR_OC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(nWD_RST_GPIO_Port, nWD_RST_Pin, GPIO_PIN_SET);


    status |= HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    status |= HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    status |= HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    status |= HAL_ADC_Start_DMA(&hadc3, &Vrefint, 1);
    status |= HAL_ADC_Start_DMA(&hadc2, &Vin, 1);
    status |= HAL_ADC_Start_DMA(&hadc1, &Imeas, 1);


    status |= HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, &Iout, 1);
    status |= HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, &Vout, 1);

    PID_SetSetpoint(&ctrl->current_controller, CURRENT_SETPOINT);

    // WD Timer
    status |= HAL_TIM_PWM_Start(WD_TIMER, TIM_CHANNEL_1);
    (WD_TIMER)->Instance->CCR1 = (WD_TIMER)->Instance->ARR >> 1; // 50% duty cycle

    //DRV
    status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    status |= HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    TIM1->CCR3 = 0;
    status |= HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1); // Master trigger for TIM8
    TIM1->CCR1 = 0; //Initial phase shift = 0
    status |= HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2); //Master Trigger for ADC sampling (FIXME:)
    TIM1->CCR2 = 0; //Initial phase shift = 0
    status |= HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    status |= HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    TIM8->CCR2 = 0; // 0% duty
    TIM1->CCR3 = 0; // 0% duty

    // Controller Timer
    status |= HAL_TIM_Base_Start_IT(CTRL_TIMER);

    // Start SPI Communication
    status |= HAL_SPI_TransmitReceive_IT(CTRL_SPI_HANDLE, &ctrl->charger_data, &ctrl->receive_data, sizeof(ChargerData));

    
    ctrl->max_counter = 0;
    

    ctrl->charger_data.ready = 1;
}

void Controller_Update(Controller *ctrl)
{
    DWT->CYCCNT = 0; // Reset cycle counter for profiling

    const float V3V3INT = getVref();
    const float vref_div_adc16 = V3V3INT * INV_ADC_RES_16B;

    // Primary measurements
    const float v_in = (float)(Vin & 0xFFFF) * INV_ADC_RES_16B * V3V3INT * INV_VIN_GAIN;
    const float i_prim = (((float)(Imeas & 0xFFFF) * vref_div_adc16) - V3V3INT / 2) * INV_IPRIM_GAIN;

    // Secondary measurements
    const float v_out = ((float)(Vout >> 8) * VOUT_FS) * DIV2Pow23;
    const float i_out = -((float)(Iout >> 8) * IOUT_FS) * DIV2Pow23;


    ChargerData *cdata = &ctrl->charger_data;
    ReceiveData *rx = &ctrl->receive_data;

    cdata->vin_10mV = (uint16_t)(v_in * 100.0f);
    cdata->vout_10mV = (uint16_t)(v_out * 100.0f);
    cdata->imeas_mA = (uint16_t)(i_out * 1000.0f);

    ctrl->Vin = v_in;
    ctrl->Imeas = i_prim;
    ctrl->Vout = v_out;
    ctrl->Iout = i_out;

    const uint8_t ov_fault = v_out > 220.0f;
    const uint8_t oc_sw_fault = i_prim > 10.0f;
    const uint8_t oc_hw_fault = HAL_GPIO_ReadPin(OC_GPIO_Port, OC_Pin);
    const uint8_t wd_fault = !HAL_GPIO_ReadPin(WD_SNS_GPIO_Port, WD_SNS_Pin);

    // Over Voltage fault
    cdata->OV_fault |= ov_fault;

    // Over Current fault
    if (oc_sw_fault) {
        cdata->OC_fault = 1;
        ctrl->oc_sw = 1;
    } else if (oc_hw_fault) {
        cdata->OC_fault = 1;
        ctrl->oc_hw = 1;
    }

    // Watchdog fault
    cdata->WD_fault |= wd_fault;

    // Charger timeout fault
    if (ctrl->timeout_counter > 10 * CHARGER_TIMEOUT_MS) {
        cdata->timeout = 1;
    }

    const uint8_t has_fault = cdata->OV_fault | cdata->OC_fault | cdata->WD_fault | cdata->timeout;
    if (has_fault) {
        if (rx->clear_faults && !cdata->active) {
            clear_faults(ctrl);
        } else {
            handle_faults(ctrl);
        }
    } else if (rx->enable) {
        const float voltage_threshold = 0.95f * VOLTAGE_SETPOINT;

        // Re-Enable charger if below threshold
        if (v_out < voltage_threshold) {
            cdata->active = 1;
            setStatusLED(0, 255, 0);
        }

        if (cdata->active) {
            TIM8->CCR2 = TIM8->ARR >> 1; // 50% duty
            TIM1->CCR3 = TIM1->ARR >> 1; //50% duty
            ++ctrl->timeout_counter;


            float Vreg = PID_Compute(&ctrl->current_controller, i_out, PERIOD);
            Vreg += v_out; // Feedforward
            Vreg = CLAMP(Vreg, 0.0f, n * v_in);
            const float phi = Vreg / (n * v_in / PI);
            set_phase_shift_rad(phi);

            // Primary Current Trigger
            TIM1->CCR2 = TIM1->CCR1 >> 1; // set ADC trigger to be in the middle of the conduction period

            // Max Charge Reached
            if (v_out > VOLTAGE_SETPOINT) {
                set_phase_shift_rad(0.0f);
                setStatusLED(0, 0, 255);
                cdata->active = 0;
                ctrl->timeout_counter = 0;
                PID_Reset(&ctrl->current_controller);
            }
        }
        else
        {
            TIM8->CCR2 = 0; // 0% duty
            TIM1->CCR3 = 0; // 0% duty
        }
    } else {
        set_phase_shift_rad(0.0f);
        setStatusLED(0, 0, 0);
        TIM8->CCR2 = 0; // 0% duty
        TIM1->CCR3 = 0; // 0% duty
        cdata->active = 0;
        PID_Reset(&ctrl->current_controller);
    }

    if (DWT->CYCCNT > ctrl->max_counter) {
        ctrl->max_counter = DWT->CYCCNT;
    }
}

void Controller_CommunicationHandler(Controller *ctrl) 
{
    ctrl->data_valid = 0;
    HAL_SPI_TransmitReceive_IT(CTRL_SPI_HANDLE, &ctrl->charger_data, &ctrl->receive_data, sizeof(ChargerData));
}

void clear_faults(Controller *ctrl) 
{
    ChargerData *cdata = &ctrl->charger_data;
    cdata->OV_fault = 0;
    cdata->OC_fault = 0;
    cdata->WD_fault = 0;
    cdata->timeout = 0;

    HAL_GPIO_WritePin(nWD_RST_GPIO_Port, nWD_RST_Pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 100000; i++);
    HAL_GPIO_WritePin(nWD_RST_GPIO_Port, nWD_RST_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(nCLR_OC_GPIO_Port, nCLR_OC_Pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 100000; i++);
    HAL_GPIO_WritePin(nCLR_OC_GPIO_Port, nCLR_OC_Pin, GPIO_PIN_SET);

    setStatusLED(0, 0, 0);
}

void handle_faults(Controller *ctrl) 
{
    ChargerData *cdata = &ctrl->charger_data;
    cdata->active = 0;
    ctrl->timeout_counter = 0;
    TIM8->CCR2 = 0; // 0% duty
    TIM1->CCR3 = 0; // 0% duty
    setStatusLED(255, 0, 0);
    set_phase_shift_rad(0.0f);
}

void set_phase_shift_rad(float rad) 
{
    CLAMP(rad, 0.0f, PI);
    TIM1->CCR1 = (uint32_t)((rad / (2.0f * PI)) * TIM1->ARR);
}
