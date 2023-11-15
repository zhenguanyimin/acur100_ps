/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    wave_config.c
 * @author  X22012
 * @date    2022骞�07鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�07鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "xil_io.h"
#include "xparameters.h"
#include "wave_config.h"
#include "../../hal/CHC2442Module/CHC2442_driver/chc2442_common.h"
#include "../../hal/CHC2442Module/CHC2442_config/chc2442_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_ramp_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_power_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_config_register.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_divider_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_mux_pin_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CHC2442_REGISTER_VALUE                0XEB23F0

#define RAMP_LIMIT_HIGH_VALUE                 0X1000000       // 1550MHz
#define RAMP_LIMIT_LOW_VALUE                  0X0             // 1500MHz

// Set VCO output frequency value
#define PLL_R_DIVIDER                         1U
#define PLL_N_DIVIDER                         30U
#define CHARGE_PUMP_GAIN_VALUE                1U
#define FRACTIONAL_MODUALATOR_ORDER			  3U
#define FRACTIONAL_MODUALATOR_DITHER     	  3U
//#define CHARGE_PUMP_GAIN_VALUE                31

#define RAMP_START_FREQUENCY                  1500            // 1500MHz

// Single frequency point waveform
#define SINGLE_DESIRED_END_FREQUENCY          RAMP_START_FREQUENCY
#define SINGLE_RAMP_INCREMENT                 0
#define SINGLE_RAMP_COUNT                     0
#define SINGLE_RAMP_DURATION                  100            // 100us

// Target waveform for Anti-UVA
#define TARGECT_WAVEFORM_BANDWIDTH            50             // 50MHz
#define RAMP0_DIFFERENT_FREQUENCY             0              
#define RAMP1_DIFFERENT_FREQUENCY             3125           // 3.125MHz
#define RAMP2_DIFFERENT_FREQUENCY             (-3125)        // -3.125MHz
#define RAMP3_DIFFERENT_FREQUENCY             0 

#define TARGECT_WAVEFORM_RAMP0_DURATION       20             // 20us
#define TARGECT_WAVEFORM_RAMP1_DURATION       205            // 205us
#define TARGECT_WAVEFORM_RAMP2_DURATION       10             // 30us

#define TARGECT_WAVEFORM_IDLE_TIME_20US       20             // 20us
#define TARGECT_WAVEFORM_IDLE_TIME_50US       50             // 50us

#define TARGECT_FRACTIONAL_DENOMINATOR        0X1000000
//#define TARGECT_WAVEFORM_CHIRP_COUNT          (1 + 3 * 32)
#define TARGECT_WAVEFORM_CHIRP_COUNT          0

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
static lmx2492_status_t set_ramp_limit_parameter(void)
{
	lmx2492_status_t status = LMX2492TransStatusOk;

    uint64_t ramp_limit_high = RAMP_LIMIT_HIGH_VALUE;
    uint64_t ramp_limit_low  = RAMP_LIMIT_LOW_VALUE;
    uint16_t PLL_R_divider = PLL_R_DIVIDER;
    uint32_t PLL_N_divider = PLL_N_DIVIDER;
    uint32_t PLL_den_val = TARGECT_FRACTIONAL_DENOMINATOR;
    uint8_t  charge_pump_gain = CHARGE_PUMP_GAIN_VALUE;
    uint8_t frac_order = FRACTIONAL_MODUALATOR_ORDER;
    uint8_t frac_dither = FRACTIONAL_MODUALATOR_DITHER;

    status = set_ramp_limit_frequency(ramp_limit_high, ramp_limit_low);
    status = set_vco_output_frequency(PLL_R_divider, PLL_N_divider, charge_pump_gain, PLL_den_val, frac_order, frac_dither);
    // To successfully configure the waveform, you must rewrite the R2 register
    LMX2492_power_up_setting();

    return status;
}


/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
//uint32_t get_chc2442_register(uint16_t regAddr)
//{
//	uint32_t val = 0;
//
//	val = LMX2492_read_single_register(regAddr);
//
//	return val;
//}
//
void set_chc2442_value(uint32_t value)
{

	CHC2442_writing(value);
}

uint8_t get_single_2492_register(uint16_t regAddr)
{
	uint8_t val = 0;

    if (regAddr < 0 || regAddr > 142)
    {
    	return 0XFF;
    }

	val = LMX2492_read_single_register(regAddr);

	return val;
}

void set_single_2492_register(uint16_t regAddr, uint8_t value)
{

	LMX2492_set_single_register(regAddr, value);
}


void VCOPLLInitFunction(void)
{
	uint32_t chc2442_val = CHC2442_REGISTER_VALUE;

	(void)set_chc2442_value(chc2442_val);
	(void)ramp_functions_enable();
	(void)set_pin_MUXout_readback();
	(void)set_ramp_limit_parameter();
//	LMX2492_write_all_register();

}

lmx2492_status_t set_pin_MUXout_readback(void)
{
	lmx2492_status_t status = LMX2492TransStatusOk;

	lmx2492_mux_drive_pin_state_index_t drive_idx = PullUpPullDownOutputIndex;
	lmx2492_mux_state_index_t state_idx = OutputInReadbackIndex;

	status = muxout_drive_state_setting(drive_idx);
	status = muxout_state_setting(state_idx);

    return status;
}

lmx2492_status_t single_frequency_waveform_configuration(wave_fre_point_t frePoint)
{
    lmx2492_status_t status;

    uint16_t ramp_duration = SINGLE_RAMP_DURATION;

    ramp_configuration_parameter_t ramp0_para = {
        .increment = SINGLE_RAMP_INCREMENT,
        .fastlock_flag = RampDelayDisable,
        .flag_index = RampFlagsClear,
        .reset_flag = RampResetEnable,
        .trigger_index = RampxLength,
        .next_ramp_index = Ramp0Index };

    ramp0_para.length = get_ramp_length(ramp_duration);

    status = single_frequency_waveform_parameter(&ramp0_para, frePoint);

    return status;

}

//lmx2492_status_t target_waveform_configuration(uint16_t ramp0_dura, uint16_t ramp1_dura, uint16_t ramp0_diff_fre, uint16_t ramp1_diff_fre, uint16_t ramp_cnt)
lmx2492_status_t target_waveform_configuration(bool isTrigFlag)
//lmx2492_status_t target_waveform_configuration(uint16_t idle_time)
{
    
	lmx2492_status_t status = LMX2492TransStatusOk;

    uint16_t ramp_count = TARGECT_WAVEFORM_CHIRP_COUNT;
    uint16_t ramp0_duration = TARGECT_WAVEFORM_RAMP0_DURATION;
    uint16_t ramp1_duration = TARGECT_WAVEFORM_RAMP1_DURATION;
    uint16_t ramp2_duration = TARGECT_WAVEFORM_RAMP2_DURATION;
//    uint16_t ramp3_duration = idle_time;
    uint16_t ramp0_diff_fre = RAMP0_DIFFERENT_FREQUENCY;
    uint16_t ramp1_diff_fre = RAMP1_DIFFERENT_FREQUENCY;
    uint16_t ramp2_diff_fre = RAMP2_DIFFERENT_FREQUENCY;
//    uint16_t ramp3_diff_fre = RAMP3_DIFFERENT_FREQUENCY;

    ramp_configuration_parameter_t ramp0_parameter = {
        .fastlock_flag = RampDelayDisable,
        .flag_index = RampFlagsClear,
        .reset_flag = RampResetEnable,              // ramp0 reset
        .trigger_index = RampxLength,
        .next_ramp_index = Ramp1Index };

    ramp_configuration_parameter_t ramp1_parameter = {
        .fastlock_flag = RampDelayDisable,
        .flag_index = RampFlagsClear,
        .reset_flag = RampResetEnable,              // ramp0 reset
        .trigger_index = RampxLength,
        .next_ramp_index = Ramp2Index };

    ramp_configuration_parameter_t ramp2_parameter = {
        .fastlock_flag = RampDelayDisable,
        .flag_index = RampFlagsClear,
        .reset_flag = RampResetDisable,
        .trigger_index = RampxLength,
        .next_ramp_index = Ramp0Index };

//    ramp_configuration_parameter_t ramp3_parameter = {
//        .fastlock_flag = RampDelayDisable,
//        .flag_index = RampFlagsClear,
//        .reset_flag = RampResetDisable,
//        .trigger_index = RampxLength,
//        .next_ramp_index = Ramp1Index };

//     Set MOD_pin to TriggerA
    if (isTrigFlag)
    {
 	   ramp0_parameter.trigger_index = TriggerA;
    }

    ramp0_parameter.length = get_ramp_length(ramp0_duration);
    ramp1_parameter.length = get_ramp_length(ramp1_duration);
    ramp2_parameter.length = get_ramp_length(ramp2_duration);
//	ramp3_parameter.length = get_ramp_length(ramp3_duration);
    ramp0_parameter.increment = get_ramp_increment(ramp0_diff_fre, ramp0_parameter.length);
    ramp1_parameter.increment = get_ramp_increment(ramp1_diff_fre, ramp1_parameter.length);
    ramp2_parameter.increment = get_ramp_increment(ramp2_diff_fre, ramp2_parameter.length);
//	ramp3_parameter.increment = get_ramp_increment(ramp3_diff_fre, ramp3_parameter.length);

    status = target_waveform_parameter(ramp_count, &ramp0_parameter, &ramp1_parameter, &ramp2_parameter);

    return status;
}


       
