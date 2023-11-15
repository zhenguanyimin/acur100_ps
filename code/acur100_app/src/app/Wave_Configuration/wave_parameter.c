/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    wave_parameter.c
 * @author  X22012
 * @date    2022濡ょ姷鍎戦幏锟�07闂佸搫鐗為幏锟�15闂佸搫鍠涢幏锟�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022濡ょ姷鍎戦幏锟�07闂佸搫鐗為幏锟�15闂佸搫鍠涢幏锟�
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "xil_io.h"
#include "sleep.h"
#include "xparameters.h"
#include "wave_parameter.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_config_register.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_mux_pin_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_ramp_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_divider_config.h"
#include "../../hal/LMX2492Module/LMX2492_config/lmx2492_individual_ramp_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PHASE_DETECT_Frequency			      50              // 50MHz
#define RAMP_MAX_LENGTH			              1310            // 1310us(RAMP_DLY is 0)
#define FRANCTIONAL_DENOMIANTOR               16777216        // 2^24
#define RAMP_INCREMENT_MAX                    1073741824      // 2^30

#define SINGLE_FRAC_24G_NUM_VALUE		  	  0X000000        // 24GHz
#define SINGLE_FRAC_24_15G_NUM_VALUE		  0X300000        // 24.15GHz
#define SINGLE_FRAC_24_25G_NUM_VALUE		  0X500000        // 24.25GHz

#ifndef XPAR_AXI2REG2492_0_BASEADDR
#define XPAR_AXI2REG2492_0_BASEADDR (0x50) // TODO: fixme
#endif /* XPAR_AXI2REG2492_0_BASEADDR */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
static lmx2492_status_t waveform_configuration(lmx2492_ramp_index_t index, ramp_configuration_parameter_t *parameter)
{
    if (parameter == NULL)
    {
        return LMX2492TransStatusError;
    }
    lmx2492_status_t status;

    // Set rampx increment
    status = rampx_increment_value_setting(index, parameter->increment);
    // fastlock and cycle slip reduction for ramp x
    if (parameter->fastlock_flag)
    {
        status = rampx_fastlock_enable(index);
    }
    // Number of PFD clocks
    status = rampx_length_setting(index, parameter->length);
    // ramp flag set
    status = rampx_flag_setting(index, parameter->flag_index);
    // ramp reset set
    if (parameter->reset_flag)
    {
        status = rampx_reset_enable(index);
    }
    // Next ramp trigger
    status = rampx_next_trigger_setting(index, parameter->trigger_index);
    // Next ramp
    status = rampx_next_setting(index, parameter->next_ramp_index);

    return status;
    
}

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
uint16_t get_ramp_length(uint16_t duration)
{
    if (duration > RAMP_MAX_LENGTH) {
        duration = RAMP_MAX_LENGTH;
    }

    return duration * PHASE_DETECT_Frequency;
}


uint32_t get_ramp_increment(int16_t diff_fre, uint16_t ramp_len)
{
    uint32_t increment;
    uint32_t frac_deno = FRANCTIONAL_DENOMIANTOR;

    increment = (frac_deno / PHASE_DETECT_Frequency * abs(diff_fre) / 1000) / ramp_len;
    
    return (diff_fre < 0) ? (RAMP_INCREMENT_MAX - increment) : increment;

}

lmx2492_status_t set_ramp_limit_frequency(uint64_t limit_high, uint64_t limit_low)
{
    lmx2492_status_t statue; 

    statue = ramp_limit_high_setting(limit_high);
    statue = ramp_limit_low_setting(limit_low);

    return statue;
}

lmx2492_status_t set_vco_output_frequency(uint16_t R_divider, uint32_t N_divider, uint8_t cpg_value, uint32_t den_value, uint8_t frac_order, uint8_t frac_dither)
{
    lmx2492_status_t statue; 

    // set PLL_R value
    statue = PLL_R_counter_divider_setting(R_divider);
    // set Fractional Modulator order
	statue = fractional_modulator_order_setting(frac_order);
	// set Fractional Modulator order
	statue = fractional_modulator_dither_setting(frac_dither);
    // set PLL_N value
    statue = PLL_N_counter_divider_setting(N_divider);
    // set charge pump gain value
    statue = charge_pump_gain_setting(cpg_value);
    // PLL_DEN 2^24
    statue = fractional_denominator_value_setting(den_value);

    return statue;
}


/* Single frequency point waveform configuration function */
lmx2492_status_t single_frequency_waveform_parameter(ramp_configuration_parameter_t *ramp_para, wave_fre_point_t fre)
{
    lmx2492_status_t status;

    reset_ramp_register_value();
    status = waveform_configuration(Ramp0Index, ramp_para);
    if (fre == FrequencyPoint24_15GHz)
    {
    	fractional_numerator_value_setting(SINGLE_FRAC_24_15G_NUM_VALUE);
    }
    else if(fre == FrequencyPoint24_25GHz)
    {
    	fractional_numerator_value_setting(SINGLE_FRAC_24_25G_NUM_VALUE);
    }
    else
    {
    	fractional_numerator_value_setting(SINGLE_FRAC_24G_NUM_VALUE);
    }

    LMX2492_write_all_register();
    usleep(2000);
    LMX2492_write_all_register();
    usleep(2000);

    return status;

}

/* Target waveform configuration function, Use four ramps, and trigger A source is MOD pin */
lmx2492_status_t target_waveform_parameter(uint16_t ramp_cnt, ramp_configuration_parameter_t *ramp0_para, ramp_configuration_parameter_t *ramp1_para,
											ramp_configuration_parameter_t *ramp2_para)
{
    lmx2492_status_t status;

    lmx2492_ramp_trigger_source_index_t trig_source_idx = MODRisingEdgeIndex;
    lmx2492_mux_drive_pin_state_index_t mod_pin_state = InputIndex;
    lmx2492_mux_state_index_t mod_state = InputTrig1Index;

    status = fractional_numerator_value_setting(SINGLE_FRAC_24G_NUM_VALUE);
    status = waveform_configuration(Ramp0Index, ramp0_para);
    status = waveform_configuration(Ramp1Index, ramp1_para);
    status = waveform_configuration(Ramp2Index, ramp2_para);
//	status = waveform_configuration(Ramp3Index, ramp3_para);
    status = ramp_counter_value_setting(ramp_cnt);

    // MOD input, trigger1
    mod_drive_state_setting(mod_pin_state);
    mod_state_setting(mod_state);

    triggera_source_setting(trig_source_idx);

    LMX2492_write_all_register();
    usleep(2000);
    LMX2492_write_all_register();
    usleep(2000);

    return status;

}
