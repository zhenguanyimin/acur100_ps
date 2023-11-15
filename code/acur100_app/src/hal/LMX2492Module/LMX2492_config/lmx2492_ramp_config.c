/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    LMX2492_config.c
 * @author  X22012
 * @date    2022骞�06鏈�01鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�06鏈�01鏃�
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "xil_io.h"
#include "xparameters.h"
#include "lmx2492_ramp_config.h"
#include "../LMX2492_driver/lmx2492_communication.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
const uint8_t trigger_source_reg_arr[3] =   {RAMP_TRIGA_SOURCE_REGISTER, RAMP_TRIGB_SOURCE_REGISTER, RAMP_TRIGC_SOURCE_REGISTER};

const uint8_t trigger_source_mask_arr[3] =  {RAMP_TRIGA_SOURCE_REG_MASK, RAMP_TRIGB_SOURCE_REG_MASK, RAMP_TRIGC_SOURCE_REG_MASK};

const uint8_t trigger_source_shift_arr[3] = {RAMP_TRIGA_SOURCE_REG_SHIFT, RAMP_TRIGB_SOURCE_REG_SHIFT, RAMP_TRIGC_SOURCE_REG_SHIFT};

const uint8_t trigger_source_arr[16] = {
    NEVER_TRIGGER,               TRIG1_TERMINAL_RISING_EDGE, TRIG2_TERMINAL_RISING_EDGE, MOD_TERMINAL_RISING_EDGE,    DLD_RISING_EDGE,             
    CMP0_DETECTED,               RAMPX_CPG_RISING_EDGE,      RAMPX_FLAG0_RISING_EDGE,    ALWAYS_TRIGGERED,            TRIG1_TERMINAL_FALLING_EDGE, 
    TRIG2_TERMINAL_FALLING_EDGE, MOD_TERMINAL_FALLING_EDGE,  DLD_FALLING_EDGE,           CMP1_DETECTED,               RAMPX_CPG_FALLING_EDGE, 
    RAMPX_FLAG0_FALLING_EDGE};

const uint8_t ramp_config_register_arr[5][5] = {
    {RAMP_CMP0_LOWEST_REGISTER,  RAMP_CMP1_LOWEST_REGISTER,  FSK_DEV_LOWEST_REGISTER,  RAMP_LIMIT_LOW_LOWEST_REGISTER,  RAMP_LIMIT_HIGH_LOWEST_REGISTER},
    {RAMP_CMP0_SECOND_REGISTER,  RAMP_CMP1_SECOND_REGISTER,  FSK_DEV_SECOND_REGISTER,  RAMP_LIMIT_LOW_SECOND_REGISTER,  RAMP_LIMIT_HIGH_SECOND_REGISTER},
    {RAMP_CMP0_THIRD_REGISTER,   RAMP_CMP1_THIRD_REGISTER,   FSK_DEV_THIRD_REGISTER,   RAMP_LIMIT_LOW_THIRD_REGISTER,   RAMP_LIMIT_HIGH_THIRD_REGISTER},
    {RAMP_CMP0_FOURTH_REGISTER,  RAMP_CMP1_FOURTH_REGISTER,  FSK_DEV_FOURTH_REGISTER,  RAMP_LIMIT_LOW_FOURTH_REGISTER,  RAMP_LIMIT_HIGH_FOURTH_REGISTER},
    {RAMP_CMP0_HIGHEST_REGISTER, RAMP_CMP1_HIGHEST_REGISTER, FSK_DEV_HIGHEST_REGISTER, RAMP_LIMIT_LOW_HIGHEST_REGISTER, RAMP_LIMIT_HIGH_HIGHEST_REGISTER}};

const uint8_t ramp_config_mask_arr[2][5] = {
    {RAMP_CMP0_COMMON_REG_MASK,  RAMP_CMP1_COMMON_REG_MASK,  FSK_DEV_COMMON_REG_MASK,  RAMP_LIMIT_LOW_COMMON_REG_MASK,  RAMP_LIMIT_HIGH_COMMON_REG_MASK}, 
    {RAMP_CMP0_HIGHEST_REG_MASK, RAMP_CMP1_HIGHEST_REG_MASK, FSK_DEV_HIGHEST_REG_MASK, RAMP_LIMIT_LOW_HIGHEST_REG_MASK, RAMP_LIMIT_HIGH_HIGHEST_REG_MASK}};

const uint8_t ramp_config_shift_arr[2][5] = {
    {RAMP_CMP0_COMMON_REG_SHIFT, RAMP_CMP1_COMMON_REG_SHIFT,  FSK_DEV_COMMON_REG_SHIFT,  RAMP_LIMIT_LOW_COMMON_REG_SHIFT,  RAMP_LIMIT_HIGH_COMMON_REG_SHIFT},
    {RAMP_CMP0_COMMON_REG_SHIFT, RAMP_CMP1_HIGHEST_REG_SHIFT, FSK_DEV_HIGHEST_REG_SHIFT, RAMP_LIMIT_LOW_HIGHEST_REG_SHIFT, RAMP_LIMIT_HIGH_HIGHEST_REG_SHIFT}};
/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
/** @brief TRIGA, TRIGB, and TRIGC sources setting function.
 *  @param [in]  trig_idx         Trigger index, the value ranges from 0-2, which corresponds to TRIGA, TRIGB and TRIGC.
 *  @param [in]  trig_source_idx  Trigger source index, the value ranges from 0-15.
 *  @return @ref lmx2492_status_t "Status return code." */
static lmx2492_status_t trigger_source_setting(lmx2492_ramp_trigger_index_t trig_idx, lmx2492_ramp_trigger_source_index_t trig_source_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t reg = trigger_source_reg_arr[trig_idx];
    uint8_t mask = trigger_source_mask_arr[trig_idx];
    uint8_t shift = trigger_source_shift_arr[trig_idx];
    uint8_t val = trigger_source_arr[trig_source_idx];
    
    status = LMX2492_register_setting(reg, val, mask, shift);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Common function for ramp setting. */
static lmx2492_status_t ramp_config_setting(lmx2492_ramp_config_parameter_index_t idx, uint64_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t lowReg   = ramp_config_register_arr[0][idx];
    uint8_t secReg   = ramp_config_register_arr[1][idx];
    uint8_t thiReg   = ramp_config_register_arr[2][idx];
    uint8_t fouReg   = ramp_config_register_arr[3][idx];
    uint8_t highReg  = ramp_config_register_arr[4][idx];
    uint8_t comMask  = ramp_config_mask_arr[0][idx];
    uint8_t highMask = ramp_config_mask_arr[1][idx];
    uint8_t highVal  = (val >> 32) & highMask;
    uint8_t fouVal   = (val >> 24) & comMask;
    uint8_t thiVal   = (val >> 16) & comMask;
    uint8_t secVal   = (val >> 8)  & comMask;
    uint8_t lowVal   = val         & comMask;
    uint8_t comShift = ramp_config_shift_arr[0][idx];
    uint8_t highShift = ramp_config_shift_arr[1][idx];

    status = LMX2492_register_setting(lowReg,  lowVal,  comMask,  comShift);
    status = LMX2492_register_setting(secReg,  secVal,  comMask,  comShift);
    status = LMX2492_register_setting(thiReg,  thiVal,  comMask,  comShift);
    status = LMX2492_register_setting(fouReg,  fouVal,  comMask,  comShift);
    status = LMX2492_register_setting(highReg, highVal, highMask, highShift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}
/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* Enable/Disable the RAMP function */
lmx2492_status_t ramp_functions_disable(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_ENABLE_REGISTER;
    uint8_t val   = RAMP_DISABLE;
    uint8_t mask  = RAMP_ENABLE_REG_MASK;
    uint8_t shift = RAMP_ENABLE_REG_SHIFT;

	uint8_t tmp = lmx2492_register_data_array[reg];
    uint8_t writeData = (val << shift) & mask;
    uint32_t data = 0x3A00;

//    status = LMX2492_register_setting(reg, val, mask, shift);

    tmp  = (tmp & (~mask)) | writeData;
    lmx2492_register_data_array[reg] = tmp;
    data = data | (tmp & 0xFF);

    Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44 , data);


    return status;

}

lmx2492_status_t ramp_functions_enable(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_ENABLE_REGISTER;
    uint8_t val   = RAMP_ENABLE;
    uint8_t mask  = RAMP_ENABLE_REG_MASK;
    uint8_t shift = RAMP_ENABLE_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Set RAMP clock source */
lmx2492_status_t set_ramp_clock_source_phase_detector(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_CLOCK_REGISTER;
    uint8_t val   = RAMP_CLOCK_SOURCE_PD;
    uint8_t mask  = RAMP_CLOCK_REG_MASK;
    uint8_t shift = RAMP_CLOCK_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t set_ramp_clock_source_mod_terminal(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_CLOCK_REGISTER;
    uint8_t val   = RAMP_CLOCK_SOURCE_MOD;
    uint8_t mask  = RAMP_CLOCK_REG_MASK;
    uint8_t shift = RAMP_CLOCK_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Phase modulation enable */
lmx2492_status_t set_ramp_phase_modulation_frequency_modulation(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_PHASE_MODULATION_ENABLE_REGISTER;
    uint8_t val   = RAMP_FREQUENCY_MODULATION;
    uint8_t mask  = RAMP_PHASE_MODULATION_ENABLE_REG_MASK;
    uint8_t shift = RAMP_PHASE_MODULATION_ENABLE_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t set_ramp_phase_modulation_phase_modulation(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_PHASE_MODULATION_ENABLE_REGISTER;
    uint8_t val   = RAMP_PHASE_MODULATION;
    uint8_t mask  = RAMP_PHASE_MODULATION_ENABLE_REG_MASK;
    uint8_t shift = RAMP_PHASE_MODULATION_ENABLE_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


// TRIGA source setting
lmx2492_status_t triggera_source_setting(lmx2492_ramp_trigger_source_index_t source_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    lmx2492_ramp_trigger_index_t trigger_index = TrigASourceIndex;
    
    status = trigger_source_setting(trigger_index, source_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// TRIGB source setting
lmx2492_status_t triggerb_source_setting(lmx2492_ramp_trigger_source_index_t source_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    lmx2492_ramp_trigger_index_t trigger_index = TrigBSourceIndex;
    
    status = trigger_source_setting(trigger_index, source_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// TRIGC source setting
lmx2492_status_t triggerc_source_setting(lmx2492_ramp_trigger_source_index_t source_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    lmx2492_ramp_trigger_index_t trigger_index = TrigCSourceIndex;
    
    status = trigger_source_setting(trigger_index, source_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


/* Ramp Comparator 0 value setting. */
lmx2492_status_t ramp_comparator0_value_setting(uint64_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    lmx2492_ramp_config_parameter_index_t para_idx = RampComparator0Index;

    status = ramp_config_setting(para_idx, val);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// ramp active of Comparator 0 control field
lmx2492_status_t ramp_comparator0_enable_setting(uint8_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_CMP0_ENABLE_REGISTER;
    uint8_t mask  = RAMP_CMP0_ENABLE_REG_MASK;
    uint8_t shift = RAMP_CMP0_ENABLE_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Ramp Comparator 1 value setting. */
lmx2492_status_t ramp_comparator1_value_setting(uint64_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    lmx2492_ramp_config_parameter_index_t para_idx = RampComparator1Index;

    status = ramp_config_setting(para_idx, val);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// ramp active of Comparator 1 control field
lmx2492_status_t ramp_comparator1_enable_setting(uint8_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_CMP1_ENABLE_REGISTER;
    uint8_t mask  = RAMP_CMP1_ENABLE_REG_MASK;
    uint8_t shift = RAMP_CMP1_ENABLE_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// Deviation trigger source setting.
lmx2492_status_t fsk_deviation_trigger_setting(lmx2492_fsk_trigger_source_index_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = FSK_TRIG_SOURCE_REGISTER;
    uint8_t mask  = FSK_TRIG_SOURCE_REG_MASK;
    uint8_t shift = FSK_TRIG_SOURCE_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* FSK deviation value setting. */
lmx2492_status_t fsk_deviation_value_setting(uint64_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    lmx2492_ramp_config_parameter_index_t para_idx = FSKDeviatonIndex;

    status = ramp_config_setting(para_idx, val);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Ramp limit low value setting. */
lmx2492_status_t ramp_limit_low_setting(uint64_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    lmx2492_ramp_config_parameter_index_t para_idx = RampLimitLow;

    status = ramp_config_setting(para_idx, val);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Ramp limit high value setting. */
lmx2492_status_t ramp_limit_high_setting(uint64_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    lmx2492_ramp_config_parameter_index_t para_idx = RampLimitHigh;

    status = ramp_config_setting(para_idx, val);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Ramp counter value setting. */
lmx2492_status_t ramp_counter_value_setting(uint16_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t highReg  = RAMP_COUNT_HIGH_REGISTER;
    uint8_t lowReg   = RAMP_COUNT_LOW_REGISTER;
    uint8_t highmask = RAMP_COUNT_HIGH_REG_MASK;
    uint8_t lowmask  = RAMP_COUNT_LOW_REG_MASK;
    uint8_t highVal  = (val >> 8) & highmask;
    uint8_t lowVal   = val        & lowmask;
    uint8_t shift    = RAMP_COUNT_REG_SHIFT;

    status = LMX2492_register_setting(lowReg,  lowVal,  lowmask,  shift);
    status = LMX2492_register_setting(highReg, highVal, highmask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Automatically clear RAMP_EN setting. */
lmx2492_status_t auto_clear_ramp_en_disable(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_AUTO_REGISTER;
    uint8_t val   = RAMP_AUTO_DISABLE;
    uint8_t mask  = RAMP_AUTO_REG_MASK;
    uint8_t shift = RAMP_AUTO_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t auto_clear_ramp_en_enable(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_AUTO_REGISTER;
    uint8_t val   = RAMP_AUTO_ENABLE;
    uint8_t mask  = RAMP_AUTO_REG_MASK;
    uint8_t shift = RAMP_AUTO_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Increment Trigger source for RAMP Counter. */
lmx2492_status_t counter_increment_trigger_source_setting(lmx2492_increment_trigger_source_index_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = RAMP_TRIG_INC_REGISTER;
    uint8_t mask  = RAMP_TRIG_INC_REG_MASK;
    uint8_t shift = RAMP_TRIG_INC_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}
