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
#include "lmx2492_individual_ramp_config.h"
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
const uint8_t rampx_increment_register_arr[8][4] = 
    {{RAMP0_INCREMENT_LOWEST_REGISTER, RAMP0_INCREMENT_SECOND_REGISTER, RAMP0_INCREMENT_THIRD_REGISTER, RAMP0_INCREMENT_HIGHEST_REGISTER},
     {RAMP1_INCREMENT_LOWEST_REGISTER, RAMP1_INCREMENT_SECOND_REGISTER, RAMP1_INCREMENT_THIRD_REGISTER, RAMP1_INCREMENT_HIGHEST_REGISTER},
     {RAMP2_INCREMENT_LOWEST_REGISTER, RAMP2_INCREMENT_SECOND_REGISTER, RAMP2_INCREMENT_THIRD_REGISTER, RAMP2_INCREMENT_HIGHEST_REGISTER},
     {RAMP3_INCREMENT_LOWEST_REGISTER, RAMP3_INCREMENT_SECOND_REGISTER, RAMP3_INCREMENT_THIRD_REGISTER, RAMP3_INCREMENT_HIGHEST_REGISTER},
     {RAMP4_INCREMENT_LOWEST_REGISTER, RAMP4_INCREMENT_SECOND_REGISTER, RAMP4_INCREMENT_THIRD_REGISTER, RAMP4_INCREMENT_HIGHEST_REGISTER},
     {RAMP5_INCREMENT_LOWEST_REGISTER, RAMP5_INCREMENT_SECOND_REGISTER, RAMP5_INCREMENT_THIRD_REGISTER, RAMP5_INCREMENT_HIGHEST_REGISTER},
     {RAMP6_INCREMENT_LOWEST_REGISTER, RAMP6_INCREMENT_SECOND_REGISTER, RAMP6_INCREMENT_THIRD_REGISTER, RAMP6_INCREMENT_HIGHEST_REGISTER},
     {RAMP7_INCREMENT_LOWEST_REGISTER, RAMP7_INCREMENT_SECOND_REGISTER, RAMP7_INCREMENT_THIRD_REGISTER, RAMP7_INCREMENT_HIGHEST_REGISTER}};

const uint8_t rampx_fastlock_register_arr[8] = {RAMP0_FASTLOCK_REGISTER, RAMP1_FASTLOCK_REGISTER, RAMP2_FASTLOCK_REGISTER, RAMP3_FASTLOCK_REGISTER, 
                                                RAMP4_FASTLOCK_REGISTER, RAMP5_FASTLOCK_REGISTER, RAMP6_FASTLOCK_REGISTER, RAMP7_FASTLOCK_REGISTER};

const uint8_t rampx_delay_register_arr[8] = {RAMP0_DELAY_REGISTER, RAMP1_DELAY_REGISTER, RAMP2_DELAY_REGISTER, RAMP3_DELAY_REGISTER,
                                             RAMP4_DELAY_REGISTER, RAMP5_DELAY_REGISTER, RAMP6_DELAY_REGISTER, RAMP7_DELAY_REGISTER};

const uint8_t rampx_length_register_arr[8][2] = {{RAMP0_LENGTH_LOW_REGISTER, RAMP0_LENGTH_HIGH_REGISTER},
                                                 {RAMP1_LENGTH_LOW_REGISTER, RAMP1_LENGTH_HIGH_REGISTER},
                                                 {RAMP2_LENGTH_LOW_REGISTER, RAMP2_LENGTH_HIGH_REGISTER},
                                                 {RAMP3_LENGTH_LOW_REGISTER, RAMP3_LENGTH_HIGH_REGISTER},
                                                 {RAMP4_LENGTH_LOW_REGISTER, RAMP4_LENGTH_HIGH_REGISTER},
                                                 {RAMP5_LENGTH_LOW_REGISTER, RAMP5_LENGTH_HIGH_REGISTER},
                                                 {RAMP6_LENGTH_LOW_REGISTER, RAMP6_LENGTH_HIGH_REGISTER},
                                                 {RAMP7_LENGTH_LOW_REGISTER, RAMP7_LENGTH_HIGH_REGISTER}};

const uint8_t rampx_flag_register_arr[8] = {RAMP0_FLAG_REGISTER, RAMP1_FLAG_REGISTER, RAMP2_FLAG_REGISTER, RAMP3_FLAG_REGISTER,
                                            RAMP4_FLAG_REGISTER, RAMP5_FLAG_REGISTER, RAMP6_FLAG_REGISTER, RAMP7_FLAG_REGISTER};

const uint8_t rampx_reset_register_arr[8] = {RAMP0_RESET_REGISTER, RAMP1_RESET_REGISTER, RAMP2_RESET_REGISTER, RAMP3_RESET_REGISTER,
                                             RAMP4_RESET_REGISTER, RAMP5_RESET_REGISTER, RAMP6_RESET_REGISTER, RAMP7_RESET_REGISTER};

const uint8_t rampx_next_trigger_register_arr[8] = {RAMP0_NEXT_TRIGGER_REGISTER, RAMP1_NEXT_TRIGGER_REGISTER, RAMP2_NEXT_TRIGGER_REGISTER, RAMP3_NEXT_TRIGGER_REGISTER,
                                                    RAMP4_NEXT_TRIGGER_REGISTER, RAMP5_NEXT_TRIGGER_REGISTER, RAMP6_NEXT_TRIGGER_REGISTER, RAMP7_NEXT_TRIGGER_REGISTER};    

const uint8_t rampx_next_register_arr[8] = {RAMP0_NEXT_REGISTER, RAMP1_NEXT_REGISTER, RAMP2_NEXT_REGISTER, RAMP3_NEXT_REGISTER,
                                            RAMP4_NEXT_REGISTER, RAMP5_NEXT_REGISTER, RAMP6_NEXT_REGISTER, RAMP7_NEXT_REGISTER};

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
/* Common function for individual ramp increment value setting. */
static lmx2492_status_t rampx_increment_setting(lmx2492_ramp_index_t idx, uint32_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t lowReg   = rampx_increment_register_arr[idx][0];
    uint8_t secReg   = rampx_increment_register_arr[idx][1];
    uint8_t thiReg   = rampx_increment_register_arr[idx][2];
    uint8_t highReg  = rampx_increment_register_arr[idx][3];
    uint8_t comMask  = RAMPX_INCREMENT_COMMON_REG_MASK;
    uint8_t highMask = RAMPX_INCREMENT_HIGHEST_REG_MASK;
    uint8_t highVal  = (val >> 24) & highMask;
    uint8_t thiVal   = (val >> 16) & comMask;
    uint8_t secVal   = (val >> 8)  & comMask;
    uint8_t lowVal   = val         & comMask;
    uint8_t shift = RAMPX_INCREMENT_COMMON_REG_SHIFT;

    status = LMX2492_register_setting(lowReg,  lowVal,  comMask,  shift);
    status = LMX2492_register_setting(secReg,  secVal,  comMask,  shift);
    status = LMX2492_register_setting(thiReg,  thiVal,  comMask,  shift);
    status = LMX2492_register_setting(highReg, highVal, highMask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* Ramp 0-7 increment value setting. */
lmx2492_status_t rampx_increment_value_setting(lmx2492_ramp_index_t ramp_idx, uint32_t value)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    status = rampx_increment_setting(ramp_idx, value);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Ramp 0-7 fast lock enable setting. */
lmx2492_status_t rampx_fastlock_disable(lmx2492_ramp_index_t ramp_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_fastlock_register_arr[ramp_idx];
    uint8_t val   = RAMPX_FASTLOCK_DISABLE;
    uint8_t mask  = RAMPX_FASTLOCK_REG_MASK;
    uint8_t shift = RAMPX_FASTLOCK_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t rampx_fastlock_enable(lmx2492_ramp_index_t ramp_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_fastlock_register_arr[ramp_idx];
    uint8_t val   = RAMPX_FASTLOCK_ENABLE;
    uint8_t mask  = RAMPX_FASTLOCK_REG_MASK;
    uint8_t shift = RAMPX_FASTLOCK_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


/* rampx increment cycle setting. */
lmx2492_status_t rampx_increment_cycle_1PFD(lmx2492_ramp_index_t ramp_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_delay_register_arr[ramp_idx];
    uint8_t val   = RAMPX_DELAY_CLOCK_SINGLE;
    uint8_t mask  = RAMPX_DELAY_REG_MASK;
    uint8_t shift = RAMPX_DELAY_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t rampx_increment_cycle_2PFD(lmx2492_ramp_index_t ramp_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_delay_register_arr[ramp_idx];
    uint8_t val   = RAMPX_DELAY_CLOCK_DOUBLE;
    uint8_t mask  = RAMPX_DELAY_REG_MASK;
    uint8_t shift = RAMPX_DELAY_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Number of PFD clocks to continue to increment ramp setting. */
lmx2492_status_t rampx_length_setting(lmx2492_ramp_index_t ramp_idx, uint16_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t lowReg   = rampx_length_register_arr[ramp_idx][0];
    uint8_t highReg  = rampx_length_register_arr[ramp_idx][1];
    uint8_t mask  = RAMPX_LENGTH_REG_MASK;
    uint8_t highVal  = (val >> 8) & mask;
    uint8_t lowVal   = val        & mask;
    uint8_t shift = RAMPX_LENGTH_REG_SHIFT;

    status = LMX2492_register_setting(lowReg,  lowVal,  mask, shift);
    status = LMX2492_register_setting(highReg, highVal, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/*  General purpose flags sent out of ramp setting. */
lmx2492_status_t rampx_flag_setting(lmx2492_ramp_index_t ramp_idx, lmx2492_ramp_flag_set_index_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_flag_register_arr[ramp_idx];
    uint8_t mask  = RAMPX_FLAG_REG_MASK;
    uint8_t shift = RAMPX_FLAG_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* rampx reset setting. */
lmx2492_status_t rampx_reset_disable(lmx2492_ramp_index_t ramp_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_reset_register_arr[ramp_idx];
    uint8_t val   = RAMPX_RESERT_DISABLE;
    uint8_t mask  = RAMPX_RESET_REG_MASK;
    uint8_t shift = RAMPX_RESET_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t rampx_reset_enable(lmx2492_ramp_index_t ramp_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_reset_register_arr[ramp_idx];
    uint8_t val   = RAMPX_RESERT_ENABLE;
    uint8_t mask  = RAMPX_RESET_REG_MASK;
    uint8_t shift = RAMPX_RESET_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* rampx next trigger setting. */
lmx2492_status_t rampx_next_trigger_setting(lmx2492_ramp_index_t ramp_idx, lmx2492_ramp_next_trigger_index_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_next_trigger_register_arr[ramp_idx];
    uint8_t mask  = RAMPX_NEXT_TRIGGER_REG_MASK;
    uint8_t shift = RAMPX_NEXT_TRIGGER_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* rampx next setting. */
lmx2492_status_t rampx_next_setting(lmx2492_ramp_index_t ramp_idx, lmx2492_ramp_index_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = rampx_next_register_arr[ramp_idx];
    uint8_t mask  = RAMPX_NEXT_REG_MASK;
    uint8_t shift = RAMPX_NEXT_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

