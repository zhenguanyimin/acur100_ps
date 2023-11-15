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
#include "lmx2492_speed_up_config.h"
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

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* Cycle slip reduction value setting. */
lmx2492_status_t fastlock_cycle_slip_reduction_value_setting(lmx2492_cycle_slip_reduction_value_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = FASTLOCK_CYCLE_SLIP_REDUCTION_REGISTER;
    uint8_t mask  = FASTLOCK_CYCLE_SLIP_REDUCTION_REG_MASK;
    uint8_t shift = FASTLOCK_CYCLE_SLIP_REDUCTION_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Fastlock charge pump gain setting. */
lmx2492_status_t fastlock_charge_pump_gain_setting(lmx2492_charge_pump_gain_value_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = FASTLOCK_CHARGE_PUMP_GAIN_REGISTER;
    uint8_t mask  = FASTLOCK_CHARGE_PUMP_GAIN_REG_MASK;
    uint8_t shift = FASTLOCK_CHARGE_PUMP_GAIN_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Fast Lock Timer value setting. */
lmx2492_status_t fastlock_timer_value_setting(uint16_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t highReg   = FASTLOCK_TIMER_HIGH_REGISTER;
    uint8_t lowReg    = FASTLOCK_TIMER_LOW_REGISTER;
    uint8_t highMask  = FASTLOCK_TIMER_HIGH_REG_MASK;
    uint8_t lowMask   = FASTLOCK_TIMER_LOW_REG_MASK;
    uint8_t highVal   = (val >> 8) & highMask;
    uint8_t lowVal    = val        & lowMask;
    uint8_t highShift = FASTLOCK_TIMER_HIGH_REG_SHIFT;
    uint8_t lowShift  = FASTLOCK_TIMER_LOW_REG_SHIFT;

    status = LMX2492_register_setting(lowReg, lowVal, lowMask, lowShift);
    status = LMX2492_register_setting(highReg, highVal, highMask, highShift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

