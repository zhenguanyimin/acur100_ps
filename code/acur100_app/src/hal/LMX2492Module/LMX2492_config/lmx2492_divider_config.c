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
#include "lmx2492_divider_config.h"
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
/* LMX2492 Feedback N counter Divide value setting. */
lmx2492_status_t PLL_N_counter_divider_setting(uint32_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t highReg  = PLL_N_COUNTER_HIGI_REGISTER;
    uint8_t midReg   = PLL_N_COUNTER_MID_REGISTER;
    uint8_t lowReg   = PLL_N_COUNTER_LOW_REGISTER;
    uint8_t highMask = PLL_N_COUNTER_HIGI_REG_MASK;
    uint8_t comMask  = PLL_N_COUNTER_COMMON_REG_MASK;
//    uint8_t highVal  = (val >> 16) & highMask;
    uint8_t highVal  = 0X2C;
    uint8_t midVal   = (val >> 8)  & comMask;
    uint8_t lowVal   = val         & comMask;
    uint8_t shift    = PLL_N_COUNTER_REG_SHIFT;

    status = LMX2492_register_setting(lowReg, lowVal, comMask, shift);
    status = LMX2492_register_setting(midReg, midVal, comMask, shift);
    status = LMX2492_register_setting(highReg, highVal, highMask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Dither used by the fractional modulator setting. */
lmx2492_status_t fractional_modulator_dither_setting(lmx2492_fractional_dither_value_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = FRACTIONAL_DITHER_REGISTER;
    uint8_t mask  = FRACTIONAL_DITHER_REG_MASK;
    uint8_t shift = FRACTIONAL_DITHER_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


/* Fractional Modulator order setting. */
lmx2492_status_t fractional_modulator_order_setting(lmx2492_fractional_modulator_order_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = FRACTIONAL_ORDER_REGISTER;
    uint8_t mask  = FRACTIONAL_ORDER_REG_MASK;
    uint8_t shift = FRACTIONAL_ORDER_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


/* Fractional numerator value setting. */
lmx2492_status_t fractional_numerator_value_setting(uint32_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t highReg = FRACTIONAL_NUMERATOR_HIGI_REGISTER;
    uint8_t midReg  = FRACTIONAL_NUMERATOR_MID_REGISTER;
    uint8_t lowReg  = FRACTIONAL_NUMERATOR_LOW_REGISTER;
    uint8_t mask    = FRACTIONAL_NUMERATOR_REG_MASK;
    uint8_t highVal = (val >> 16) & mask;
    uint8_t midVal  = (val >> 8)  & mask;
    uint8_t lowVal  = val         & mask;
    uint8_t shift   = FRACTIONAL_NUMERATOR_REG_SHIFT;

    status = LMX2492_register_setting(lowReg, lowVal, mask, shift);
    status = LMX2492_register_setting(midReg, midVal, mask, shift);
    status = LMX2492_register_setting(highReg, highVal, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Fractional denominator value setting. */
lmx2492_status_t fractional_denominator_value_setting(uint32_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t highReg = FRACTIONAL_DENOMINATOR_HIGI_REGISTER;
    uint8_t midReg  = FRACTIONAL_DENOMINATOR_MID_REGISTER;
    uint8_t lowReg  = FRACTIONAL_DENOMINATOR_LOW_REGISTER;
    uint8_t mask    = FRACTIONAL_DENOMINATOR_REG_MASK;
    uint8_t highVal = (val >> 16) & mask;
    uint8_t midVal  = (val >> 8)  & mask;
    uint8_t lowVal  = val         & mask;
    uint8_t shift   = FRACTIONAL_DENOMINATOR_REG_SHIFT;

    status = LMX2492_register_setting(lowReg, lowVal, mask, shift);
    status = LMX2492_register_setting(midReg, midVal, mask, shift);
    status = LMX2492_register_setting(highReg, highVal, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Reference Divider value setting. */
lmx2492_status_t PLL_R_counter_divider_setting(uint16_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t highReg = PLL_R_COUNTER_HIGI_REGISTER;
    uint8_t lowReg  = PLL_R_COUNTER_LOW_REGISTER;
    uint8_t mask    = PLL_R_COUNTER_REG_MASK;
    uint8_t highVal = (val >> 8) & mask;
    uint8_t lowVal  = val         & mask;
    uint8_t shift   = PLL_R_COUNTER_REG_SHIFT;

    status = LMX2492_register_setting(lowReg, lowVal, mask, shift);
    status = LMX2492_register_setting(highReg, highVal, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* OSC doubler state setting. */
lmx2492_status_t OSC_doubler_disable(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = OSC_2X_REGISTER;
    uint8_t val   = OSC_DOUBLER_DISABLE;
    uint8_t mask  = OSC_2X_REG_MASK;
    uint8_t shift = OSC_2X_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t OSC_doubler_enable(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = OSC_2X_REGISTER;
    uint8_t val   = OSC_DOUBLER_ENABLE;
    uint8_t mask  = OSC_2X_REG_MASK;
    uint8_t shift = OSC_2X_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


/* The Differential R counter state setting. */
lmx2492_status_t R_counter_single_ended(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = PLL_R_DIFF_REGISTER;
    uint8_t val   = R_DIVIDER_SINGLE_ENDED;
    uint8_t mask  = PLL_R_DIFF_REG_MASK;
    uint8_t shift = PLL_R_DIFF_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t R_counter_differential(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = PLL_R_DIFF_REGISTER;
    uint8_t val   = R_DIVIDER_DIFFERENTIAL;
    uint8_t mask  = PLL_R_DIFF_REG_MASK;
    uint8_t shift = PLL_R_DIFF_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Charge pump minimum pulse width setting. */
lmx2492_status_t set_charge_pump_pulse_width_min(lmx2492_minimum_pulse_width_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = PFD_DLY_REGISTER;
    uint8_t mask  = PFD_DLY_REG_MASK;
    uint8_t shift = PFD_DLY_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Charge pump gain setting. */
lmx2492_status_t charge_pump_gain_setting(lmx2492_charge_pump_gain_value_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = CHARGE_PUMP_GAIN_REGISTER;
    uint8_t mask  = CHARGE_PUMP_GAIN_REG_MASK;
    uint8_t shift = CHARGE_PUMP_GAIN_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Charge pump polarity setting. */
lmx2492_status_t set_charge_pump_polarity_negative(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = CHARGE_PUMP_POLARITY_REGISTER;
    uint8_t val   = CHARGE_PUMP_POLARITY_NEGATIVE;
    uint8_t mask  = CHARGE_PUMP_POLARITY_REG_MASK;
    uint8_t shift = CHARGE_PUMP_POLARITY_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

lmx2492_status_t set_charge_pump_polarity_positive(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = CHARGE_PUMP_POLARITY_REGISTER;
    uint8_t val   = CHARGE_PUMP_POLARITY_POSITIVE;
    uint8_t mask  = CHARGE_PUMP_POLARITY_REG_MASK;
    uint8_t shift = CHARGE_PUMP_POLARITY_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}
