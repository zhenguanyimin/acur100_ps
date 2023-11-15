/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    lmx2492_power_config.c
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
#include "lmx2492_power_config.h"
#include "../LMX2492_driver/lmx2492_communication.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Local Variables
 ******************************************************************************/
/* POWERDOWN and Reset control register */
static uint16_t reg  = POWERDOWN_RESET_REGISTER;

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* LMX2492 power status setting. */
lmx2492_status_t LMX2492_power_status_setting(lmx2492_power_state_value_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t mask  = POWERDOWN_REG_MASK;
    uint8_t shift = POWERDOWN_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;
}

/* Set chip status to POWERDOWN and ignore CE. */
lmx2492_status_t LMX2492_power_down_setting(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t val   = POWERDOWN_IGNORE_CE;
    uint8_t mask  = POWERDOWN_REG_MASK;
    uint8_t shift = POWERDOWN_REG_SHIFT;
    
    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;
}

/* Set chip status to POWERUP and ignore CE. */
lmx2492_status_t LMX2492_power_up_setting(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t val   = POWERUP_IGNORE_CE;
    uint8_t mask  = POWERDOWN_REG_MASK;
    uint8_t shift = POWERDOWN_REG_SHIFT;
    
    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;
}

/* Set chip status to controlled by CE terminal state. */
lmx2492_status_t LMX2492_power_CE_setting(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t val   = POWERUP_STATE_BY_CE;
    uint8_t mask  = POWERDOWN_REG_MASK;
    uint8_t shift = POWERDOWN_REG_SHIFT;
    
    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;
}

/* software reset. */
lmx2492_status_t LMX2492_software_reset(void)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t val   = REGISTER_RESET_STATUS;
    uint8_t mask  = SEWRST_REG_MASK;
    uint8_t shift = SEWRST_REG_SHIFT;
    
    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}
