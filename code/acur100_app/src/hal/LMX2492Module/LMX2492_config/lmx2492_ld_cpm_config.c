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
#include "lmx2492_ld_cpm_config.h"
#include "lmx2492_config_register.h"
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
/* Charge pump voltage low threshold value setting. */
lmx2492_status_t low_threshold_value_setting(lmx2492_charge_pump_voltage_threshold_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = CPM_THR_LOW_REGISTER;
    uint8_t mask  = CPM_THR_LOW_REG_MASK;
    uint8_t shift = CPM_THR_LOW_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* get Charge pump voltage Flag of CPM_THR_LOW. */
bool get_low_threshold_flag(void)
{
    uint8_t value = 0;

    uint8_t reg   = CPM_FLAGL_REGISTER;
    uint8_t mask  = CPM_FLAGL_REG_MASK;

    value = LMX2492_read_single_register(reg);

    return value & mask;

}

/* Charge pump voltage high threshold value setting. */
lmx2492_status_t high_threshold_value_setting(lmx2492_charge_pump_voltage_threshold_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = CPM_THR_HIGH_REGISTER;
    uint8_t mask  = CPM_THR_HIGH_REG_MASK;
    uint8_t shift = CPM_THR_HIGH_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* get Charge pump voltage Flag of CPM_THR_HIGH. */
bool get_high_threshold_flag(void)
{
    uint8_t value = 0;
    
    uint8_t reg   = CPM_FLAGH_REGISTER;
    uint8_t mask  = CPM_FLAGL_REG_MASK;

    value = LMX2492_read_single_register(reg);

    return value & mask;

}

/* Digital Lock Detect Filter amount setting. */
lmx2492_status_t digital_lock_detect_filter_amount_setting(uint8_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = DLD_PASS_CNT_REGISTER;
    uint8_t mask  = DLD_PASS_CNT_REG_MASK;
    uint8_t shift = DLD_PASS_CNT_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Digital Lock Detect error count setting. */
lmx2492_status_t digital_lock_detect_error_count_setting(uint8_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = DLD_ERR_CNT_REGISTER;
    uint8_t mask  = DLD_ERR_CNT_REG_MASK;
    uint8_t shift = DLD_ERR_CNT_REG_SHIFT;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/* Digital Lock detect edge window value setting. */
lmx2492_status_t digital_lock_detect_edge_value_setting(lmx2492_digital_lock_detect_edge_t val)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    
    uint8_t reg   = DLD_TOL_REG_MASK;
    uint8_t mask  = DLD_TOL_REG_SHIFT;
    uint8_t shift = DLD_TOL_VALUE_ZERO;

    status = LMX2492_register_setting(reg, val, mask, shift);
    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}
