/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    LMX2492_config.c
 * @author  X22012
 * @date    2022楠烇拷06閺堬拷01閺冿拷
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022楠烇拷06閺堬拷01閺冿拷
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_communication.h"
#include "lmx2492_mux_pin_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
const uint8_t mux_pin_register_arr[4] = {TRIG1_PIN_DRIVE_STATE_REGISTER, TRIG2_PIN_DRIVE_STATE_REGISTER, MOD_PIN_DRIVE_STATE_REGISTER, MUXOUT_PIN_DRIVE_STATE_REGISTER};

const uint8_t mux_pin_drive_state_arr[7] = {COMMON_STATE_TRI_STATE, COMMON_STATE_OPEN_DRAIN_OUTPUT, COMMON_STATE_PULLUP_PULLDOWN_OUTPUT, COMMON_STATE_GND,
                                            COMMON_STATE_INVERTED_OPEN_DRAIN_OUTPUT, COMMON_STATE_INVERTED_PULLUP_PULLDOWN_OUTPUT, COMMON_STATE_INPUT};

const uint8_t mux_pin_state_arr[33] = {
    COMMON_MUX_STATE_GND,                     COMMON_MUX_STATE_INPIT_TRIG1,            COMMON_MUX_STATE_INPIT_TRIG2,          COMMON_MUX_STATE_INPIT_MOD,
    COMMON_MUX_STATE_OUTPUT_TRIG1,            COMMON_MUX_STATE_OUTPUT_TRIG2,           COMMON_MUX_STATE_OUTPUT_MOD,           COMMON_MUX_STATE_OUTPUT_READ_BACK,
    COMMON_MUX_STATE_OUTPUT_CMP0,             COMMON_MUX_STATE_OUTPUT_CMP1,            COMMON_MUX_STATE_OUTPUT_LD,            COMMON_MUX_STATE_OUTPUT_DLD,
    COMMON_MUX_STATE_OUTPUT_CMPON_GOOD,       COMMON_MUX_STATE_OUTPUT_CMPON_TOO_HIGH,  COMMON_MUX_STATE_OUTPUT_CMPON_TOO_LOW, COMMON_MUX_STATE_OUTPUT_RAMP_LIMIT_EXCEEDED,
    COMMON_MUX_STATE_OUTPUT_R_DIVIDE_2,       COMMON_MUX_STATE_OUTPUT_R_DIVIDE_4,      COMMON_MUX_STATE_OUTPUT_N_DIVIDE_2,    COMMON_MUX_STATE_OUTPUT_N_DIVIDE_4,
    COMMON_MUX_STATE_OUTPUT_CMP0RAMP,         COMMON_MUX_STATE_OUTPUT_CMP1RAMP,        COMMON_MUX_STATE_OUTPUT_FASTLOCK,      COMMON_MUX_STATE_OUTPUT_CPG_FROM_RAMP,
    COMMON_MUX_STATE_OUTPUT_FLAG0_FROM_RAMP,  COMMON_MUX_STATE_OUTPUT_FLAG1_FROM_RAMP, COMMON_MUX_STATE_OUTPUT_TRIGA,         COMMON_MUX_STATE_OUTPUT_TRIGB,
    COMMON_MUX_STATE_OUTPUT_TRIGC,            COMMON_MUX_STATE_OUTPUT_R_DIVIDE,        COMMON_MUX_STATE_OUTPUT_CPUP,          COMMON_MUX_STATE_OUTPUT_CPDN,
    COMMON_MUX_STATE_OUTPUT_RAMP_CNT_FINISHED};
/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
/** @brief TRIG1, TRIG2, MOD, and MUXout terminal states setting function.
 *  @param [in]  pin_idx        Pins index, the value ranges from 0-3, which corresponds to TRIG1, TRIG2, MOD, and MUXout.
 *  @param [in]  state_idx      Pin drive state index, the value ranges from 0-7 except for 4, which corresponds to TRISTATE, Open Drain Output, 
 *                              Open Drain Output, GND, Inverted Open Drain Output, Inverted Pullup / Pulldown output, Input.
 *  @return @ref lmx2492_status_t "Status return code." */
static lmx2492_status_t mux_pins_state_setting(lmx2492_mux_pin_index_t pin_idx, lmx2492_mux_drive_pin_state_index_t state_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;
    uint8_t reg = mux_pin_register_arr[pin_idx];
    uint8_t mask = MUX_PIN_DRIVE_STATE_REG_MASK;
    uint8_t shift = MUX_PIN_DRIVE_STATE_REG_SHIFT;
    uint8_t val = mux_pin_drive_state_arr[state_idx];
    
    status = LMX2492_register_setting(reg, val, mask, shift);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


/** @brief TRIG1, TRIG2, MOD, and MUXout terminal states setting function.
 *  @param [in]  pin_idx            Pins index, the value ranges from 0-3, which corresponds to TRIG1, TRIG2, MOD, and MUXout.
 *  @param [in]  mux_state_idx      MUX state index, the value ranges from 0-38 except for 20闁靛棴鎷�21 and 24-27, which corresponds to TRISTATE, Open Drain Output,
 *                                  Open Drain Output, GND, Inverted Open Drain Output, Inverted Pullup / Pulldown output, Input.
 *  @return @ref lmx2492_status_t "Status return code." */
static lmx2492_status_t mux_state_setting(lmx2492_mux_pin_index_t pin_idx, lmx2492_mux_state_index_t state_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t reg = mux_pin_register_arr[pin_idx];
    uint8_t mask = MUX_DRIVE_STATE_REG_MASK;
    uint8_t shift = MUX_DRIVE_STATE_REG_SHIFT;
    uint8_t val = mux_pin_state_arr[state_idx];
    
    status = LMX2492_register_setting(reg, val, mask, shift);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
// TRIG1 pin drive state setting
lmx2492_status_t trig1_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = TRIG1PinIndex;
    
    status = mux_pins_state_setting(pin_index, drive_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// TRIG2 pin drive state setting
lmx2492_status_t trig2_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = TRIG2PinIndex;
    
    status = mux_pins_state_setting(pin_index, drive_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// MOD pin drive state setting
lmx2492_status_t mod_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = MODPinIndex;
    
    status = mux_pins_state_setting(pin_index, drive_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// MUXout pin drive state setting
lmx2492_status_t muxout_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = MUXoutPinIndex;
    
    status = mux_pins_state_setting(pin_index, drive_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}


// TRIG1 state setting
lmx2492_status_t trig1_state_setting(lmx2492_mux_state_index_t state_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = TRIG1PinIndex;
    
    status = mux_state_setting(pin_index, state_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// TRIG2 state setting
lmx2492_status_t trig2_state_setting(lmx2492_mux_state_index_t state_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = TRIG2PinIndex;
    
    status = mux_state_setting(pin_index, state_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// MOD state setting
lmx2492_status_t mod_state_setting(lmx2492_mux_state_index_t state_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = MODPinIndex;
    
    status = mux_state_setting(pin_index, state_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}

// MUXout state setting
lmx2492_status_t muxout_state_setting(lmx2492_mux_state_index_t state_idx)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

    uint8_t pin_index = MUXoutPinIndex;
    
    status = mux_state_setting(pin_index, state_idx);

    if (status != LMX2492TransStatusOk)
    {
        return LMX2492TransStatusError;
    }

    return status;

}
