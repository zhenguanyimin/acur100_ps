/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    driver_CHC2442.c
 * @author  X22012
 * @date    2022骞�05鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�05鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "chc2442_communication.h"
#include "chc2442_spi_hal.h"
#include "chc2442_assert.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CHC2442_COMM_FRAME_SIZE     0x03U  		/* Length of the communication frame */
#define REGISTER_DEFAULT_VAL 		0XC81302    /* default value */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
// Saves the value of the last write data, which is used for sending data when the register is read
static uint32_t read_instruct_data = REGISTER_DEFAULT_VAL;

const uint32_t register_mask[14] = {
    SPI_REG_VGA_GAIN_MASK,    SPI_REG_IF_LPF_MASK,    SPI_REG_MPA_POWER_MASK,  SPI_REG_MPA_MUTE_MASK, SPI_REG_LO_AMPS_MASK,
    SPI_REG_IF_HPF_MASK,      SPI_REG_PRESCALER_MASK, SPI_REG_PILOT_TONE_MASK, SPI_REG_MUX_MASK,      SPI_REG_DC_POWER_MODE_MASK, 
    SPI_REG_IF_AMP_GAIN_MASK, SPI_REG_MHZ_MODE_MASK,  SPI_REG_MIXER_GAIN_MASK, SPI_REG_IC_ENABLE_MASK};

const uint32_t register_shift[14] = { 
    SPI_REG_VGA_GAIN_SHIFT,    SPI_REG_IF_LPF_SHIFT,    SPI_REG_MPA_POWER_SHIFT,  SPI_REG_MPA_MUTE_SHIFT, SPI_REG_LO_AMPS_SHIFT,
    SPI_REG_IF_HPF_SHIFT,      SPI_REG_PRESCALER_SHIFT, SPI_REG_PILOT_TONE_SHIFT, SPI_REG_MUX_SHIFT,      SPI_REG_DC_POWER_MODE_SHIFT, 
    SPI_REG_IF_AMP_GAIN_SHIFT, SPI_REG_MHZ_MODE_SHIFT,  SPI_REG_MIXER_GAIN_SHIFT, SPI_REG_IC_ENABLE_SHIFT};

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
/* Creates a raw frame for SPI transfer. */
static void CHC2442_SPI_CreateSendFrame(uint32_t txData, uint8_t* txFrame)
{
    FS_ASSERT(txData != NULL);
    FS_ASSERT(txFrame != NULL);

    // create transmission frame.
	txFrame[2] = (uint8_t)((txData >> 16) & 0xFF);
	txFrame[1] = (uint8_t)((txData >> 8)  & 0xFF);
	txFrame[0] = (uint8_t)((txData)       & 0xFF);

}

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* Performs SPI transfer of the txData. Received frame is saved into rxData structure. */
chc2442_status_t CHC2442_read_register(uint32_t* rxData)
{

    FS_ASSERT(txData != NULL);
    FS_ASSERT(rxData != NULL);

    chc2442_status_t status = chc2442TransStatusOk;
    uint32_t txData = read_instruct_data;
    // uint8_t txFrame[CHC2442_COMM_FRAME_SIZE] = {0};
    uint8_t rxFrame[CHC2442_COMM_FRAME_SIZE] = {0};

    // CHC2442_SPI_CreateSendFrame(txData, txFrame);

    status = CHC2442_SPI_TransData(txData);
    status = CHC2442_SPI_TransData(txData);
    if (status != chc2442TransStatusOk)
    {
        return status;
    }

    if (rxFrame != NULL)
    {
        *rxData = (uint32_t)(rxFrame[2] << 16U | rxFrame[1] << 8U | rxFrame[0]);
    }

    return status;

}

/* Performs SPI transfer of the txData. Received frame is saved into rxData structure. */
chc2442_status_t CHC2442_write_register(uint32_t txData)
{

    FS_ASSERT(txData != NULL);
    FS_ASSERT(rxData != NULL);

    chc2442_status_t status = chc2442TransStatusOk;
    uint8_t txFrame[CHC2442_COMM_FRAME_SIZE] = {0};

    CHC2442_SPI_CreateSendFrame(txData, txFrame);

    read_instruct_data = txData & SPI_REGISTER_MASK;
    // Make sure to write data
    status = CHC2442_SPI_TransData(read_instruct_data);
    status = CHC2442_SPI_TransData(read_instruct_data);
    if (status != chc2442TransStatusOk)
    {
        return status;
    }

    return status;

}


/* CHC2442 register settings. */
chc2442_status_t CHC2442_register_setting(chc2442_para_idx_t index, uint8_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;

    uint32_t txData = 0;
    uint32_t rxData = 0;
    uint32_t tmp = 0;
    
    /* Readout register value */
    status = CHC2442_read_register(&rxData);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    /* Write to register after modifying corresponding bit */
    tmp  = rxData & (~register_mask[index]);
    tmp |= (value & register_mask[index]) << register_shift[index];

    txData = tmp;
    rxData = 0;

    status = CHC2442_write_register(txData);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    /* Read the register value again to determine whether the write was successful */
    rxData = 0;

    status = CHC2442_read_register(&rxData);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    if (rxData != txData) {
        return chc2442WriteRegisterError;
    }

    return status;
}
