/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492_driver
 * @file    lmx2492_commmunication.c
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
#include "lmx2492_communication.h"
#include "lmx2492_spi_hal.h"
#include "lmx2492_assert.h"
#include "xil_io.h"
#include "sleep.h"
#include "xparameters.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LMX2492_COMM_FRAME_SIZE     0x03U  		/* Length of the communication frame */
#define LMX2492_REGISTER_LENGTH     0XFFFFFF   /* Number of valid register */

/*******************************************************************************
 * Local Variables
 ******************************************************************************/
uint8_t lmx2492_register_data_array[142] = {
	0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 1, 3-15 reserved
	0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x0A, 0x32,
	0x00, 0x0F, 0x00, 0x41, 0x08, 0x10, 0x18, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 0X30-0X3F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 0X50-0X5F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// 0X70-0X7F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/
/* Creates a raw frame for SPI transfer. */
static void LMX2492_SPI_CreateSendFrame(lmx2492_data_frame_t* txData, uint8_t* txFrame)
{
    FS_ASSERT(txData != NULL);
    FS_ASSERT(txFrame != NULL);

    /* check if register is r or w */
    switch (txData->commandType)
    {
        case LMX2492RegRead:
            txData->regAddress |= REGISTER_RW_BIT_MASK;
            /* Creates read command. */
            txFrame[2] = 0x00;
            txFrame[1] = (uint8_t)(txData->regAddress);
            txFrame[0] = (uint8_t)(txData->regAddress >> 8);
            break;

        case LMX2492RegWrite:
            /* Creates write command - MSB first. */
            txFrame[2] = (uint8_t)(txData->writeData);
            txFrame[1] = (uint8_t)(txData->regAddress);
            txFrame[0] = (uint8_t)(txData->regAddress >> 8);
            break;
    }

}

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* Performs SPI transfer of the txData. Received frame is saved into rxData structure. */
lmx2492_status_t LMX2492_write_register(lmx2492_data_frame_t* txData)
{

    FS_ASSERT(txData != NULL);
    FS_ASSERT(rxData != NULL);

    lmx2492_status_t status = LMX2492TransStatusOk;
    uint8_t txFrame[LMX2492_COMM_FRAME_SIZE] = {0};

    LMX2492_SPI_CreateSendFrame(txData, txFrame);
    status = LMX2492_SPI_TransData(txFrame);
    if (status != LMX2492TransStatusOk)
    {
        return status;
    }

    return status;

}

void LMX2492_write_single_register(uint32_t data)
{
	Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44, data);

}

///* LMX2492 register reading. */
//lmx2492_status_t LMX2492_read_register(uint16_t regAddr, uint8_t *rxData, uint8_t mask)
//{
//    lmx2492_status_t status = LMX2492TransStatusOk;
//
//    uint8_t tmp = 0;
//    lmx2492_data_frame_t txData = {
//        .commandType = LMX2492RegRead,
//        .regAddress  = regAddr };
//
//	Xil_Out32(XPAR_AXI_GPIO_2492_RDWR_NUM_BASEADDR, 0x81);
//    /* Readout register value */
//    status = LMX2492_write_register(&txData, &tmp);
//    if (status != LMX2492TransStatusOk)
//    {
//        return LMX2492TransStatusError;
//    }
//
//    *rxData = tmp & mask;
//
//    return status;
//
//}

lmx2492_status_t LMX2492_read_register(uint32_t data, uint8_t *value)
{
    lmx2492_status_t status = LMX2492TransStatusOk;

	Xil_Out32(XPAR_AXI_GPIO_2492_RDWR_NUM_BASEADDR, 0x81);
	Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44, data);
	usleep(100);

	*value = Xil_In32(XPAR_AXI2REG2492_0_BASEADDR + 4);

    return status;

}


/* individual register setting. */
lmx2492_status_t LMX2492_register_setting(uint16_t regAddr, uint8_t value, uint8_t mask, uint8_t shift)
{

	lmx2492_status_t status = LMX2492TransStatusOk;

	uint8_t tmp = lmx2492_register_data_array[regAddr];

    uint8_t writeData = (value << shift) & mask;

    tmp  = (tmp & (~mask)) | writeData;

    lmx2492_register_data_array[regAddr] = tmp;

    return status;

}



//lmx2492_status_t LMX2492_register_setting(uint16_t regAddr, uint8_t value, uint8_t mask, uint8_t shift)
//{
//    lmx2492_status_t status = LMX2492TransStatusOk;
//
//    lmx2492_data_frame_t txData = {
//        .commandType = LMX2492RegRead,
//        .regAddress  = regAddr };
//    uint8_t rxData = 0;
//    uint8_t tmp = 0;
//
//    /* Readout register value */
//    status = LMX2492_write_register(&txData, &rxData);
//    if (status != LMX2492TransStatusOk)
//    {
//        return LMX2492TransStatusError;
//    }
//
//    /* Write to register after modifying corresponding bit */
//    txData.commandType = LMX2492RegWrite;
//    tmp  = rxData & (~mask);
//    tmp |= (value & mask) << shift;
//
//    txData.writeData = tmp;
//
//    status = LMX2492_write_register(&txData, &rxData);
//    if (status != LMX2492TransStatusOk)
//    {
//        return LMX2492TransStatusError;
//    }
//
//    /* Read the register value again to determine whether the write was successful */
//    txData.commandType = LMX2492RegRead;
//    rxData = 0;
//
//    status = LMX2492_write_register(&txData, &rxData);
//    if (status != LMX2492TransStatusOk)
//    {
//        return LMX2492TransStatusError;
//    }
//
//    if (rxData != ((value & mask) << shift)) {
//        return LMX2492WriteRegisterError;
//    }
//
//    return status;
//}
