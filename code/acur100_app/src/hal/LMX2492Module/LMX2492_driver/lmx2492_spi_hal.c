/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    lmx2492_spi_hal.c
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
#include "lmx2492_spi_hal.h"
#include "xparameters.h"
#include "xil_io.h"

/*******************************************************************************
 * 2492 driver extern Functions Implementation
 ******************************************************************************/

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
lmx2492_status_t LMX2492_SPI_TransData(uint8_t* txData)
{
	// status_t status;
	uint32_t data = 0;

	data = (uint32_t)((data | txData[0]) << 16 | (data | txData[1]) << 8 | txData[2]);

	Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44 , data);

	return LMX2492TransStatusOk;

}

