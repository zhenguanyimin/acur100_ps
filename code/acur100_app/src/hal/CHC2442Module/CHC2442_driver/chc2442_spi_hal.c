/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    CHC2442_spi_hal.c
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
#include "chc2442_common.h"
#include "chc2442_spi_hal.h"
#include "xil_io.h"
#include "xparameters.h"
/*******************************************************************************
 * 2442 driver extern Functions Implementation
 ******************************************************************************/

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/


/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
chc2442_status_t CHC2442_SPI_TransData(uint32_t txData)
{
	// status_t status;

	/* Call of blocking driver function. */
	Xil_Out32(XPAR_AXI2REG2442_0_BASEADDR + 16 , txData);

	return chc2442TransStatusOk;
}

