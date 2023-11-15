/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    lmx2492_read_register.c
 * @author  X22012
 * @date    2022年06月01日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年06月01日
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "xil_io.h"
#include "xparameters.h"
#include "xil_io.h"
#include "lmx2492_config_register.h"
#include "../LMX2492_driver/lmx2492_communication.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define LMX2492_REGISTER_NUMBER     142U   		/* Number of valid register */
#define LMX2492_REGISTER_NUMBER     107U   		/* Number of valid register */

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
void reset_ramp_register_value(void)
{
	for (int i = 86; i < 142; i++)
	{
		lmx2492_register_data_array[i] = 0;
	}
}


/* register 0: readonly reg. */
uint8_t LMX2492_read_single_register(uint16_t reg)
{

    uint8_t rdata = 0;
    uint32_t data = 0;

    data = reg & LMX2492_REGISTER_COMMON_MASK;
    data = (data << 8) | 0x8000FF;

    LMX2492_read_register(data, &rdata);

    return rdata;

}


void LMX2492_set_single_register(uint16_t reg, uint8_t val)
{

	uint32_t data = 0;

    data = reg & LMX2492_REGISTER_COMMON_MASK;
    data = ((data << 8) | val) & 0XFFFFFF;

    LMX2492_write_single_register(data);

}

/* write all registers, A total of 108 registers are operated. */
lmx2492_status_t LMX2492_write_all_register(void)
{

    lmx2492_status_t status = LMX2492TransStatusOk;
    uint32_t data;
    uint16_t reg_idx;

    for (reg_idx = 3; reg_idx < LMX2492_REGISTER_NUMBER; reg_idx++) {
    	if (reg_idx == 85 || (reg_idx > 2 && reg_idx < 16) || (reg_idx > 39 && reg_idx < 58)) {
    		// Reserved registers
    		continue;
    	}

    	data = 0;
    	data = (((data | reg_idx) << 8) | lmx2492_register_data_array[reg_idx]) & 0XFFFFFF;
    	if (reg_idx >= 19 && reg_idx <= 21)
    	{
    		Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44, data);
    	}
    	Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44, data);
    }

    Xil_Out32(XPAR_AXI2REG2492_0_BASEADDR + 44 , 0x00000201);// power up

    return status;

}

