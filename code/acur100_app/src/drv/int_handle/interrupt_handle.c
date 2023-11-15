/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    interrupt_handle.c
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
#include "interrupt_handle.h"

/*******************************************************************************
 * Definitions
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
#include <stdio.h>
#include "xscugic.h"
#include "xil_printf.h"
#include "xil_exception.h"

#define INT_CFG0_OFFSET 0x00000C00

// Parameter definitions
#define CHC2442_INT_ID              61
#define LMX2492_INT_ID              62
#define AWMF0165_INT_ID             63
#define RF_CPIE_INT_ID              85

#define INT_TYPE_RISING_EDGE        0x03
#define INT_TYPE_HIGHLEVEL          0x01
#define INT_TYPE_MASK               0x03


/* Definitions for peripheral AXI_GPIO_0_WRITE_READ */
#define XPAR_AXI_GPIO_0_WRITE_READ_BASEADDR 0x41200000
#define XPAR_AXI_GPIO_0_WRITE_READ_HIGHADDR 0x4120FFFF
#define XPAR_AXI_GPIO_0_WRITE_READ_DEVICE_ID 0
#define XPAR_AXI_GPIO_0_WRITE_READ_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_0_WRITE_READ_IS_DUAL 0

/* Definitions for peripheral AXI_GPIO_1_0165_INTERRUPT */
#define XPAR_AXI_GPIO_1_0165_INTERRUPT_BASEADDR 0x41210000
#define XPAR_AXI_GPIO_1_0165_INTERRUPT_HIGHADDR 0x4121FFFF
#define XPAR_AXI_GPIO_1_0165_INTERRUPT_DEVICE_ID 1
#define XPAR_AXI_GPIO_1_0165_INTERRUPT_INTERRUPT_PRESENT 1
#define XPAR_AXI_GPIO_1_0165_INTERRUPT_IS_DUAL 0


/* Definitions for peripheral AXI_GPIO_2_0165_MODEL */
#define XPAR_AXI_GPIO_2_0165_MODEL_BASEADDR 0x41220000
#define XPAR_AXI_GPIO_2_0165_MODEL_HIGHADDR 0x4122FFFF
#define XPAR_AXI_GPIO_2_0165_MODEL_DEVICE_ID 2
#define XPAR_AXI_GPIO_2_0165_MODEL_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_2_0165_MODEL_IS_DUAL 0

extern XScuGic xInterruptController;

void CHC2442_intr_Handler(void *param)
{
	int rdataPL = 0;

	rdataPL = Xil_In32(XPAR_AXI2REG2442_0_BASEADDR + 1*4);
	//xil_printf("PL CHC2442 reg1 to PS data is : 0x%x \n\r", rdataPL);

}

void LMX2492_intr_Handler2(void *param)
{
	int rdataPL = 0;

	rdataPL = Xil_In32(XPAR_AXI2REG2492_0_BASEADDR + 1*4);
   // xil_printf("PL 2492 reg1 to PS data is : 0x%x \n\r", rdataPL);
}

void AWFM0165_intr_Handler(void *param)
{
//    int sw_id = (int)param;
    // printf("SW%d int\n\r", sw_id);

    //xil_printf("=============0165 debug============ :\n");
}

void RF_CPIE_intr_Handler(void *param)
{
//    int sw_id = (int)param;
    // printf("SW%d int\n\r", sw_id);
//	bk_tran_func(TRUE);
   // xil_printf("=============0165 debug============ :\n");
}

void IntcTypeSetup(XScuGic *InstancePtr, int intId, int intType)
{
    int mask;

    intType &= INT_TYPE_MASK;
    mask = XScuGic_DistReadReg(InstancePtr, INT_CFG0_OFFSET + (intId/16)*4);
    mask &= ~(INT_TYPE_MASK << (intId%16)*2);
    mask |= intType << ((intId%16)*2);
    XScuGic_DistWriteReg(InstancePtr, INT_CFG0_OFFSET + (intId/16)*4, mask);
}

int IntcInitFunction(u16 DeviceId)
{
    int status;

    // Connect SW1~SW3 interrupt to handler
    status = XScuGic_Connect(&xInterruptController,
                             CHC2442_INT_ID,
                             (Xil_ExceptionHandler)CHC2442_intr_Handler,
                             (void *)1);
    if(status != XST_SUCCESS) return XST_FAILURE;

    status = XScuGic_Connect(&xInterruptController,
                             LMX2492_INT_ID,
                             (Xil_ExceptionHandler)LMX2492_intr_Handler2,
                             (void *)2);
    status = XScuGic_Connect(&xInterruptController,
    						 AWMF0165_INT_ID,
							 (Xil_ExceptionHandler)AWFM0165_intr_Handler,
							 (void *)3);
    if(status != XST_SUCCESS) return XST_FAILURE;

    status = XScuGic_Connect(&xInterruptController,
    						 RF_CPIE_INT_ID,
							 (Xil_ExceptionHandler)RF_CPIE_intr_Handler,
							 (void *)NULL);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Set interrupt type of SW1~SW2 to rising edge
    IntcTypeSetup(&xInterruptController, CHC2442_INT_ID, INT_TYPE_RISING_EDGE);
    IntcTypeSetup(&xInterruptController, LMX2492_INT_ID, INT_TYPE_RISING_EDGE);
    IntcTypeSetup(&xInterruptController, AWMF0165_INT_ID, INT_TYPE_RISING_EDGE);
    IntcTypeSetup(&xInterruptController, RF_CPIE_INT_ID, INT_TYPE_RISING_EDGE);

//    XScuGic_SetPriorityTriggerType(&xInterruptController, CHC2442_INT_ID, 0xb0, 0x3);
//    XScuGic_SetPriorityTriggerType(&xInterruptController, LMX2492_INT_ID, 0xb8, 0x3);
//    XScuGic_SetPriorityTriggerType(&xInterruptController, AWMF0165_INT_ID, 0xc0, 0x3);
//    XScuGic_SetPriorityTriggerType(&xInterruptController, RF_CPIE_INT_ID, 0xc8, 0x3);

    // Enable SW1~SW2 interrupts in the controller
    XScuGic_Enable(&xInterruptController, CHC2442_INT_ID);
    XScuGic_Enable(&xInterruptController, LMX2492_INT_ID);
    XScuGic_Enable(&xInterruptController, AWMF0165_INT_ID);
    XScuGic_Enable(&xInterruptController, RF_CPIE_INT_ID);


    return XST_SUCCESS;
}


void *IntcGetGicInst(void)
{
	return(void*)&xInterruptController ;
}
