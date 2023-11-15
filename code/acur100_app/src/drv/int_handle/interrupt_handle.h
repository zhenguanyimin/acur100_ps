/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    interrupt_handle.h
 * @author  X22012
 * @date    2022骞�05鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�05鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef INTERRUPT_HANDLE_H_
#define INTERRUPT_HANDLE_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <xparameters.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define INTC_DEVICE_ID          XPAR_PS7_SCUGIC_0_DEVICE_ID


/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Write CHC2442 register.
 *  @param [in]  Value      Data to be read.
 * @return @ref chc2442_status_t "Status return code." */
void CHC2442_intr_Handler(void *param);


void LMX2492_intr_Handler2(void *param);

void AWFM0165_intr_Handler(void *param);

int IntcInitFunction(uint16_t DeviceId);

void *IntcGetGicInst(void);
#endif /* INTERRUPT_HANDLE_H_ */
