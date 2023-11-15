/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    CHC2442_spi_hal.h
 * @author  X22012
 * @date    2022骞�05鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�05鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */


#ifndef CHC2442_SPI_HAL_H_
#define CHC2442_SPI_HAL_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
//#include "S32R294.h"
#include "chc2442_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI_COMM_TIMEOUT_MS 5

/*******************************************************************************
 * Prototypes of extern functions
 ******************************************************************************/
/** @defgroup Extern MCU specific functions
 * @brief Functions in this group must be implemented by the user.
 * @{

 * @param [in]  txFrame             Frame to be send.
 * @param [in]  frameLengthBytes    Bytes Length of the frame in bytes.
 * @return @ref chc2442_status_t "Status return code." */
extern chc2442_status_t CHC2442_SPI_TransData(uint32_t txFrame);

#endif /* CHC2442_SPI_HAL_H_ */
