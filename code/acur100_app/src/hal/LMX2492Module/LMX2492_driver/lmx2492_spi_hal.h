/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    lmx2492_spi_hal.h
 * @author  X22012
 * @date    2022年06月01日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年06月01日
 *         Version: V1.0
 *         details: Created
 */

#ifndef LMX2492_SPI_HAL_H_
#define LMX2492_SPI_HAL_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
//#include "S32R294.h"
#include "lmx2492_common.h"

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
 * @return @ref lmx2492_status_t "Status return code." */
extern lmx2492_status_t LMX2492_SPI_TransData(uint8_t* txFrame);

#endif /* LMX2492_SPI_HAL_H_ */
