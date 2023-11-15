/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492_driver
 * @file    lmx2492_commmunication.h
 * @author  X22012
 * @date    2022年06月01日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年06月01日
 *         Version: V1.0
 *         details: Created
 */

#ifndef LMX2492QPG_COMMUNICATION_H_
#define LMX2492QPG_COMMUNICATION_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define REGISTER_RW_BIT_MASK    0X8000U

extern uint8_t lmx2492_register_data_array[142];
/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Performs a read/write from a LMX2492 register.
 *
 * Performs a single read/write register based on provided address.
 * The response is returned in @ref lmx2492_tx_frame_t structure.
 * @param [in]  data      Data to be sent.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_write_register(lmx2492_data_frame_t* txData);

/** @brief Write single register.
 *  @param [in]  data      Data to be set.
 *  @return void */
void LMX2492_write_single_register(uint32_t data);

/** @brief Set IF LPF value.
 *
 * Sends IF_LPF setting command to the LMX2492.
 * @param [in]   register  register address.
 * @param [out]  rxData    register value.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_read_register(uint32_t data, uint8_t *value);

/** @brief Set IF LPF value.
 *
 * Sends IF_LPF setting command to the LMX2492.
 * @param [in]  idx      register function index.
 * @param [in]  value    function Value.
 * @param [in]  mask     parameters mask.
 * @param [in]  shift    shift number.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_register_setting(uint16_t regAddr, uint8_t value, uint8_t mask, uint8_t shift);

#endif /* LMX2492QPG_COMMUNICATION_H_ */
