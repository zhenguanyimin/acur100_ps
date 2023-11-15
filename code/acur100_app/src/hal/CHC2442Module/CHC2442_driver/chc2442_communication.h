/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    driver_CHC2442.h
 * @author  X22012
 * @date    2022年05月15日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年05月15日
 *         Version: V1.0
 *         details: Created
 */

#ifndef CHC2442QPG_COMMUNICATION_H_
#define CHC2442QPG_COMMUNICATION_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "chc2442_common.h"

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Performs a read from a CHC2442 register.
 *
 * Performs a single read/write register based on provided address.
 * The response is returned in @ref chc2442_tx_frame_t structure.
 * @param [out] rxData      Structure holding the response from SBC.
 * @return @ref fs8x_status_t "Status return code." */
chc2442_status_t CHC2442_read_register(uint32_t* rxData);

/** @brief Performs a read/write from a CHC2442 register.
 *
 * Performs a single read/write register based on provided address.
 * The response is returned in @ref chc2442_tx_frame_t structure.
 * @param [in]  txData      Data to be sent.
 * @return @ref fs8x_status_t "Status return code." */
chc2442_status_t CHC2442_write_register(uint32_t txData);

/** @brief Set IF LPF value.
 *
 * Sends IF_LPF setting command to the CHC2442.
 * @param [in]  idx      register function index.
 * @param [in]  value    function value.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_register_setting(chc2442_para_idx_t index, uint8_t value);

#endif /* CHC2442QPG_COMMUNICATION_H_ */
