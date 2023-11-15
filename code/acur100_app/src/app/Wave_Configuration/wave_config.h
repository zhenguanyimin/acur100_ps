/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    wave_config.h
 * @author  X22012
 * @date    2022骞�07鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�07鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef WAVE_CONFIG_H_
#define WAVE_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "wave_parameter.h"
#include "../../hal/CHC2442Module/CHC2442_driver/chc2442_common.h"
#include "../../hal/LMX2492Module/LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Global Variables
 ******************************************************************************/


/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
///** @brief Init chc2442 value.
// * @return nothing */
void VCOPLLInitFunction(void);

/** @brief Set MUXout to readback pin.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_pin_MUXout_readback(void);

/** @brief config single frequency waveform.
 *  @param [in]  value      register value.
 * @return @ref uint8_t "Status return code." */
void set_chc2442_value(uint32_t value);

/** @brief config single frequency waveform.
 *  @param [in]  regAddr      register.
 * @return @ref uint8_t register value */
uint8_t get_single_2492_register(uint16_t regAddr);

/** @brief Set single PLL register.
 *  @param [in]  regAddr    register.
 *  @param [in]  value      register value.
 * @return @ref." */
void set_single_2492_register(uint16_t regAddr, uint8_t value);
///** @brief Set ramp high and low limit frequency.
// * @return @ref lmx2492_status_t "Status return code." */
//lmx2492_status_t set_ramp_limit_parameter(void);

/** @brief config single frequency waveform.
 *  @param [in]  ramp_dura      ramp duration(us).
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t single_frequency_waveform_configuration(wave_fre_point_t frePoint);

 /** @brief Config target waveform.
  * @param [in]  idle_time      Idle time(us).
  * @param [in]  isTrigFlag     Trigger mode.
 * @return @ref lmx2492_status_t "Status return code." */
//lmx2492_status_t target_waveform_configuration(uint16_t idle_time, bool isTrigFlag);
lmx2492_status_t target_waveform_configuration(bool isTrigFlag);

#endif /* WAVE_CONFIG_H_ */
