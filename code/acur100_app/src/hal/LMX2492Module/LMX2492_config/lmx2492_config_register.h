/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    lmx2492_config_register.h
 * @author  X22012
 * @date    2022骞�06鏈�01鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�06鏈�01鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef LMX2492_CONFIG_REGISTER_CONFIG_H_
#define LMX2492_CONFIG_REGISTER_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LMX2492_REGISTER0_ADDRESS         0X00U
#define LMX2492_REGISTER0_MASK            0X00U
#define LMX2492_REGISTER0_PORVALUE        0X18U


/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Reset ramp register */
void reset_ramp_register_value(void);

/** @brief Read the value of register 0, which can be used to judge whether the chip works normally after power-on.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_register0_judging(void);

/** @brief Read the value of register.
 * @param [in]  reg      register.
 * @return @ref uint16_t value." */
uint8_t LMX2492_read_single_register(uint16_t reg);

/** @brief Read the value of register.
 * @param [in]  reg      register.
 * @param [in]  val      Value to be set.
 * @return @ref uint16_t value." */
void LMX2492_set_single_register(uint16_t reg, uint8_t val);

/** @brief Set IF LPF value.
 *
 * Write all registers 0f LMX2492.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_write_all_register(void);

#endif /* LMX2492_CONFIG_REGISTER_CONFIG_H_ */
