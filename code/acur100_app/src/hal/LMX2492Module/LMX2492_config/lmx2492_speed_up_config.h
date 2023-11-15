/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    LMX2492_config.h
 * @author  X22012
 * @date    2022骞�06鏈�01鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�06鏈�01鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef LMX2492_SPEED_UP_CONFIG_H_
#define LMX2492_SPEED_UP_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FASTLOCK_CYCLE_SLIP_REDUCTION_REGISTER      0x1BU    /*!< Cycle slip reduction register */
#define FASTLOCK_CYCLE_SLIP_REDUCTION_REG_MASK      0x60U    /*!< Cycle slip reduction register mask */
#define FASTLOCK_CYCLE_SLIP_REDUCTION_REG_SHIFT     0x5U     /*!< Cycle slip reduction register offset */
#define FASTLOCK_CYCLE_SLIP_REDUCTION_DISABLED      0x0U     /*!< Disabled CSR, POR value */
#define FASTLOCK_CYCLE_SLIP_REDUCTION_DOUBLE        0x1U     /*!< Value x 2 */
#define FASTLOCK_CYCLE_SLIP_REDUCTION_QUADRUPLE     0x2U     /*!< Value x 4 */

#define FASTLOCK_CHARGE_PUMP_GAIN_REGISTER          0x1DU    /*!< Fastlock charge pump gain register. Only when fast lock timer is counting down or a ramp is running with RAMPx_FL=1. POR value is 0 */
#define FASTLOCK_CHARGE_PUMP_GAIN_REG_MASK          0x0CU    /*!< Fastlock charge pump gain register mask */
#define FASTLOCK_CHARGE_PUMP_GAIN_REG_SHIFT         0x2U     /*!< Fastlock charge pump gain register shift */

#define FASTLOCK_TIMER_HIGH_REGISTER                0x1DU    /*!< Fractional modulator register */
#define FASTLOCK_TIMER_LOW_REGISTER                 0x20U    /*!< Fractional modulator register, POR value is 0 */
#define FASTLOCK_TIMER_HIGH_REG_MASK                0xE0U    /*!< Fractional modulator high register mask */
#define FASTLOCK_TIMER_LOW_REG_MASK                 0xFFU    /*!< Fractional modulator low register mask */
#define FASTLOCK_TIMER_HIGH_REG_SHIFT               0x5U     /*!< Fractional modulator high register shift */
#define FASTLOCK_TIMER_LOW_REG_SHIFT                0x0U     /*!< Fractional modulator low register shift */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Set LMX2492 Cycle slip reduction value.
 *  @param [in]  val      Data of CSR value.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fastlock_cycle_slip_reduction_value_setting(lmx2492_cycle_slip_reduction_value_t val);

/** @brief Set LMX2492 fastlock charge pump gain value.
 *  @param [in]  val      Data of FL_CPG value.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fastlock_charge_pump_gain_setting(lmx2492_charge_pump_gain_value_t val);

/** @brief Set LMX2492 fastlock timer value.
 *  @param [in]  val      Data of FL_TOC value.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fastlock_timer_value_setting(uint16_t val);

#endif /* LMX2492_SPEED_UP_CONFIG_H_ */
