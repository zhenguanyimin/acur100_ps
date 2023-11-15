/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492 driver module
 * @file    lmx2492_power_config.h
 * @author  X22012
 * @date    2022骞�06鏈�01鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�06鏈�01鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef LMX2492_POWER_CONFIG_H_
#define LMX2492_POWER_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define POWERDOWN_RESET_REGISTER        0x2U    /*!< POWERDOWN and Reset control register, bit1-bit0 controls power state, bit2 controls reset state */

#define POWERDOWN_IGNORE_CE             0x0U    /*!< POWERDOWN, ignore CE. POR value */
#define POWERUP_IGNORE_CE               0x1U    /*!< POWER UP, ignore CE */
#define POWERUP_STATE_BY_CE             0x2U    /*!< Power state defined by CE terminal state */
#define POWERDOWN_REG_MASK              0x3U    /*!< POWERDOWN control field mask */
#define POWERDOWN_REG_SHIFT             0x0U    /*!< POWERDOWN control field offset */

#define NORMAL_OPERATION_STATUS         0x0U    /*!< normal operation */
#define REGISTER_RESET_STATUS           0x1U    /*!< register reset, setting bit2 sets all registers to their POR default values */
#define SEWRST_REG_MASK                 0x4U    /*!< SWSET control field mask */
#define SEWRST_REG_SHIFT                0x2U    /*!< SWSET control field offset */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/


/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Set the state of POWERDOWN and Reset control register.
 *  @param [in]  val   Register setting value.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_power_status_setting(lmx2492_power_state_value_t val);

/** @brief Set chip status to POWERDOWN and ignore CE.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_power_down_setting(void);

/** @brief Set chip status to POWERUP and ignore CE.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_power_up_setting(void);

/** @brief Set chip status to controlled by CE terminal state.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_power_CE_setting(void);

/** @brief Set chip soft reset.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t LMX2492_software_reset(void);

#endif /* LMX2492_POWER_CONFIG_H_ */
