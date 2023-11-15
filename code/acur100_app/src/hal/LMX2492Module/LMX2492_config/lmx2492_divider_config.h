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

#ifndef LMX2492_DIVIDER_CONFIG_H_
#define LMX2492_DIVIDER_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PLL_N_COUNTER_HIGI_REGISTER             0x12U       /*!<  */
#define PLL_N_COUNTER_MID_REGISTER              0x11U       /*!<  */
#define PLL_N_COUNTER_LOW_REGISTER              0x10U       /*!< Feedback N counter Divide register. Writing of the register R16 begins any ramp execution when RAMP_EN=1. POR value is 16*/
#define PLL_N_COUNTER_HIGI_REG_MASK             0x03U       /*!< PLL_N value high register mask */
#define PLL_N_COUNTER_COMMON_REG_MASK           0xFFU       /*!< PLL_N value middle and low register mask */
#define PLL_N_COUNTER_REG_SHIFT                 0x0U        /*!< PLL_N value register offset */
#define PLL_N_COUNTER_MAX_VALUE                 0x3FFF4U    /*!< PLL N counter Divide max value */
#define PLL_N_COUNTER_MIN_VALUE                 0x10U       /*!< PLL N counter Divide min value. POR value */

#define FRACTIONAL_DITHER_REGISTER              0x12U       /*!< Fractional dither register */
#define FRACTIONAL_DITHER_REG_MASK              0x0CU       /*!< Fractional dither register mask */
#define FRACTIONAL_DITHER_REG_SHIFT             0x2U        /*!< Fractional dither register offset*/
#define FRACTIONAL_DITHER_WEAK_VALUE            0x0U        /*!< Weak, POR value */
#define FRACTIONAL_DITHER_MEDIUM_VALUE          0x1U        /*!< Medium */
#define FRACTIONAL_DITHER_STRONG_VALUE          0x2U        /*!< Strong */
#define FRACTIONAL_DITHER_DISABLED              0x3U        /*!< Disable */

#define FRACTIONAL_ORDER_REGISTER               0x12U       /*!< Fractional modulator register */
#define FRACTIONAL_ORDER_REG_MASK               0x70U       /*!< Fractional modulator register mask */
#define FRACTIONAL_ORDER_REG_SHIFT              0x4U        /*!< Fractional modulator register offset*/
#define FRACTIONAL_ORDER_INTEGER_MODE           0x0U        /*!< Integer mode, POR value, Minimum N value is 16 */
#define FRACTIONAL_ORDER_FIRST_ORDER_MODE       0x1U        /*!< 1st order modulator, Minimum N value is 16 */
#define FRACTIONAL_ORDER_SECOND_ORDER_MODE      0x2U        /*!< 2nd order modulator, Minimum N value is 17 */
#define FRACTIONAL_ORDER_THIRD_ORDER_MODE       0x3U        /*!< 3rd order modulator, Minimum N value is 19 */
#define FRACTIONAL_ORDER_FOURTH_ORDER_MODE      0x4U        /*!< 4th order modulator, Minimum N value is 25 */

#define FRACTIONAL_NUMERATOR_HIGI_REGISTER      0x15U       /*!<  */
#define FRACTIONAL_NUMERATOR_MID_REGISTER       0x14U       /*!<  */
#define FRACTIONAL_NUMERATOR_LOW_REGISTER       0x13U       /*!< Fractional Numerator register. The value should be less than or equal to the fractional denominator. POR value is 0 */
#define FRACTIONAL_NUMERATOR_REG_MASK           0xFFU       /*!< Fractional Numerator register mask */
#define FRACTIONAL_NUMERATOR_REG_SHIFT          0x0U        /*!< Fractional Numerator register offset*/

#define FRACTIONAL_DENOMINATOR_HIGI_REGISTER    0x18U       /*!<  */
#define FRACTIONAL_DENOMINATOR_MID_REGISTER     0x17U       /*!<  */
#define FRACTIONAL_DENOMINATOR_LOW_REGISTER     0x16U       /*!< Fractional denomeinator register. If the RAMP_EN=1, this field id ignored and the denominator is fixed to 2^24. POR value is 0 */
#define FRACTIONAL_DENOMINATOR_REG_MASK         0xFFU       /*!< Fractional denomeinator register mask */
#define FRACTIONAL_DENOMINATOR_REG_SHIFT        0x0U        /*!< Fractional denomeinator register offset*/

#define PLL_R_COUNTER_HIGI_REGISTER             0x1AU       /*!<  */
#define PLL_R_COUNTER_LOW_REGISTER              0x19U       /*!< Reference divider value register. Selecting 1 will bypass counter. POR value is 1 */
#define PLL_R_COUNTER_REG_MASK                  0xFFU       /*!< Reference divider value register mask */
#define PLL_R_COUNTER_REG_SHIFT                 0x0U        /*!< Reference divider value register offset*/

#define OSC_2X_REGISTER                         0x1BU       /*!< OSC doubler register */
#define OSC_2X_REG_MASK                         0x1U        /*!< OSC doubler register mask */
#define OSC_2X_REG_SHIFT                        0x0U        /*!< OSC doubler register offset*/
#define OSC_DOUBLER_DISABLE                     0x0U        /*!< Disable doubler. POR value */
#define OSC_DOUBLER_ENABLE                      0x1U        /*!< Enable doubler */

#define PLL_R_DIFF_REGISTER                     0x1BU       /*!< Differential R counter register */
#define PLL_R_DIFF_REG_MASK                     0x4U        /*!< Differential R counter register mask */
#define PLL_R_DIFF_REG_SHIFT                    0x2U        /*!< Differential R counter register offset*/
#define R_DIVIDER_SINGLE_ENDED                  0x0U        /*!< Single-ended. POR value */
#define R_DIVIDER_DIFFERENTIAL                  0x1U        /*!< Differential */

#define PFD_DLY_REGISTER                        0x1BU       /*!< Charge pump pulse width register. Setting 1 is recommended for general use. */
#define PFD_DLY_REG_MASK                        0x18U       /*!< Charge pump pulse width register mask */
#define PFD_DLY_REG_SHIFT                       0x3U        /*!< Charge pump pulse width register offset*/
#define PULSE_WIDTH_860PS                       0x1U        /*!< 860ps. POR value */
#define PULSE_WIDTH_1200PS                      0x2U        /*!< 1200ps */
#define PULSE_WIDTH_1500PS                      0x2U        /*!< 1500ps */

#define CHARGE_PUMP_GAIN_REGISTER               0x1CU       /*!< Charge pump gain register. POR value is 0 */
#define CHARGE_PUMP_GAIN_REG_MASK               0x1FU       /*!< Charge pump gain register mask */
#define CHARGE_PUMP_GAIN_REG_SHIFT              0x0U        /*!< Charge pump gain register offset*/

#define CHARGE_PUMP_POLARITY_REGISTER           0x1CU       /*!< Charge pump polarity */
#define CHARGE_PUMP_POLARITY_REG_MASK           0x20U       /*!< Charge pump polarity mask*/
#define CHARGE_PUMP_POLARITY_REG_SHIFT          0x4U        /*!< Charge pump polarity offset*/
#define CHARGE_PUMP_POLARITY_NEGATIVE           0x0U        /*!< Negative polarity, POR value */
#define CHARGE_PUMP_POLARITY_POSITIVE           0x1U        /*!< Positive polarity */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Set LMX2492 Feedback N counter Divide value.
 *  @param [in]  val      Data of N counter Divide value.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t PLL_N_counter_divider_setting(uint32_t val);

/** @brief Set dither of LMX2492 fractional modulator value.
 *  @param [in]  val      Data of dither.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fractional_modulator_dither_setting(lmx2492_fractional_dither_value_t val);

/** @brief Set LMX2492 Fractional Modulator order.
 *  @param [in]  val      Data of fractional Modulator order.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fractional_modulator_order_setting(lmx2492_fractional_modulator_order_t val);

/** @brief Set LMX2492 Fractional numerator value.
 *  @param [in]  val      Data of fractional numerator.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fractional_numerator_value_setting(uint32_t val);

/** @brief Set LMX2492 Fractional denominator value.
 *  @param [in]  val      Data of fractional denominator.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fractional_denominator_value_setting(uint32_t val);

/** @brief Set LMX2492 Feedback N counter Divide value.
 *  @param [in]  val      Data of R counter Divide value.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t PLL_R_counter_divider_setting(uint16_t val);

/** @brief Disable LMX2492 doubler before the Reference divider.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t OSC_doubler_disable(void);

/** @brief Enable LMX2492 doubler before the Reference divider.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t OSC_doubler_enable(void);

/** @brief Set LMX2492 R counter Single-Ended.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t R_counter_single_ended(void);

/** @brief Enable LMX2492 R counter Differential.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t R_counter_differential(void);

/** @brief Set the charge pump minimum pulse width.
 *  @param [in]  val      Data of charge pump minimum pulse width.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_charge_pump_pulse_width_min(lmx2492_minimum_pulse_width_t val);

/** @brief Set the charge pump gain.
 *  @param [in]  val      Data of charge pump gain.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t charge_pump_gain_setting(lmx2492_charge_pump_gain_value_t val);

/** @brief Set the charge pump polarity negative.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_charge_pump_polarity_negative(void);

/** @brief Set the charge pump polarity positive.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_charge_pump_polarity_positive(void);


#endif /* LMX2492_DIVIDER_CONFIG_H_ */
