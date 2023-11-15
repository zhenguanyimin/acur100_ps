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

#ifndef LMX2492_LD_CPM_CONFIG_H_
#define LMX2492_LD_CPM_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CPM_THR_LOW_REGISTER                    0x1EU   /*!< Charge pump voltage low threshole value. When the charge pump voltage is below this threshold, the LD gose low. */
                                                            /* It's value ranges from 0-63, corresponding to threshold from lowest to highest. POR value is 0x0A */
#define CPM_THR_LOW_REG_MASK                    0x3FU   /*!< CPM_THR_LOW register mask */
#define CPM_THR_LOW_REG_SHIFT                   0x0U    /*!< CPM_THR_LOW register offset */

#define CPM_FLAGL_REGISTER                      0x1EU   /*!< This is a read only bit. Low indicates the charge pump voltage is below the minimum threshold */
#define CPM_FLAGL_REG_MASK                      0x40U   /*!< CPM_FLAGL register mask */
#define CPM_FLAGL_REG_SHIFT                     0x5U    /*!< CPM_FLAGL register offset */
#define CPM_BELOW_LOW_THRESHOLD_FLAG            0x0U    /*!< Charge pump is below CPM_THR_LOW threshold */
#define CPM_ABOVE_LOW_THRESHOLD_FLAG            0x1U    /*!< Charge pump is above CPM_THR_LOW threshold */

#define CPM_THR_HIGH_REGISTER                   0x1FU   /*!< Charge pump voltage high threshole value. When the charge pump voltage is above this threshold, the LD gose low. */
                                                            /* It's value ranges from 0-63, corresponding to threshold from lowest to highest. POR value is 0x32 */
#define CPM_THR_HIGH_REG_MASK                   0x3FU   /*!< CPM_THR_HIGH register mask */
#define CPM_THR_HIGH_REG_SHIFT                  0x0U    /*!< CPM_THR_HIGH register offset */

#define CPM_FLAGH_REGISTER                      0x1FU   /*!< This is a read only bit. High indicates the charge pump voltage is above the maximum threshold */
#define CPM_FLAGH_REG_MASK                      0x40U   /*!< CPM_FLAGH register mask  */
#define CPM_FLAGH_REG_SHIFT                     0x5U    /*!< CPM_FLAGH register offset */
#define CPM_BELOW_HIGH_THRESHOLD_FLAG           0x0U    /*!< Charge pump is below CPM_THR_HIGH threshold */
#define CPM_ABOVE_HIGH_THRESHOLD_FLAG           0x1U    /*!< Charge pump is above CPM_THR_HIGH threshold */

#define DLD_PASS_CNT_REGISTER                   0x21U   /*!< Digital Lock Detect Filter amount register. There must be at least DLD_PASS_CNT good edges and less than DLD_ERR */
                                                            /* edges before the DLD is considered in lock. Making this number smaller will speed the detection of lock, but also */
                                                            /* will allow a higher chance of DLD chatter. POR value 0xFF */
#define DLD_PASS_CNT_REG_MASK                   0x0U    /*!< DLD_PASS_CNT register mask */
#define DLD_PASS_CNT_REG_SHIFT                  0x0U    /*!< DLD_PASS_CNT register offset */

#define DLD_ERR_CNT_REGISTER                    0x22U   /*!< Digital Lock Detect error count register. This is the maximum number of errors greater than DLD_TOL that are */
                                                            /* allowed before DLD is de-asserted. Although the default is 0, the recommended value is 4. POR value 0x0 */
#define DLD_ERR_CNT_REG_MASK                    0x1FU   /*!< DLD_ERR_CNT register mask */
#define DLD_ERR_CNT_REG_SHIFT                   0x0U    /*!< DLD_ERR_CNT register offset */

#define DLD_TOL_REGISTER                        0x22U   /*!< Digital Lock detect edge window. If both N and R edges are within this window, it is and R edges are within this */
                                                            /* window, it is farther apart in time are considered 鈥渆rror鈥� edges. Window choice depends on phase detector */
                                                            /* frequency, charge pump minimum pulse width, fractional modulator order and the users desired margin. */
#define DLD_TOL_REG_MASK                        0xE0U   /*!< DLD_TOL register mask */
#define DLD_TOL_REG_SHIFT                       0x5U    /*!< DLD_TOL register offset */
#define DLD_TOL_VALUE_ZERO                      0x0U    /*!< 1ns (Fpd > 130 MHz). POR value */
#define DLD_TOL_VALUE_ONE                       0x1U    /*!< 1.7ns (80MHz, Fpd 鈮� 130MHz) */
#define DLD_TOL_VALUE_TWO                       0x2U    /*!< 3ns (60MHz , Fpd 鈮� 80 MHz) */
#define DLD_TOL_VALUE_THREE                     0x3U    /*!< 6ns (45MHz, Fpd 鈮� 60 MHz) */
#define DLD_TOL_VALUE_FOUR                      0x4U    /*!< 10ns (30 MHz < Fpd 鈮� 45MHz)  */
#define DLD_TOL_VALUE_FIVE                      0x5U    /*!< 18ns (Fpd 鈮� 30 MHz) */


/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Set charge pump voltage low threshold value.
 *  @param [in]  val      Data of CPM_THR_LOW.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t low_threshold_value_setting(lmx2492_charge_pump_voltage_threshold_t val);

/** @brief Get charge pump voltage flag of CPM_THR_LOW.
 * @return @ref lmx2492_status_t "Status return code." */
bool get_flag_of_low_threshold(void);

/** @brief Set charge pump voltage high threshold value.
 *  @param [in]  val      Data of CPM_THR_HIGH.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t high_threshold_value_setting(lmx2492_charge_pump_voltage_threshold_t val);

/** @brief Get charge pump voltage flag of CPM_THR_HIGH.
 * @return @ref lmx2492_status_t "Status return code." */
bool get_flag_of_high_threshold(void);

/** @brief Set digital lock detect filter amount.
 *  @param [in]  val      Data of DLD_PASS_CNT.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t digital_lock_detect_filter_amount_setting(uint8_t val);

/** @brief Set digital lock detect error count.
 *  @param [in]  val      Data of DLD_REE_CNT.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t digital_lock_detect_error_count_setting(uint8_t val);

/** @brief Set digital lock detect edge window value.
 *  @param [in]  val      Data of DLD_TOL.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t digital_lock_detect_edge_value_setting(lmx2492_digital_lock_detect_edge_t val);

#endif /* LMX2492_LD_CPM_CONFIG_H_ */
