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

#ifndef LMX2492_MUX_PIN_CONFIG_H_
#define LMX2492_MUX_PIN_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TRIG1_PIN_DRIVE_STATE_REGISTER                0x24U     /*!< TRIG1 pin drive state register */
#define TRIG2_PIN_DRIVE_STATE_REGISTER                0x25U     /*!< TRIG2 pin drive state register */
#define MOD_PIN_DRIVE_STATE_REGISTER                  0x26U     /*!< MOD pin drive state register */
#define MUXOUT_PIN_DRIVE_STATE_REGISTER               0x27U     /*!< MUXout pin drive state register */

#define MUX_PIN_DRIVE_STATE_REG_MASK                  0x3U      /*!< MUX pins drive state register mask */
#define MUX_PIN_DRIVE_STATE_REG_SHIFT                 0x0U      /*!< MUX pins drive state register offset */

#define MUX_DRIVE_STATE_REG_MASK                  	  0xFCU     /*!< MUX drive state register mask */
#define MUX_DRIVE_STATE_REG_SHIFT                	  0x3U      /*!< MUX drive state register offset */

// This is the terminal drive state for the TRIG1, TRIG2, MOD, and MUXout Pins
#define COMMON_STATE_TRI_STATE                        0x0U      /*!< TRISTATE (default) */
#define COMMON_STATE_OPEN_DRAIN_OUTPUT                0x1U      /*!< Open Drain Output */
#define COMMON_STATE_PULLUP_PULLDOWN_OUTPUT           0x2U      /*!< Pullup/Pulldown Output */
#define COMMON_STATE_GND                              0x4U      /*!< GND */
#define COMMON_STATE_INVERTED_OPEN_DRAIN_OUTPUT       0x5U      /*!< Inverted Open Drain Output */
#define COMMON_STATE_INVERTED_PULLUP_PULLDOWN_OUTPUT  0x6U      /*!< Inverted Pullup/Pulldown Output */
#define COMMON_STATE_INPUT                            0x7U      /*!< Inverted Pullup/Pulldown Output */


#define COMMON_MUX_STATE_HIGH_REGISTER                0x23U     /*!< TRIG1, TRIG2, MOD and MUXout MUX state high register */
#define TRIG1_MUX_STATE_LOW_REGISTER                  0x24U     /*!< TRIG1 MUX state low register */
#define TRIG1_MUX_STATE_HIGH_REG_MASK                 0x08U     /*!< TRIG1 MUX state low register mask */
#define TRIG1_MUX_STATE_HIGH_REG_SHIFT                0x03U     /*!< TRIG1 MUX state low register mask */

#define TRIG2_MUX_STATE_LOW_REGISTER                  0x25U     /*!< TRIG2 MUX state low register */
#define TRIG2_MUX_STATE_HIGH_REG_MASK                 0x10U     /*!< TRIG2 MUX state low register mask */
#define TRIG2_MUX_STATE_HIGH_REG_SHIFT                0x04U     /*!< TRIG2 MUX state low register mask */

#define MOD_MUX_STATE_LOW_REGISTER                    0x26U     /*!< MOD MUX state low register */
#define MOD_MUX_STATE_HIGH_REG_MASK                   0x80U     /*!< MOD MUX state low register mask */
#define MOD_MUX_STATE_HIGH_REG_SHIFT                  0x07U     /*!< MOD MUX state low register mask */

#define MUXOUT_MUX_STATE_LOW_REGISTER                 0x27U     /*!< MUXout MUX state low register register */
#define MUXOUT_MUX_STATE_HIGH_REG_MASK                0x20U     /*!< MUXout MUX state low register register mask */
#define MUXOUT_MUX_STATE_HIGH_REG_SHIFT               0x05U     /*!< MUXout MUX state low register register mask */

#define COMMON_MUX_STATE_LOW_REG_MASK                 0xF8U     /*!< TRIG1, TRIG2, MOD and MUXout MUX state low register mask */
#define COMMON_MUX_STATE_LOW_REG_SHIFT                0x3U      /*!< TRIG1, TRIG2, MOD and MUXout MUX state low register offset */

// These fields control what signal is muxed to or from the TRIG1, TRIG2, MOD, and MUXout pins.
#define COMMON_MUX_STATE_GND                          0U        /*!< GND */ 
#define COMMON_MUX_STATE_INPIT_TRIG1                  1U        /*!< Input TRIG1 */
#define COMMON_MUX_STATE_INPIT_TRIG2                  2U        /*!< Input TRIG2  */
#define COMMON_MUX_STATE_INPIT_MOD                    3U        /*!< Input MOD  */
#define COMMON_MUX_STATE_OUTPUT_TRIG1                 4U        /*!< Output TRIG1 after synchronizer */
#define COMMON_MUX_STATE_OUTPUT_TRIG2                 5U        /*!< Output TRIG2 after synchronizer */
#define COMMON_MUX_STATE_OUTPUT_MOD                   6U        /*!< Output MOD after synchronizer */
#define COMMON_MUX_STATE_OUTPUT_READ_BACK             7U        /*!< Output Read back  */
#define COMMON_MUX_STATE_OUTPUT_CMP0                  8U        /*!< Output CMP0 */
#define COMMON_MUX_STATE_OUTPUT_CMP1                  9U        /*!< Output CMP1 */
#define COMMON_MUX_STATE_OUTPUT_LD                    10U       /*!< Output LD (DLD good AND CPM good) */
#define COMMON_MUX_STATE_OUTPUT_DLD                   11U       /*!< Output DLD  */
#define COMMON_MUX_STATE_OUTPUT_CMPON_GOOD            12U       /*!< Output CPMON good */
#define COMMON_MUX_STATE_OUTPUT_CMPON_TOO_HIGH        13U       /*!< Output CPMON too high */
#define COMMON_MUX_STATE_OUTPUT_CMPON_TOO_LOW         14U       /*!< Output CPMON too low */
#define COMMON_MUX_STATE_OUTPUT_RAMP_LIMIT_EXCEEDED   15U       /*!< Output ramp limit exceeded */
#define COMMON_MUX_STATE_OUTPUT_R_DIVIDE_2            16U       /*!< Output R Divide/2 */
#define COMMON_MUX_STATE_OUTPUT_R_DIVIDE_4            17U       /*!< Output R Divide/4 */ 
#define COMMON_MUX_STATE_OUTPUT_N_DIVIDE_2            18U       /*!< Output N Divide/2 */
#define COMMON_MUX_STATE_OUTPUT_N_DIVIDE_4            19U       /*!< Output N Divide/4 */
#define COMMON_MUX_STATE_OUTPUT_CMP0RAMP              22U       /*!< Output CMP0RAMP */
#define COMMON_MUX_STATE_OUTPUT_CMP1RAMP              23U       /*!< Output CMP1RAMP */
#define COMMON_MUX_STATE_OUTPUT_FASTLOCK              28U       /*!< Output Fastlock */
#define COMMON_MUX_STATE_OUTPUT_CPG_FROM_RAMP         29U       /*!< Output charge pump gain from ramp */
#define COMMON_MUX_STATE_OUTPUT_FLAG0_FROM_RAMP       30U       /*!< Output Flag0 from ramp */
#define COMMON_MUX_STATE_OUTPUT_FLAG1_FROM_RAMP       31U       /*!< Output Flag1 from ramp */
#define COMMON_MUX_STATE_OUTPUT_TRIGA                 32U       /*!< Output TRIGA */
#define COMMON_MUX_STATE_OUTPUT_TRIGB                 33U       /*!< Output TRIGB */
#define COMMON_MUX_STATE_OUTPUT_TRIGC                 34U       /*!< Output TRIGC */
#define COMMON_MUX_STATE_OUTPUT_R_DIVIDE              35U       /*!< Output R divide */
#define COMMON_MUX_STATE_OUTPUT_CPUP                  36U       /*!< Output charge pump up pulse */
#define COMMON_MUX_STATE_OUTPUT_CPDN                  37U       /*!< Output charge pump down pulse */
#define COMMON_MUX_STATE_OUTPUT_RAMP_CNT_FINISHED     38U       /*!< Output RAMP_CNT Finished */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern const uint8_t mux_pin_register_arr[4];
extern const uint8_t mux_pin_drive_state_arr[7];
extern const uint8_t mux_pin_state_arr[33];
/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Set TRIG1 pin drive state.
 *  @param [in]  val      Data of TRIG1 drive state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t trig1_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx);

/** @brief Set TRIG2 pin drive state.
 *  @param [in]  val      Data of TRIG2 drive state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t trig2_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx);

/** @brief Set MOD pin drive state.
 *  @param [in]  val      Data of MOD drive state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t mod_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx);

/** @brief Set MUXout pin drive state.
 *  @param [in]  val      Data of MUXout drive state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t muxout_drive_state_setting(lmx2492_mux_drive_pin_state_index_t drive_idx);



/** @brief Set TRIG1 state.
 *  @param [in]  val      Data of TRIG1 state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t trig1_state_setting(lmx2492_mux_state_index_t state_idx);

/** @brief Set TRIG2e state.
 *  @param [in]  val      Data of TRIG2 state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t trig2_state_setting(lmx2492_mux_state_index_t state_idx);

/** @brief Set MOD state.
 *  @param [in]  val      Data of MOD state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t mod_state_setting(lmx2492_mux_state_index_t state_idx);

/** @brief Set MUXout state.
 *  @param [in]  val      Data of MUXout state.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t muxout_state_setting(lmx2492_mux_state_index_t state_idx);

#endif /* LMX2492_MUX_PIN_CONFIG_H_ */
