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

#ifndef LMX2492_RAMP_CONFIG_H_
#define LMX2492_RAMP_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RAMP_ENABLE_REGISTER                    0x3AU   /*!< Enable the RAMP functions. When this bit is set, the Franctional Denominator is fixed to 2^24. RAMP execution begins at */
                                                            /* RAMP0 upon the PLL_N[7:0] write. The ramp should be set up before RAMP_EN is set. POR value is 0 */
#define RAMP_ENABLE_REG_MASK                    0x1U    /*!< RAMP_EN field mask */
#define RAMP_ENABLE_REG_SHIFT                   0x0U    /*!< RAMP_EN field offset */
#define RAMP_DISABLE                            0x0U    /*!< RAMP disable */
#define RAMP_ENABLE                             0x1U    /*!< RAMP enable */

#define RAMP_CLOCK_REGISTER                     0x3AU   /*!< RAMP clock input source. The ramp can be clocked by either the phase detector clock or the MOD terminal based on this */
                                                            /* selection. POR value is 0 */
#define RAMP_CLOCK_REG_MASK                     0x2U    /*!< RAMP_CLK field mask */
#define RAMP_CLOCK_REG_SHIFT                    0x1U    /*!< RAMP_CLK field shift */
#define RAMP_CLOCK_SOURCE_PD                    0x0U    /*!< Ramp clock source is phase detector */
#define RAMP_CLOCK_SOURCE_MOD                   0x1U    /*!< Ramp clock source is MOD terminal */

#define RAMP_PHASE_MODULATION_ENABLE_REGISTER   0x3AU   /*!< Phase modulation enable register. POR value is 0 */
#define RAMP_PHASE_MODULATION_ENABLE_REG_MASK   0x4U    /*!< RAMP_PM_EN field mask */
#define RAMP_PHASE_MODULATION_ENABLE_REG_SHIFT  0x2U    /*!< RAMP_PM_EN field offset */
#define RAMP_FREQUENCY_MODULATION               0x0U    /*!< Frequency modulation */
#define RAMP_PHASE_MODULATION                   0x1U    /*!< Phase modulation */

// Trigger A, B and C Sources
#define RAMP_TRIGA_SOURCE_REGISTER              0x3AU   /*!< Trigger A Source register. POR value is 0 */
#define RAMP_TRIGA_SOURCE_REG_MASK              0xF0U   /*!< RAMP_TRIGA field mask */
#define RAMP_TRIGA_SOURCE_REG_SHIFT             0x4U    /*!< RAMP_TRIGA field offset */

#define RAMP_TRIGB_SOURCE_REGISTER              0x3BU   /*!< Trigger B Source register. POR value is 0 */
#define RAMP_TRIGB_SOURCE_REG_MASK              0x0FU   /*!< RAMP_TRIGB field mask */
#define RAMP_TRIGB_SOURCE_REG_SHIFT             0x0U    /*!< RAMP_TRIGB field offset */

#define RAMP_TRIGC_SOURCE_REGISTER              0x3BU   /*!< Trigger C Source register. POR value is 0 */
#define RAMP_TRIGC_SOURCE_REG_MASK              0xF0U   /*!< RAMP_TRIGC field mask */
#define RAMP_TRIGC_SOURCE_REG_SHIFT             0x4U    /*!< RAMP_TRIGC field offset */

// Trigger A, B and C Sources status
#define NEVER_TRIGGER                           0x0U    /*!< Never Triggers (default) */
#define TRIG1_TERMINAL_RISING_EDGE              0x1U    /*!< TRIG1 terminal rising edge */
#define TRIG2_TERMINAL_RISING_EDGE              0x2U    /*!< TRIG2 terminal rising edge */
#define MOD_TERMINAL_RISING_EDGE                0x3U    /*!< MOD terminal rising edge */
#define DLD_RISING_EDGE                         0x4U    /*!< DLD rising edge */
#define CMP0_DETECTED                           0x5U    /*!< CMP0 detected (level)  */
#define RAMPX_CPG_RISING_EDGE                   0x6U    /*!< RAMPx_CPG Rising edge */
#define RAMPX_FLAG0_RISING_EDGE                 0x7U    /*!< RAMPx_FLAG0 Rising edge */
#define ALWAYS_TRIGGERED                        0x8U    /*!< Always Triggered (level) */
#define TRIG1_TERMINAL_FALLING_EDGE             0x9U    /*!< TRIG1 terminal falling edge */
#define TRIG2_TERMINAL_FALLING_EDGE             0xAU    /*!< TRIG2 terminal falling edge */
#define MOD_TERMINAL_FALLING_EDGE               0xBU    /*!< MOD terminal falling edge */
#define DLD_FALLING_EDGE                        0xCU    /*!< DLD Falling Edge */
#define CMP1_DETECTED                           0xDU    /*!< CMP1 detected (level) */
#define RAMPX_CPG_FALLING_EDGE                  0xEU    /*!< RAMPx_CPG Falling edge */
#define RAMPX_FLAG0_FALLING_EDGE                0xFU    /*!< RAMPx_FLAG0 Falling edge */

// RAMP_CMP0 register field
#define RAMP_CMP0_HIGHEST_REGISTER              0x46U   /*!<  */
#define RAMP_CMP0_FOURTH_REGISTER               0x3FU   /*!<  */
#define RAMP_CMP0_THIRD_REGISTER                0x3EU   /*!<  */
#define RAMP_CMP0_SECOND_REGISTER               0x3DU   /*!<  */
#define RAMP_CMP0_LOWEST_REGISTER               0x3CU   /*!< RAMP_CMP0 register. Twos compliment of Ramp Comparator 0 value. The MSB is R70[0]. POR value is 0 */
#define RAMP_CMP0_HIGHEST_REG_MASK              0x1U    /*!< RAMP_CMP0 highest register field mask */
#define RAMP_CMP0_COMMON_REG_MASK               0xFFU   /*!< RAMP_CMP0 field common mask */
#define RAMP_CMP0_COMMON_REG_SHIFT              0x0U    /*!< RAMP_CMP0 field common offset */

#define RAMP_CMP0_ENABLE_REGISTER               0x40U   /*!< Comparator 0 is active during each RAMP corresponding to the bit. Place a 1 for ramps it is active in and 0 for ramps */
                                                             /*!< it should be ignored. RAMP0 corresponds to R64[0], RAMP7 corresponds to R64[7] */
#define RAMP_CMP0_ENABLE_REG_MASK               0xFFU   /*!< RAMP_CMP0_EN field mask */
#define RAMP_CMP0_ENABLE_REG_SHIFT              0x0U    /*!< RAMP_CMP0_EN field offset */

// RAMP_CMP1 register field.
#define RAMP_CMP1_HIGHEST_REGISTER              0x46U   /*!< Comparator 0 is active during each RAMP corresponding to the bit. Place a 1 for ramps it is active in and 0 */
#define RAMP_CMP1_FOURTH_REGISTER               0x44U       /*!< for ramps it should be ignored. RAMP0 corresponds to R64[0], RAMP7 corresponds to R64[7] */
#define RAMP_CMP1_THIRD_REGISTER                0x43U       /*!<  */
#define RAMP_CMP1_SECOND_REGISTER               0x42U       /*!<  */
#define RAMP_CMP1_LOWEST_REGISTER               0x41U       /*!<  */
#define RAMP_CMP1_HIGHEST_REG_MASK              0x2U    /*!< RAMP_CMP1 highest register field mask */
#define RAMP_CMP1_COMMON_REG_MASK               0xFFU   /*!< RAMP_CMP1 field common mask */
#define RAMP_CMP1_HIGHEST_REG_SHIFT             0x1U    /*!< RAMP_CMP1 highest register field offset */
#define RAMP_CMP1_COMMON_REG_SHIFT              0x0U    /*!< RAMP_CMP1 field common offset */

#define RAMP_CMP1_ENABLE_REGISTER               0x40U   /*!< Comparator 1 is active during each RAMP corresponding to the bit. RAMP0 corresponds to R64[0], RAMP7 corresponds to R64[7]. */
#define RAMP_CMP1_ENABLE_REG_MASK               0xFFU   /*!< RAMP_CMP1_EN field mask */
#define RAMP_CMP1_ENABLE_REG_SHIFT              0x0U    /*!< RAMP_CMP1_EN field offset */

// FSK trigger register field.
#define FSK_TRIG_SOURCE_REGISTER                0x46U   /*!< FSK_DEV register. The MSB is R70[2].Twos compliment of the deviation value for frequency modulation and phase modulation.  */
#define FSK_TRIG_SOURCE_REG_MASK                0x60U   /*!< FSK_DEV highest register field mask */
#define FSK_TRIG_SOURCE_REG_SHIFT               0x5U    /*!< FSK_DEV field common offset */

// FSK_DEV register field.
#define FSK_DEV_HIGHEST_REGISTER                0x46U   /*!< FSK_DEV register. The MSB is R70[2].Twos compliment of the deviation value for frequency modulation and phase modulation.  */
#define FSK_DEV_FOURTH_REGISTER                 0x4AU       /*!< This value should be written with 0 when not used. */
#define FSK_DEV_THIRD_REGISTER                  0x49U       /*!<  */
#define FSK_DEV_SECOND_REGISTER                 0x48U       /*!<  */
#define FSK_DEV_LOWEST_REGISTER                 0x47U       /*!<  */
#define FSK_DEV_HIGHEST_REG_MASK                0x4U    /*!< FSK_DEV highest register field mask */
#define FSK_DEV_COMMON_REG_MASK                 0xFFU   /*!< FSK_DEV field common mask */
#define FSK_DEV_HIGHEST_REG_SHIFT               0x2U    /*!< FSK_DEV highest register field offset */
#define FSK_DEV_COMMON_REG_SHIFT                0x0U    /*!< FSK_DEV field common offset */

 // RAMP_LIMIT_LOW register field.
#define RAMP_LIMIT_LOW_HIGHEST_REGISTER         0x46U   /*!< Twos compliment of the ramp lower limit that the ramp can not go below. The ramp limit occurs before any deviation values */
#define RAMP_LIMIT_LOW_FOURTH_REGISTER          0x4EU       /*!< are included. Care must be taken if the deviation is used and the ramp limit must be set appropriately. */
#define RAMP_LIMIT_LOW_THIRD_REGISTER           0x4DU       /*!< Be aware that the MSB is R70[3]. POR value is 0X00000000 */
#define RAMP_LIMIT_LOW_SECOND_REGISTER          0x4CU       /*!<  */
#define RAMP_LIMIT_LOW_LOWEST_REGISTER          0x4BU       /*!< RAMP_LIMIT_LOW register. */
#define RAMP_LIMIT_LOW_HIGHEST_REG_MASK         0x8U    /*!< RAMP_LIMIT_LOW highest register field mask */
#define RAMP_LIMIT_LOW_COMMON_REG_MASK          0xFFU   /*!< RAMP_LIMIT_LOW field common mask */
#define RAMP_LIMIT_LOW_HIGHEST_REG_SHIFT        0x3U    /*!< RAMP_LIMIT_LOW highest register field offset */
#define RAMP_LIMIT_LOW_COMMON_REG_SHIFT         0x0U    /*!< RAMP_LIMIT_LOW field common offset */

 // RAMP_LIMIT_HIGH register field.
#define RAMP_LIMIT_HIGH_HIGHEST_REGISTER        0x46U   /*!< Twos compliment of the ramp higher limit that the ramp can not go above. The ramp limit occurs before any deviation values */
#define RAMP_LIMIT_HIGH_FOURTH_REGISTER         0x52U       /*!< are included. Care must be taken if the deviation is used and the ramp limit must be set appropriately. */
#define RAMP_LIMIT_HIGH_THIRD_REGISTER          0x51U       /*!< Be aware that the MSB is R70[4]. POR value is 0XFFFFFFFF */
#define RAMP_LIMIT_HIGH_SECOND_REGISTER         0x50U       /*!<  */
#define RAMP_LIMIT_HIGH_LOWEST_REGISTER         0x4FU       /*!< RAMP_LIMIT_HIGH register. */
#define RAMP_LIMIT_HIGH_HIGHEST_REG_MASK        0x10U   /*!< RAMP_LIMIT_HIGH highest register field mask */
#define RAMP_LIMIT_HIGH_COMMON_REG_MASK         0xFFU   /*!< RAMP_LIMIT_HIGH field common mask */
#define RAMP_LIMIT_HIGH_HIGHEST_REG_SHIFT       0x4U    /*!< RAMP_LIMIT_HIGH highest register field offset */
#define RAMP_LIMIT_HIGH_COMMON_REG_SHIFT        0x0U    /*!< RAMP_LIMIT_HIGH field common offset */

// RAMP_COUNT register field.
#define RAMP_COUNT_HIGH_REGISTER                0x54U   /*!< Number of RAMPs that will be executed before a trigger or ramp enable is brought down. Load zero if this feature is not used. */
#define RAMP_COUNT_LOW_REGISTER                 0x53U       /*!< Counter is automatically reset when RAMP_EN goes from 0 to 1. POR value is 0 */
#define RAMP_COUNT_HIGH_REG_MASK                0x1FU   /*!< RAMP_COUNT high register field mask */
#define RAMP_COUNT_LOW_REG_MASK                 0xFFU   /*!< RAMP_COUNT low register field mask */
#define RAMP_COUNT_REG_SHIFT                    0x0U    /*!< RAMP_COUNT field offset */

// RAMP_AUTO register field.
#define RAMP_AUTO_REGISTER                      0x54U   /*!< Automatically clear RAMP_EN RAMP Count hits terminal count. POR value is 0 */
#define RAMP_AUTO_REG_MASK                      0x20U   /*!< RAMP_AUTO field mask */
#define RAMP_AUTO_REG_SHIFT                     0x5U    /*!< RAMP_AUTO field offset */
#define RAMP_AUTO_DISABLE                       0x0U    /*!< RAMP_AUTO field mask */
#define RAMP_AUTO_ENABLE                        0x1U    /*!< RAMP_AUTO field offset */

// RAMP_TRIG_INC register field.
#define RAMP_TRIG_INC_REGISTER                  0x54U   /*!< RAMP Count hits terminal count Counter. To disable ramp counter, load a count value of 0. POR value is 0 */
#define RAMP_TRIG_INC_REG_MASK                  0xC0U   /*!< RAMP_TRIG_INC field mask */
#define RAMP_TRIG_INC_REG_SHIFT                 0x6U    /*!< RAMP_TRIG_INC field offset */
#define RAMP_TRIG_INC_ALL                       0x0U    /*!< RAMP_TRIG_INC field mask */
#define RAMP_TRIG_INC_TRIGA                     0x1U    /*!< RAMP_TRIG_INC field offset */
#define RAMP_TRIG_INC_TRIGB                     0x2U    /*!< RAMP_TRIG_INC field offset */
#define RAMP_TRIG_INC_TRIGC                     0x3U    /*!< RAMP_TRIG_INC field offset */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern const uint8_t trigger_source_reg_arr[3];
extern const uint8_t trigger_source_mask_arr[3];
extern const uint8_t trigger_source_shift_arr[3];
extern const uint8_t trigger_source_arr[16];
extern const uint8_t ramp_config_register_arr[5][5];
extern const uint8_t ramp_config_mask_arr[2][5];
extern const uint8_t ramp_config_shift_arr[2][5];

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Disable the ramp function.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_functions_disable(void);

/** @brief Enable the ramp function.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_functions_enable(void);

/** @brief Set RAMP clock source to phase detector.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_ramp_clock_source_phase_detector(void);

/** @brief Set RAMP clock source to MOD terminal.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_ramp_clock_source_mod_terminal(void);

/** @brief Set ramp phase modulation to frequency modulation.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_ramp_phase_modulation_frequency_modulation(void);

/** @brief Set ramp phase modulation to phase modulation.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_ramp_phase_modulation_phase_modulation(void);

/** @brief Set TRIGA source.
 *  @param [in]  source_idx      Index of source, Its value is limited by lmX2492_RAMP_trigger_source_index_t.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t triggera_source_setting(lmx2492_ramp_trigger_source_index_t source_idx);

/** @brief Set TRIGB source.
 *  @param [in]  source_idx      Index of source, Its value is limited by lmX2492_RAMP_trigger_source_index_t.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t triggerb_source_setting(lmx2492_ramp_trigger_source_index_t source_idx);

/** @brief Set TRIGC source.
 *  @param [in]  source_idx      Index of source, Its value is limited by lmX2492_RAMP_trigger_source_index_t.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t triggerc_source_setting(lmx2492_ramp_trigger_source_index_t source_idx);

/** @brief Set ramp comparator 0 value.
 *  @param [in]  val      Data of comparator 0.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_comparator0_value_setting(uint64_t val);

/** @brief Set ramp comparator 0 value.
 *  @param [in]  val      Data of ramp active.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_comparator0_enable_setting(uint8_t val);

/** @brief Set ramp comparator 1 value.
 *  @param [in]  val      Data of comparator 1.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_comparator1_value_setting(uint64_t val);

/** @brief Set ramp comparator 1 value.
 *  @param [in]  val      Data of ramp active.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_comparator1_enable_setting(uint8_t val);

/** @brief Set deviation trigger source.
 *  @param [in]  val      Index of source, Its value is limited by lmx2492_fsk_trigger_source_index_t.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fsk_deviation_trigger_setting(lmx2492_fsk_trigger_source_index_t val);

/** @brief Set FSK deviation value.
 *  @param [in]  val      Data of FSK deviation.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t fsk_deviation_value_setting(uint64_t val);

/** @brief Set ramp limit low value.
 *  @param [in]  val      Data of RAMP_LIMIT_LOW.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_limit_low_setting(uint64_t val);

/** @brief Set ramp limit high value.
 *  @param [in]  val      Data of RAMP_LIMIT_HIGH.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_limit_high_setting(uint64_t val);

/** @brief Set ramp counter value.
 *  @param [in]  val      Data of RAMP_COUNT.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t ramp_counter_value_setting(uint16_t val);

/** @brief Disable RAMP_AUTO .
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t auto_clear_ramp_en_disable(void);

/** @brief Enable RAMP_AUTO .
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t auto_clear_ramp_en_enable(void);

/** @brief Set increment trigger source of ramp counter.
 *  @param [in]  val      Index of source, Its value is limited by lmx2492_increment_trigger_source_index_t.
 *  @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t counter_increment_trigger_source_setting(lmx2492_increment_trigger_source_index_t val);

#endif /* LMX2492_RAMP_CONFIG_H_ */
