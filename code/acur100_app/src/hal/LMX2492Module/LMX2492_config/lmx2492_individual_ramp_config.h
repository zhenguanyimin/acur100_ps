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

#ifndef LMX2492_INDIVIDUAL_RAMP_CONFIG_H_
#define LMX2492_INDIVIDUAL_RAMP_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

    /* ******************************************************** Common character for Ramp0-7, See the 8.3.11 section for details ******************************************************** 
     * 1 Ramp Length: it's programming filed includes RAMPx_LEN and RAMPx_DLY.
     *                The user programs the length of the ramp in phase detector cycles. If RAMPx_DLY=1, then each count of RAMPx_LEN is actually two phase detector cycles.
     * 
     * 2 Ramp Slope: it's programming filed includes RAMPx_LEN銆丷AMPx_DLY and RAMPx_INC.
     *              The user does not directly program slope of the line, but rather this is done by defining how long the ramp is and how much the fractional numerator is increased 
     *              per phase detector cycle. The value for RAMPx_INC is calculated by taking the total expected increase in the frequency, expressed in terms of how much the fractional 
     *              numerator increases, and dividing it by RAMPx_LEN. The value programmed into RAMPx_INC is actually the two's complement of the desired mathematical value.
     * 
     * 3 Trigger for Next Ramp: The event that triggers the next ramp can be defined to be the ramp finishing or can wait for a trigger as defined by TRIG A, TRIG B, or TRIG C.
     * 
     * 4 Next Ramp: This sets the ramp that follows. Waveforms are constructed by defining a chain ramp segments. To make the waveform repeat, make RAMPx_NEXT point to the first ramp 
     *              in the pattern.
     * 
     * 5 Ramp Fastlock: This allows the ramp to use a different charge pump current or use Fastlock.
     * 
     * 6 Ramp Flags: This allows the ramp to set a flag that can be routed to external terminals to trigger other devices.
     ********************************************************************************************************************************************************************************* */

// RAMP0 register field
#define RAMP0_INCREMENT_HIGHEST_REGISTER        0x59U    /*!<  */
#define RAMP0_INCREMENT_THIRD_REGISTER          0x58U    /*!<  */
#define RAMP0_INCREMENT_SECOND_REGISTER         0x57U    /*!<  */
#define RAMP0_INCREMENT_LOWEST_REGISTER         0x56U    /*!< Signed ramp increment. */
#define RAMP0_FASTLOCK_REGISTER                 0x59U    /*!< This enables fastlock and cycle slip reduction for rampx. */
#define RAMP0_DELAY_REGISTER                    0x59U    /*!< During this ramp, each increment takes 2 PFD cycles per LEN clock instead of the normal 1 PFD cycle. Slows the ramp by a factor of 2. */
#define RAMP0_LENGTH_HIGH_REGISTER              0x5BU    /*!<  */
#define RAMP0_LENGTH_LOW_REGISTER               0x5AU    /*!< Number of PFD clocks, maximum of 65536 cycles. */
#define RAMP0_FLAG_REGISTER                     0x5CU    /*!< General purpose FLAGS sent out of RAMP. */
#define RAMP0_RESET_REGISTER                    0x5CU    /*!< Forces a clear of the ramp accumulator. This is used to erase any accumulator creep that can occur depending on how the */
                                                            /*!< ramps are defined. Should be done at the start of a ramp pattern. */
#define RAMP0_NEXT_TRIGGER_REGISTER             0x5CU    /*!< Determines what event is necessary to cause the state machine to go to the next ramp. It can be set to when the RAMPx_LEN */
                                                            /*!< counter reaches zero or one of the events for Triggers A,B, or C. */
#define RAMP0_NEXT_REGISTER                     0x5CU    /*!< The next RAMP to execute when the length counter times out */

// RAMP1 register field
#define RAMP1_INCREMENT_HIGHEST_REGISTER        0x60U    /*!<  */
#define RAMP1_INCREMENT_THIRD_REGISTER          0x5FU    /*!<  */
#define RAMP1_INCREMENT_SECOND_REGISTER         0x5EU    /*!<  */
#define RAMP1_INCREMENT_LOWEST_REGISTER         0x5DU    /*!< Signed ramp increment. */
#define RAMP1_FASTLOCK_REGISTER                 0x60U    /*!<  */
#define RAMP1_DELAY_REGISTER                    0x60U    /*!<  */
#define RAMP1_LENGTH_HIGH_REGISTER              0x62U    /*!<  */
#define RAMP1_LENGTH_LOW_REGISTER               0x61U    /*!<  */
#define RAMP1_FLAG_REGISTER                     0x63U    /*!<  */
#define RAMP1_RESET_REGISTER                    0x63U    /*!<  */
#define RAMP1_NEXT_TRIGGER_REGISTER             0x63U    /*!<  */
#define RAMP1_NEXT_REGISTER                     0x63U    /*!<  */

// RAMP2 register field
#define RAMP2_INCREMENT_HIGHEST_REGISTER        0x67U    /*!<  */
#define RAMP2_INCREMENT_THIRD_REGISTER          0x66U    /*!<  */
#define RAMP2_INCREMENT_SECOND_REGISTER         0x65U    /*!<  */
#define RAMP2_INCREMENT_LOWEST_REGISTER         0x64U    /*!< Signed ramp increment. */
#define RAMP2_FASTLOCK_REGISTER                 0x67U    /*!<   */
#define RAMP2_DELAY_REGISTER                    0x67U    /*!<  */
#define RAMP2_LENGTH_HIGH_REGISTER              0x69U    /*!<  */
#define RAMP2_LENGTH_LOW_REGISTER               0x68U    /*!<  */
#define RAMP2_FLAG_REGISTER                     0x6AU    /*!<  */
#define RAMP2_RESET_REGISTER                    0x6AU    /*!<  */
#define RAMP2_NEXT_TRIGGER_REGISTER             0x6AU    /*!<  */
#define RAMP2_NEXT_REGISTER                     0x6AU    /*!<  */

// RAMP3 register field
#define RAMP3_INCREMENT_HIGHEST_REGISTER        0x6EU    /*!<  */
#define RAMP3_INCREMENT_THIRD_REGISTER          0x6DU    /*!<  */
#define RAMP3_INCREMENT_SECOND_REGISTER         0x6CU    /*!<  */
#define RAMP3_INCREMENT_LOWEST_REGISTER         0x6BU    /*!<  */
#define RAMP3_FASTLOCK_REGISTER                 0x6EU    /*!<   */
#define RAMP3_DELAY_REGISTER                    0x6EU    /*!<  */
#define RAMP3_LENGTH_HIGH_REGISTER              0x70U    /*!<  */
#define RAMP3_LENGTH_LOW_REGISTER               0x6FU    /*!<  */
#define RAMP3_FLAG_REGISTER                     0x71U    /*!<  */
#define RAMP3_RESET_REGISTER                    0x71U    /*!<  */
#define RAMP3_NEXT_TRIGGER_REGISTER             0x71U    /*!<  */
#define RAMP3_NEXT_REGISTER                     0x71U    /*!<  */

// RAMP4 register field
#define RAMP4_INCREMENT_HIGHEST_REGISTER        0x75U    /*!<  */
#define RAMP4_INCREMENT_THIRD_REGISTER          0x74U    /*!<  */
#define RAMP4_INCREMENT_SECOND_REGISTER         0x73U    /*!<  */
#define RAMP4_INCREMENT_LOWEST_REGISTER         0x72U    /*!< Signed ramp increment. */
#define RAMP4_FASTLOCK_REGISTER                 0x75U    /*!<   */
#define RAMP4_DELAY_REGISTER                    0x75U    /*!<  */
#define RAMP4_LENGTH_HIGH_REGISTER              0x77U    /*!<  */
#define RAMP4_LENGTH_LOW_REGISTER               0x76U    /*!<  */
#define RAMP4_FLAG_REGISTER                     0x78U    /*!<  */
#define RAMP4_RESET_REGISTER                    0x78U    /*!<  */
#define RAMP4_NEXT_TRIGGER_REGISTER             0x78U    /*!<  */
#define RAMP4_NEXT_REGISTER                     0x78U    /*!<  */

// RAMP5 register field
#define RAMP5_INCREMENT_HIGHEST_REGISTER        0x7CU    /*!<  */
#define RAMP5_INCREMENT_THIRD_REGISTER          0x7BU    /*!<  */
#define RAMP5_INCREMENT_SECOND_REGISTER         0x7AU    /*!<  */
#define RAMP5_INCREMENT_LOWEST_REGISTER         0x79U    /*!< Signed ramp increment. */
#define RAMP5_FASTLOCK_REGISTER                 0x7CU    /*!<   */
#define RAMP5_DELAY_REGISTER                    0x7CU    /*!<  */
#define RAMP5_LENGTH_HIGH_REGISTER              0x7EU    /*!<  */
#define RAMP5_LENGTH_LOW_REGISTER               0x7DU    /*!<  */
#define RAMP5_FLAG_REGISTER                     0x7FU    /*!<  */
#define RAMP5_RESET_REGISTER                    0x7FU    /*!<  */
#define RAMP5_NEXT_TRIGGER_REGISTER             0x7FU    /*!<  */
#define RAMP5_NEXT_REGISTER                     0x7FU    /*!<  */

// RAMP6 register field
#define RAMP6_INCREMENT_HIGHEST_REGISTER        0x83U    /*!<  */
#define RAMP6_INCREMENT_THIRD_REGISTER          0x82U    /*!<  */
#define RAMP6_INCREMENT_SECOND_REGISTER         0x81U    /*!<  */
#define RAMP6_INCREMENT_LOWEST_REGISTER         0x80U    /*!< Signed ramp increment. */
#define RAMP6_FASTLOCK_REGISTER                 0x83U    /*!<   */
#define RAMP6_DELAY_REGISTER                    0x83U    /*!<  */
#define RAMP6_LENGTH_HIGH_REGISTER              0x88U    /*!<  */
#define RAMP6_LENGTH_LOW_REGISTER               0x87U    /*!<  */
#define RAMP6_FLAG_REGISTER                     0x86U    /*!<  */
#define RAMP6_RESET_REGISTER                    0x86U    /*!<  */
#define RAMP6_NEXT_TRIGGER_REGISTER             0x86U    /*!<  */
#define RAMP6_NEXT_REGISTER                     0x86U    /*!<  */

// RAMP7 register field
#define RAMP7_INCREMENT_HIGHEST_REGISTER        0x8AU    /*!<  */
#define RAMP7_INCREMENT_THIRD_REGISTER          0x89U    /*!<  */
#define RAMP7_INCREMENT_SECOND_REGISTER         0x88U    /*!<  */
#define RAMP7_INCREMENT_LOWEST_REGISTER         0x87U    /*!< Signed ramp increment. */
#define RAMP7_FASTLOCK_REGISTER                 0x8AU    /*!<   */
#define RAMP7_DELAY_REGISTER                    0x8AU    /*!<  */
#define RAMP7_LENGTH_HIGH_REGISTER              0x8CU    /*!<  */
#define RAMP7_LENGTH_LOW_REGISTER               0x8BU    /*!<  */
#define RAMP7_FLAG_REGISTER                     0x8DU    /*!<  */
#define RAMP7_RESET_REGISTER                    0x8DU    /*!<  */
#define RAMP7_NEXT_TRIGGER_REGISTER             0x8DU    /*!<  */
#define RAMP7_NEXT_REGISTER                     0x8DU    /*!<  */


// Common macro definition for rampx. For the field names, x can be 0,1,2,3,4,5,6, or 7.
#define RAMPX_INCREMENT_HIGHEST_REG_MASK        0x3FU    /*!< The RAMPx_INC highest register mask */
#define RAMPX_INCREMENT_COMMON_REG_MASK         0xFFU    /*!< The RAMPx_INC register mask except the highest register */
#define RAMPX_INCREMENT_COMMON_REG_SHIFT        0x0U     /*!< The RAMPx_INC register offset */
#define RAMPX_FASTLOCK_REG_MASK                 0x40U    /*!< RAMPx_FL register mask */
#define RAMPX_FASTLOCK_REG_SHIFT                0x6U     /*!< RAMPx_FL register offset */
#define RAMPX_DELAY_REG_MASK                    0x80U    /*!< RAMPx_DLY register mask */
#define RAMPX_DELAY_REG_SHIFT                   0x7U     /*!< RAMPx_DLY register offset */
#define RAMPX_LENGTH_REG_MASK                   0xFFU    /*!< RAMPx_LEN register mask */
#define RAMPX_LENGTH_REG_SHIFT                  0x0U     /*!< RAMPx_LEN register offset */
#define RAMPX_FLAG_REG_MASK                     0x03U    /*!< RAMPx_FLAG register mask */
#define RAMPX_FLAG_REG_SHIFT                    0x0U     /*!< RAMPx_FLAG register offset */
#define RAMPX_RESET_REG_MASK                    0x4U     /*!< RAMPx_RST register mask */
#define RAMPX_RESET_REG_SHIFT                   0x2U     /*!< RAMPx_RST register offset */
#define RAMPX_NEXT_TRIGGER_REG_MASK             0x18U    /*!< RAMPx_NEXT_TRIG register mask */
#define RAMPX_NEXT_TRIGGER_REG_SHIFT            0x3U     /*!< RAMPx_NEXT_TRIG register offset */
#define RAMPX_NEXT_REG_MASK                     0xE0U    /*!< RAMPx_NEXT register mask */
#define RAMPX_NEXT_REG_SHIFT                    0x5U     /*!< RAMPx_NEXT register offset */


#define RAMPX_FASTLOCK_DISABLE                  0x0U     /*!< Disable  */
#define RAMPX_FASTLOCK_ENABLE                   0x1U     /*!< This enables fastlock and cycle slip reduction for rampx */
#define RAMPX_DELAY_CLOCK_SINGLE                0x0U     /*!< 1 PFD clock per RAMP tick.(default) */
#define RAMPX_DELAY_CLOCK_DOUBLE                0x1U     /*!< 2 PFD clocks per RAMP tick. */
#define RAMPX_FLAG_BOTH_ZERO                    0x0U     /*!< Both FLAG1 and FLAG0 are zero. (default) */
#define RAMPX_FLAG_FLAG0_SET                    0x1U     /*!< FLAG0 is set, FLAG1 is clear */
#define RAMPX_FLAG_FLAG1_SET                    0x2U     /*!< FLAG0 is clear, FLAG1 is set */
#define RAMPX_FLAG_BOTH_SET                     0x3U     /*!< Both FLAG1 and FLAG0 are set. */
#define RAMPX_RESERT_DISABLE                    0x0U     /*!< Disable ramp reset. */
#define RAMPX_RESERT_ENABLE                     0x1U     /*!< enable ramp reset. */
#define RAMPX_NEXT_TRIG_LEN                     0x0U     /*!< RAMPx_LEN counter reaches zero. */
#define RAMPX_NEXT_TRIG_A                       0x1U     /*!< TRIG_A. */
#define RAMPX_NEXT_TRIG_B                       0x2U     /*!< TRIG_B. */
#define RAMPX_NEXT_TRIG_C                       0x3U     /*!< TRIG_C. */

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern const uint8_t rampx_increment_register_arr[8][4];
extern const uint8_t rampx_fastlock_register_arr[8];
extern const uint8_t rampx_delay_register_arr[8];
extern const uint8_t rampx_length_register_arr[8][2];
extern const uint8_t rampx_flag_register_arr[8];
extern const uint8_t rampx_reset_register_arr[8];
extern const uint8_t rampx_next_trigger_register_arr[8];
extern const uint8_t rampx_next_register_arr[8];

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 *  For the field names, x can be 0,1,2,3,4,5,6, or 7
 * @{ */
/** @brief Set rampx increment value.
 *  @param [in]  ramp_idx   Index of ramp.
 *  @param [in]  value      Data of rampx increment.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_increment_value_setting(lmx2492_ramp_index_t ramp_idx, uint32_t value);

/** @brief disable fastlock of rampx.
 *  @param [in]  ramp_idx   Index of ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_fastlock_disable(lmx2492_ramp_index_t ramp_idx);

/** @brief Enable fastlock of rampx.
 *  @param [in]  ramp_idx   Index of ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_fastlock_enable(lmx2492_ramp_index_t ramp_idx);

/** @brief Set rampx increment cycle to 1 PFD clock per ramp tick(default).
 *  @param [in]  ramp_idx   Index of ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_increment_cycle_1PFD(lmx2492_ramp_index_t ramp_idx);

/** @brief Set rampx increment cycle to 2 PFD clock per ramp tick.
 *  @param [in]  ramp_idx   Index of ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_increment_cycle_2PFD(lmx2492_ramp_index_t ramp_idx);

/** @brief Set number of PFD clocks to continue to increment ramp.
 *  @param [in]  ramp_idx   Index of ramp.
 *  @param [in]  val      Data of number of PFD clocks.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_length_setting(lmx2492_ramp_index_t ramp_idx, uint16_t val);

/** @brief Set General purpose flags sent out of ramp.
 *  @param [in]  ramp_idx   Index of ramp.
 *  @param [in]  val        Data of flag.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_flag_setting(lmx2492_ramp_index_t ramp_idx, lmx2492_ramp_flag_set_index_t val);

/** @brief Disable ramp reset.
 *  @param [in]  ramp_idx   Index of ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_reset_disable(lmx2492_ramp_index_t ramp_idx);

/** @brief Enable ramp reset.
 *  @param [in]  ramp_idx   Index of ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_reset_enable(lmx2492_ramp_index_t ramp_idx);

/** @brief Set ramp next trigger.
 *  @param [in]  ramp_idx   Index of ramp.
 *  @param [in]  val        Index of next trigger.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_next_trigger_setting(lmx2492_ramp_index_t ramp_idx, lmx2492_ramp_next_trigger_index_t val);

/** @brief Set next ramp .
 *  @param [in]  ramp_idx   Index of ramp.
 *  @param [in]  val        Index of next ramp.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t rampx_next_setting(lmx2492_ramp_index_t ramp_idx, lmx2492_ramp_index_t val);

#endif /* LMX2492_INDIVIDUAL_RAMP_CONFIG_H_ */
