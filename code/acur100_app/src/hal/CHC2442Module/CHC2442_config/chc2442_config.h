/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    chc2442_config.h
 * @author  X22012
 * @date    2022骞�05鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�05鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef CHC2442QPG_CONFIG_H_
#define CHC2442QPG_CONFIG_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../CHC2442_driver/chc2442_communication.h"

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
/** @brief Write CHC2442 register.
 *  @param [in]  Value      Data to be read.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_reading(uint32_t *rxData);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Data to be sent.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_writing(uint32_t Value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of VGA gain.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_VGA_setting(chc2442_vga_gain_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of low pass filter.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_IF_LPF_setting(chc2442_low_pass_filter_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of MPA.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_MPA_setting(chc2442_mpa_nominal_power_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of MPA mute.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_MPA_mute_setting(chc2442_mpa_mute_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of LO amplifier.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_LO_amplifier_setting(chc2442_lo_amplifier_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of IF HPF.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_IF_HPF_setting(chc2442_high_pass_filter_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of prescaler value.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_prescaler_value_setting(chc2442_prescaler_mode_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of pilot tone.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_pilot_tone_setting(uint8_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of MUX mode.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_MUX_mode_setting(chc2442_mux_mode_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of DC power mode.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_DC_power_mode_setting(chc2442_direct_power_mode_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of IF amplifier gain.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_IF_amplifier_gain_setting(chc2442_if_gain_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of 1.5MHz mode.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_MHz_mode_setting(chc2442_mhz_mode_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of mixer gain.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_mixer_gain_setting(chc2442_mixer_gain_value_t value);

/** @brief Write CHC2442 register.
 *  @param [in]  Value      Value of IC enable.
 * @return @ref chc2442_status_t "Status return code." */
chc2442_status_t CHC2442_IC_enable_setting(chc2442_ic_enable_setting_t value);
#endif /* CHC2442QPG_CONFIG_H_ */
