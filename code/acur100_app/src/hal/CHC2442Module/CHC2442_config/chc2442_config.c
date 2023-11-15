/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    chc2442_config.c
 * @author  X22012
 * @date    2022年05月15日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年05月15日
 *         Version: V1.0
 *         details: Created
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "chc2442_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/*******************************************************************************
 * Local Functions - Implementation
 ******************************************************************************/

/*******************************************************************************
 * API - Implementation
 ******************************************************************************/
/* Performs a read to 2442 register. */
chc2442_status_t CHC2442_reading(uint32_t *rxData)
{
	chc2442_status_t status = chc2442TransStatusOk;

	status = CHC2442_read_register(rxData);
	if (status != chc2442TransStatusOk)
	{
		return chc2442TransStatusError;
	}

	return status;
}

/* Performs a write to 2442 register. */
chc2442_status_t CHC2442_writing(uint32_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;

    uint32_t txData = value & SPI_REGISTER_MASK;

    status = CHC2442_write_register(txData);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;

}

/* Setting VGA gain. */
chc2442_status_t CHC2442_VGA_setting(chc2442_vga_gain_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = VGAGainIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting IF LPF, 0: Fcut=25MHz(nominal), 1: 60MHz. */
chc2442_status_t CHC2442_IF_LPF_setting(chc2442_low_pass_filter_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = IFLowPassFilterIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting MPA nominal power value. */
chc2442_status_t CHC2442_MPA_setting(chc2442_mpa_nominal_power_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = MPAPowerGainIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting MPA mute value. */
chc2442_status_t CHC2442_MPA_mute_setting(chc2442_mpa_mute_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = MPAMuteIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting LO amps value. */
chc2442_status_t CHC2442_LO_amplifier_setting(chc2442_lo_amplifier_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = LOAmplifierEnableIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting IF HPF value. */
chc2442_status_t CHC2442_IF_HPF_setting(chc2442_high_pass_filter_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = IFhighPassFilterIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting prescaler value. */
chc2442_status_t CHC2442_prescaler_value_setting(chc2442_prescaler_mode_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = PrescalerIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting pilot tone value. */
chc2442_status_t CHC2442_pilot_tone_setting(uint8_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = PilotToneIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting mux value. */
chc2442_status_t CHC2442_MUX_mode_setting(chc2442_mux_mode_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = MUXIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting DC power mode value. */
chc2442_status_t CHC2442_DC_power_mode_setting(chc2442_direct_power_mode_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = DCPowerModeIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting IF amp gain value. */
chc2442_status_t CHC2442_IF_amplifier_gain_setting(chc2442_if_gain_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = IFAmplifierGainIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting 1.5MHz mode value. */
chc2442_status_t CHC2442_MHz_mode_setting(chc2442_mhz_mode_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = MHzModeIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting mixer gain value. */
chc2442_status_t CHC2442_mixer_gain_setting(chc2442_mixer_gain_value_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = MixerGainIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}

/* Setting IC enable value. */
chc2442_status_t CHC2442_IC_enable_setting(chc2442_ic_enable_setting_t value)
{
    chc2442_status_t status = chc2442TransStatusOk;
    chc2442_para_idx_t idx = ICEnableIndex;

    status = CHC2442_register_setting(idx, value);
    if (status != chc2442TransStatusOk)
    {
        return chc2442TransStatusError;
    }

    return status;
}
