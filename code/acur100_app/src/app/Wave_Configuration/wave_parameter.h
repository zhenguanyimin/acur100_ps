/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   Realize chip function configuration
 * @file    wave_parameter.h
 * @author  X22012
 * @date    2022骞�07鏈�15鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�07鏈�15鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef WAVE_PARAMETER_H_
#define WAVE_PARAMETER_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "../../hal/LMX2492Module/LMX2492_driver/lmx2492_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
typedef enum {
    FrequencyPoint24GHz       = 0U,   	/**< 24GHz. */
	FrequencyPoint24_15GHz,   			/**< 24.15GHz. */
	FrequencyPoint24_25GHz				/**< 24.25GHz. */
}wave_fre_point_t;

typedef enum {
    RampConfigStatusOk       = 0U,   /**< No error. */
    RampConfigStatusError    = 1U,   /**< Config Error. */
}ramp_config_status_t;

/** @brief Enables fastlock and cycle slip reduction for rampx LEN clock instead of 
 * the normal 1 PFD cycle. Slows the ramp by a factor of 2. 
 * */
typedef enum {
    RampDelayDisable,           /**< 1 PFD clock per RAMP tick.(default). */
	RampDelayEnable             /**< 2 PFD clocks per RAMP tick. */
}ramp_fastlock_value_t;

/** @brief During this ramp, each increment takes 2 PFD cycles per. */
typedef enum {
    RampFastlockDisable,        /**< Disable. */
	RampFastlockEnable          /**< Enable. */
}ramp_delay_value_t;

/** @brief General purpose flags sent out of RAMP. */
typedef enum {
    RampFlagsClear,             /**< Both flag0 and flag1 are zero(default). */
	RampFlag0Set,          	    /**< Flag0 is set, flag1 is clear. */
	RampFlag1Set,               /**< Flag1 is set, flag0 is clear. */
    RampFlagsSet                /**< Both flag0 and flag1 are set. */
}ramp_flag_index_t;

/** @brief Forces a clear of the ramp accumulator.. */
typedef enum {
    RampResetDisable,           /**< Disable. */
	RampResetEnable             /**< Enable. */
}ramp_reset_value_t;

/** @brief Rampx next trigger index. */
typedef enum {
    RampxLength,                /**< RAMPx_LEN. */
	TriggerA,          	        /**< TRIG_A. */
	TriggerB,                   /**< TRIG_B. */
    TriggerC                    /**< TRIG_C. */
}ramp_next_trigger_index_t;


/** @defgroup StructDefs Struct definitions
 * @{ */
/** @brief Structure representing wave configuration parameter. */
typedef struct
{
    /** @brief Desired end frequency. */
    uint32_t increment;

    /** @brief Fastlock and cycle slip reduction for ramp. */
    ramp_fastlock_value_t fastlock_flag;

    /** @brief Ramp length(duration(us) * Fpd(MHz)). */
    uint16_t length;

    /** @brief Ramp flag. */
    ramp_flag_index_t flag_index;

    /** @brief Ramp reset flag. */
    ramp_reset_value_t  reset_flag;

    /** @brief Next ramp trigger. */
    ramp_next_trigger_index_t  trigger_index;

    /** @brief Next ramp index.  */
    lmx2492_ramp_index_t next_ramp_index;

}ramp_configuration_parameter_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/** @addtogroup API
 * @{ */
/** @brief Set ramp high and low limit frequency.
 *  @param [in]  duration      ramp duration.
 * @return @ref unint16_t ramp length */
uint16_t get_ramp_length(uint16_t duration);


/** @brief Set ramp high and low limit frequency.
 *  @param [in]  diff_fre      different frequency.
 *  @param [in]  ramp_len      ramp lnegth.
 * @return @ref unint32_t ramp increment */
uint32_t get_ramp_increment(int16_t diff_fre, uint16_t ramp_len);

/** @brief Set ramp high and low limit frequency.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_ramp_limit_frequency(uint64_t limit_high, uint64_t limit_low);

/** @brief config single frequency waveform.
 *  @param [in]  R_divider      PLL_R divider value.
 *  @param [in]  N_divider      PLL_N divider value.
 *  @param [in]  cpg_value      Charge pump gain value.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t set_vco_output_frequency(uint16_t R_divider, uint32_t N_divider, uint8_t cpg_value, uint32_t den_value, uint8_t frac_order, uint8_t frac_dither);

/** @brief config single frequency waveform.
 *  @param [in]  ramp_duration      ramp duration(us).
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t single_frequency_waveform_parameter(ramp_configuration_parameter_t *ramp_para, wave_fre_point_t fre);

/** @brief Config target waveform.
 *  @param [in]  ramp0_para      ramp0 parameters.
 *  @param [in]  ramp1_para      ramp1 parameters.
 *  @param [in]  ramp2_para      ramp2 parameters.
 *  @param [in]  ramp3_para      ramp3 parameters.
 * @return @ref lmx2492_status_t "Status return code." */
lmx2492_status_t target_waveform_parameter(uint16_t ramp_cnt, ramp_configuration_parameter_t *ramp0_para, ramp_configuration_parameter_t *ramp1_para,
											ramp_configuration_parameter_t *ramp2_para);

#endif /* WAVE_PARAMETER_H_ */
