/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   CHC2442 driver module
 * @file    chc2442_SPI.h
 * @author  X22012
 * @date    2022年05月15日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年05月15日
 *         Version: V1.0
 *         details: Created
 */

#ifndef CHC2442QPG_COMMON_H_
#define CHC2442QPG_COMMON_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
//#include <cstdint>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI_REG_LENGTH                   (0X18U)

/* ----------------------------------------------------------------------------
   -- SPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI Register Masks
 * @{
 */

/*! @name Module Configuration Register */
/*! @{ */
#define SPI_REGISTER_MASK                (0xFFFFFFU)
#define SPI_REGISTER_SHIFT               (0U)
#define SPI_REGISTER_WIDTH               (24U)

#define SPI_REG_VGA_GAIN_MASK            (0x7U)
#define SPI_REG_VGA_GAIN_SHIFT           (0U)
#define SPI_REG_VGA_GAIN_WIDTH           (3U)

#define SPI_REG_IF_LPF_MASK              (0x8U)
#define SPI_REG_IF_LPF_SHIFT             (3U)
#define SPI_REG_IF_LPF_WIDTH             (1U)

#define SPI_REG_MPA_POWER_MASK           (0xF0U)
#define SPI_REG_MPA_POWER_SHIFT          (4U)
#define SPI_REG_MPA_POWER_WIDTH          (4U)

#define SPI_REG_MPA_MUTE_MASK            (0x100U)
#define SPI_REG_MPA_MUTE_SHIFT           (8U)
#define SPI_REG_MPA_MUTE_WIDTH           (1U)

#define SPI_REG_LO_AMPS_MASK             (0x200U)
#define SPI_REG_LO_AMPS_SHIFT            (9U)
#define SPI_REG_LO_AMPS_WIDTH            (1U)

#define SPI_REG_IF_HPF_MASK              (0x400U)
#define SPI_REG_IF_HPF_SHIFT             (10U)
#define SPI_REG_IF_HPF_WIDTH             (1U)

#define SPI_REG_PRESCALER_MASK           (0x3000U)
#define SPI_REG_PRESCALER_SHIFT          (12U)
#define SPI_REG_PRESCALER_WIDTH          (2U)

#define SPI_REG_PILOT_TONE_MASK          (0x4000U)
#define SPI_REG_PILOT_TONE_SHIFT         (14U)
#define SPI_REG_PILOT_TONE_WIDTH         (1U)

#define SPI_REG_MUX_MASK                 (0x18000U)
#define SPI_REG_MUX_SHIFT                (15U)
#define SPI_REG_MUX_WIDTH                (2U)

#define SPI_REG_DC_POWER_MODE_MASK       (0x20000U)
#define SPI_REG_DC_POWER_MODE_SHIFT      (17U)
#define SPI_REG_DC_POWER_MODE_WIDTH      (1U)

#define SPI_REG_IF_AMP_GAIN_MASK         (0x1C0000U)
#define SPI_REG_IF_AMP_GAIN_SHIFT        (18U)
#define SPI_REG_IF_AMP_GAIN_WIDTH        (3U)

#define SPI_REG_MHZ_MODE_MASK            (0x200000U)
#define SPI_REG_MHZ_MODE_SHIFT           (21U)
#define SPI_REG_MHZ_MODE_WIDTH           (1U)

#define SPI_REG_MIXER_GAIN_MASK          (0x400000U)
#define SPI_REG_MIXER_GAIN_SHIFT         (22U)
#define SPI_REG_MIXER_GAIN_WIDTH         (1U)

#define SPI_REG_IC_ENABLE_MASK           (0x800000U)
#define SPI_REG_IC_ENABLE_SHIFT          (23U)
#define SPI_REG_IC_ENABLE_WIDTH          (1U)

/** @defgroup EnumsDefs Enums definition
 * @{ */
/** @brief Status return codes. */
typedef enum
{
    chc2442TransStatusOk      = 0U,   /**< No error. */
    chc2442TransStatusError   = 1U,   /**< SPI trans Error. */
    chc2442WriteRegisterError = 2U    /**< SPI trans Error. */
}chc2442_status_t;

/** @brief Command type. */
typedef enum {
    CHC2442RegRead,    /**< Register Read */
    CHC2442RegWrite    /**< Register Write */
}chc2442_command_type_t;
/** @} */

/** @defgroup StructDefs Struct definitions
 * @{ */
/** @brief Structure representing transmit data frame. */
typedef struct
{
    /** @brief Command type (R/W). */
    chc2442_command_type_t commandType;

    /** @brief Data to be written to the register.
     *
     * If commandType is "read", this value will be ignored. */
    uint32_t writeData;

}chc2442_data_frame_t;


/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief parameter index. */
typedef enum{
    VGAGainIndex,                     /**< VGA Gain index */
    IFLowPassFilterIndex,             /**< IF Low Pass Filter index */
    MPAPowerGainIndex,                /**< MPA Power index */
    MPAMuteIndex,                     /**< MPA Mute index */
    LOAmplifierEnableIndex,           /**< LO AMPS index */
    IFhighPassFilterIndex,            /**< IF High Pass Filter index */
    PrescalerIndex,                   /**< Prescaler index */
    PilotToneIndex,                   /**< Pilot Tone index */
    MUXIndex,                         /**< MUX index */
    DCPowerModeIndex,                 /**< DC Power mode index */
    IFAmplifierGainIndex,             /**< IF Amp Gain index */
    MHzModeIndex,                     /**< 1.5MHz mode  index */
    MixerGainIndex,                   /**< Mixer Gain index */
    ICEnableIndex                     /**< IC Enable index */           
}chc2442_para_idx_t;



/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief VGA gain value. */
typedef enum{
    VGAGainPositive2_6dB,              /**< +2.6dB */
    VGAGainPositive1_2dB,              /**< +1.2dB */
    VGAGainNominal0dB,                 /**< Nominal/0dB */
    VGAGainNegative1_1dB,              /**< -1.1dB */
    VGAGainNegative1_9dB,              /**< -1.9dB */
    VGAGainNegative2_9dB,              /**< -2.9dB */
    VGAGainNegative3_6dB,              /**< -3.6dB */
    VGAGainNegative4_3dB               /**< -4.3dB */          
}chc2442_vga_gain_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief IF LPF value. */
typedef enum{
    IFLowPassFilter25MHz,              /**< +2.6dB */
    IFLowPassFilter60MHz               /**< +1.2dB */        
}chc2442_low_pass_filter_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief MPA nominal power value. */
typedef enum{
    MPANominalPower0dB,                 /**< Nominal/0dB */
    MPANominalPowerNegative0_7dB,       /**< -0.7dB */
    MPANominalPowerNegative1_3dB,       /**< -1.3dB */
    MPANominalPowerNegative1_9dB,       /**< -1.9dB */
    MPANominalPowerNegative2_5dB,       /**< -2.5dB */
    MPANominalPowerNegative3_0dB,       /**< -3.0dB */
    MPANominalPowerNegative3_6dB,       /**< -3.6dB */
    MPANominalPowerNegative4_2dB,       /**< -4.2dB */
    MPANominalPowerNegative4_7dB,       /**< -4.7dB */
    MPANominalPowerNegative5_2dB,       /**< -5.2dB */
    MPANominalPowerNegative5_8dB,       /**< -5.8dB */
    MPANominalPowerNegative6_5dB,       /**< -6.5dB */
    MPANominalPowerNegative7_3dB,       /**< -7.3dB */
    MPANominalPowerNegative8_3dB,       /**< -8.3dB */
    MPANominalPowerNegative9_6dB,       /**< -9.6dB */
    MPANominalPowerNegative11_7dB       /**< -11.7dB */        
}chc2442_mpa_nominal_power_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief IF LPF value. */
typedef enum{
    MPAMuteTXOFF,                      /**< TX OFF */
    MPAMuteTXON                        /**< TX ON */        
}chc2442_mpa_mute_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief IF LPF value. */
typedef enum{
    LOAmplifierOFF,                     /**< LO amplifier OFF */
    LOAmplifierON                       /**< LO amplifier ON */        
}chc2442_lo_amplifier_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief IF LPF value. */
typedef enum{
    IFHighPassFilterNominal,             /**< Nominal IF High Pass Filter */
    IFHighPassFilterLowPass              /**< IF low pass through */        
}chc2442_high_pass_filter_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief prescaler mode. */
typedef enum{
    PrescalerOFF,                        /**< OFF */
    Frequency1_5MHz,                     /**< Frequency output 1.5MHz */
    Frequency1_5GHz,                     /**< Frequency output 1.5MHz */
    OFFAndLowPulling                     /**< OFF/Low Pulling */          
}chc2442_prescaler_mode_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief prescaler mode. */
typedef enum{
    MUXPositiveInfinity,                  /**< ANA = 鈭炩劍 */
    MUXTemperature,                       /**< ANA = T掳C */
    MUXPTX                                /**< ANA = PTX */  
}chc2442_mux_mode_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief DC power mode. */
typedef enum{
    ReducedDCPower,                      /**< Nominal IF High Pass Filter */
    P1G5nominal                          /**< IF low pass through */        
}chc2442_direct_power_mode_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief IF gain value. */
typedef enum{
    IFGainPositive4dB,                   /**< +4dB */
    IFGainPositive2dB,                   /**< +2dB */
    IFGainNominal0dB,                    /**< Nominal/0dB */
    IFGainNegative2dB,                   /**< -2dB */
    IFGainNegative4dB,                   /**< -4dB */
    IFGainNegative6dB,                   /**< -6dB */
    IFGainNegative8dB,                   /**< -8dB */
    IFGainNegative10dB                   /**< -10dB */          
}chc2442_if_gain_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief 1.5MHz mode. */
typedef enum{
    FullOutputSwing0_Vcc,                /**< Full output swing 0-Vcc */
    Mode0_1Vpp                           /**< 0.1Vpp/50鈩� */
}chc2442_mhz_mode_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief Mixer gain value. */
typedef enum{
    Nominalsubtract3dB,                  /**< nominal 鈥� 3dB */
    NominalGainvalue                     /**< nominal */        
}chc2442_mixer_gain_value_t;

/** @defgroup EnumsDefs Enums definitions
 * @{ */
/** @brief IC enable setting. */
typedef enum{
    ICEnableOFF,                         /**< OFF(except core VCO) */
    ICEnableON                           /**< ON */        
}chc2442_ic_enable_setting_t;

#endif /* CHC2442QPG_COMMON_H_ */
