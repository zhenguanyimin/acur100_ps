/**
 * Copyright (C), Autoroad Tech. Co., Ltd.
 * @brief   LMX2492_driver
 * @file    lmx2492_commmon.h
 * @author  X22012
 * @date    2022骞�06鏈�01鏃�
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022骞�06鏈�01鏃�
 *         Version: V1.0
 *         details: Created
 */

#ifndef LMX2492QPG_COMMON_H_
#define LMX2492QPG_COMMON_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LMX2492_REGISTER_COMMON_MASK     0X7FFF   		/* 15bit register */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @defgroup EnumsDefs Enums definition
 * @{ */
/** @brief Status return codes. */
typedef enum {
    LMX2492TransStatusOk       = 0U,   /**< No error. */
    LMX2492TransStatusError    = 1U,   /**< SPI trans Error. */
    LMX2492WriteRegisterError  = 2U,   /**< SPI write Error. */
    LMX2492RegisterStatusError = 3U    /**< Reg status Error. */
}lmx2492_status_t;

/** @brief Command type. */
typedef enum {
    LMX2492RegRead,    /**< Register Read */
    LMX2492RegWrite    /**< Register Write */
}lmx2492_command_type_t;
/** @} */

/** @defgroup StructDefs Struct definitions
 * @{ */
/** @brief Structure representing transmit data frame. */
typedef struct
{
    /** @brief register address. */
    uint16_t reg_addr;

    /** @brief Data of register.. */
    uint8_t reg_data;

}lmx2492_register_data_t;

/** @defgroup StructDefs Struct definitions
 * @{ */
/** @brief Structure representing transmit data frame. */
typedef struct
{
    /** @brief Command type (R/W). */
    lmx2492_command_type_t commandType;

    /** @brief register address and Data to be written to the register. 
     * The highest bit of the register is the read/write control bit, and the remaining 15 bits are register addresses.
     * When performing a read operation, the highest position is set to 1.t
    */
    uint16_t regAddress;

    /** @brief Data to be written to the register.
     *
     * If commandType is "read", this value will be ignored. */
    uint8_t writeData;

}lmx2492_data_frame_t;


/** @brief Dither used by the fractional modulator. */
typedef enum {
    PowerDownIgnoreCE,               /**< Power down, ignore CE. */
	PowerUpIgnoreCE,          		 /**< Power up, ignore CE. */
	PowerStateDependOnCE,            /**< Power state defined by CE terminal state. */
}lmx2492_power_state_value_t;

/** @brief Dither used by the fractional modulator. */
typedef enum {
    FractionalDitherWeak,            /**< Weak. */
    FractionalDitherMedium,          /**< Weak. */
    FractionalDitherStrong,          /**< Strong. */
    FractionalDitherDisable          /**< Disabled. */
}lmx2492_fractional_dither_value_t;

/** @brief Fractional Modulator order. */
typedef enum {
    IntegerMode,                     /**< Integer mode, POR value, Minimum N value is 16. */
    FirstOrderMode,                  /**< 1st order modulator, Minimum N value is 16. */
    SecondOrderMode,                 /**< 2nd order modulator, Minimum N value is 17. */
    ThirdOrderMode,                  /**< 3rd order modulator, Minimum N value is 19. */
    FourthOrderMode                  /**< 4th order modulator, Minimum N value is 25. */
}lmx2492_fractional_modulator_order_t;


/** @brief Sets the charge pump minimum pulse width. */
typedef enum {
    MinimumPulseWidth860PS  = 1U,    /**< 860ps. POR value. */
    MinimumPulseWidth1200PS,         /**< 1200ps. */
    MinimumPulseWidth1500PS          /**< 1500ps. */
}lmx2492_minimum_pulse_width_t;


/** @brief Common charge pump gain value, it's applicative for CPG and FL_ CPG . */
typedef enum {
    ChargePumpGainTriState,          /**< Tri-State. */
    ChargePumpGain100UA,             /**< 100uA. */
    ChargePumpGain200UA,             /**< 200uA. */
    ChargePumpGain300UA,             /**< 300uA. */
    ChargePumpGain400UA,             /**< 400uA. */
    ChargePumpGain500UA,             /**< 500uA. */
    ChargePumpGain600UA,             /**< 600uA. */
    ChargePumpGain700UA,             /**< 700uA. */
    ChargePumpGain800UA,             /**< 800uA. */
    ChargePumpGain900UA,             /**< 900uA. */
    ChargePumpGain1000UA,            /**< 1000uA. */
    ChargePumpGain1100UA,            /**< 1100uA. */
    ChargePumpGain1200UA,            /**< 1200uA. */
    ChargePumpGain1300UA,            /**< 1300uA. */
    ChargePumpGain1400UA,            /**< 1400uA. */
    ChargePumpGain1500UA,            /**< 1500uA. */
    ChargePumpGain1600UA,            /**< 1600uA. */
    ChargePumpGain1700UA,            /**< 1700uA. */
    ChargePumpGain1800UA,            /**< 1800uA. */
    ChargePumpGain1900UA,            /**< 1900uA. */
    ChargePumpGain2000UA,            /**< 2000uA. */
    ChargePumpGain2100UA,            /**< 2100uA. */
    ChargePumpGain2200UA,            /**< 2200uA. */
    ChargePumpGain2300UA,            /**< 2300uA. */
    ChargePumpGain2400UA,            /**< 2400uA. */
    ChargePumpGain2500UA,            /**< 2500uA. */
    ChargePumpGain2600UA,            /**< 2600uA. */
    ChargePumpGain2700UA,            /**< 2700uA. */
    ChargePumpGain2800UA,            /**< 2800uA. */
    ChargePumpGain2900UA,            /**< 2900uA. */
    ChargePumpGain3000UA,            /**< 3000uA. */
    ChargePumpGain3100UA             /**< 3100uA. */
}lmx2492_charge_pump_gain_value_t;

/** @brief Cycle Slip Reduction value setting. */
typedef enum {
    CycleSlipReductionDisable,       /**< Disabled CSR, POR value. */
    CycleSlipReductionDouble,        /**< Value x 2. */
    CycleSlipReductionQuadruple      /**< Value x 4. */
}lmx2492_cycle_slip_reduction_value_t;



/** @brief Charge pump voltage threshold value, it's applicative for CPM_THR_LOW and CPM_THR_HIGH . */
typedef enum {
    ChargePumpVoltageValue0,         /**< lowest. POR value is 0xA */
    ChargePumpVoltageValue1,         /**< . */
    ChargePumpVoltageValue2,         /**< . */
    ChargePumpVoltageValue3,         /**< . */
    ChargePumpVoltageValue4,         /**< . */
    ChargePumpVoltageValue5,         /**< . */
    ChargePumpVoltageValue6,         /**< . */
    ChargePumpVoltageValue7,         /**< . */
    ChargePumpVoltageValue8,         /**< . */
    ChargePumpVoltageValue9,         /**< . */
    ChargePumpVoltageValue10,        /**< POR value. */
    ChargePumpVoltageValue11,        /**< . */
    ChargePumpVoltageValue12,        /**< . */
    ChargePumpVoltageValue13,        /**< . */
    ChargePumpVoltageValue14,        /**< . */
    ChargePumpVoltageValue15,        /**< . */
    ChargePumpVoltageValue16,        /**< . */
    ChargePumpVoltageValue17,        /**< . */
    ChargePumpVoltageValue18,        /**< . */
    ChargePumpVoltageValue19,        /**< . */
    ChargePumpVoltageValue20,        /**< . */
    ChargePumpVoltageValue21,        /**< . */
    ChargePumpVoltageValue22,        /**< . */
    ChargePumpVoltageValue23,        /**< . */
    ChargePumpVoltageValue24,        /**< . */
    ChargePumpVoltageValue25,        /**< . */
    ChargePumpVoltageValue26,        /**< . */
    ChargePumpVoltageValue27,        /**< . */
    ChargePumpVoltageValue28,        /**< . */
    ChargePumpVoltageValue29,        /**< . */
    ChargePumpVoltageValue30,        /**< . */
    ChargePumpVoltageValue31,        /**< . */
    ChargePumpVoltageValue32,        /**< . */
    ChargePumpVoltageValue33,        /**< . */
    ChargePumpVoltageValue34,        /**< . */
    ChargePumpVoltageValue35,        /**< . */
    ChargePumpVoltageValue36,        /**< . */
    ChargePumpVoltageValue37,        /**< . */
    ChargePumpVoltageValue38,        /**< . */
    ChargePumpVoltageValue39,        /**< . */
    ChargePumpVoltageValue40,        /**< . */
    ChargePumpVoltageValue41,        /**< . */
    ChargePumpVoltageValue42,        /**< . */
    ChargePumpVoltageValue43,        /**< . */
    ChargePumpVoltageValue44,        /**< . */
    ChargePumpVoltageValue45,        /**< . */
    ChargePumpVoltageValue46,        /**< . */
    ChargePumpVoltageValue47,        /**< . */
    ChargePumpVoltageValue48,        /**< . */
    ChargePumpVoltageValue49,        /**< . */
    ChargePumpVoltageValue50,        /**< . */
    ChargePumpVoltageValue51,        /**< . */
    ChargePumpVoltageValue52,        /**< . */
    ChargePumpVoltageValue53,        /**< . */
    ChargePumpVoltageValue54,        /**< . */
    ChargePumpVoltageValue55,        /**< . */
    ChargePumpVoltageValue56,        /**< . */
    ChargePumpVoltageValue57,        /**< . */
    ChargePumpVoltageValue58,        /**< . */
    ChargePumpVoltageValue59,        /**< . */
    ChargePumpVoltageValue60,        /**< . */
    ChargePumpVoltageValue61,        /**< . */
    ChargePumpVoltageValue62,        /**< . */
    ChargePumpVoltageValue63,        /**< highest. */
}lmx2492_charge_pump_voltage_threshold_t;

/** @brief Set digital lock detect edge window value. */
typedef enum {
    DigitalLockDetectEdgeVALUE0,      /*!< 1ns (Fpd > 130 MHz). POR value */
    DigitalLockDetectEdgeVALUE1,      /*!< 1.7ns (80MHz, Fpd 鈮� 130MHz) */
    DigitalLockDetectEdgeVALUE2,      /*!< 3ns (60MHz , Fpd 鈮� 80 MHz) */
    DigitalLockDetectEdgeVALUE3,      /*!< 6ns (45MHz, Fpd 鈮� 60 MHz) */
    DigitalLockDetectEdgeVALUE4,      /*!< 10ns (30 MHz < Fpd 鈮� 45MHz)  */
    DigitalLockDetectEdgeVALUE5       /*!< 18ns (Fpd 鈮� 30 MHz) */
}lmx2492_digital_lock_detect_edge_t;


/** @brief MUX pins index. */
typedef enum {
    TRIG1PinIndex,                          /*!< TRIG1 */
    TRIG2PinIndex,                          /*!< TRIG2 */
    MODPinIndex,                            /*!< MOD */
    MUXoutPinIndex                          /*!< MUXout */
}lmx2492_mux_pin_index_t;

/** @brief MUX pins drive state index. */
typedef enum {
    TriStateIndex = 0,                          /*!< 0 */
    OpenDrainOutputIndex = 1,                   /*!< 1 */
    PullUpPullDownOutputIndex = 2,              /*!< 2 */
    DriverStateGroundIndex = 4,                 /*!< 4 */
    InvertedOpenDrainOutputIndex = 5,           /*!< 5 */
    InvertedPullUpPullDownOutputIndex= 6,       /*!< 6 */
    InputIndex = 7                              /*!< 7 */
}lmx2492_mux_drive_pin_state_index_t;


/** @brief MUX state index. */
typedef enum {
    StateGroundIndex,                       /*!< 0 */
    InputTrig1Index,                        /*!< 1 */
    InputTrig2Index,                        /*!< 2 */
    InputMODIndex,                          /*!< 3 */
    OutputTrig1Index,                       /*!< 4 */
    OutputTrig2Index,                       /*!< 5 */
    OutputMODIndex,                         /*!< 6 */
    OutputInReadbackIndex,                  /*!< 7 */
    OutputCMP0Index,                        /*!< 8 */
    OutputCMP1Index,                        /*!< 9 */
    OutputLDIndex,                          /*!< 10 */
    OutputDLDIndex,                         /*!< 11 */
    OutputCPMONGoodIndex,                   /*!< 12 */
    OutputCPMONTooHighIndex,                /*!< 13 */
    OutputCPMONTooLowIndex,                 /*!< 14 */
    OutputRampLimitExceededIndex,           /*!< 15 */
    OutputRDivide2Index,                    /*!< 16 */
    OutputRDivide4Index,                    /*!< 17 */
    OutputNDivide2Index,                    /*!< 18 */
    OutputNDivide4Index,                    /*!< 19 */
    OutputCPM0RampIndex,                    /*!< 20 */
    OutputCPM1RampIndex,                    /*!< 21 */
    OutputFastlockIndex,                    /*!< 22 */
    OutputCPGRampIndex,                     /*!< 23 */
    OutputFlag0RampIndex,                   /*!< 24 */
    OutputFlag1RampIndex,                   /*!< 25 */
    OutputTgigAIndex,                       /*!< 26 */
    OutputTgigBIndex,                       /*!< 27 */
    OutputTgigCIndex,                       /*!< 28 */
    OutputRDivideIndex,                     /*!< 29 */
    OutputCPUPIndex,                        /*!< 30 */
    OutputCPONIndex,                        /*!< 31 */
    OutputRampCntFinishedIndex,             /*!< 32 */
}lmx2492_mux_state_index_t;


/** @brief Ramp index. */
typedef enum {
    Ramp0Index,                             /*!< 0 */
    Ramp1Index,                             /*!< 1 */
    Ramp2Index,                             /*!< 2 */
    Ramp3Index,                             /*!< 3 */
    Ramp4Index,                             /*!< 4 */
    Ramp5Index,                             /*!< 5 */
    Ramp6Index,                             /*!< 6 */
    Ramp7Index                              /*!< 7 */
}lmx2492_ramp_index_t;

/** @brief Trigger A,B, and C Sources. */
typedef enum {
    TrigASourceIndex,                       /*!< 0 */
    TrigBSourceIndex,                       /*!< 1 */
    TrigCSourceIndex,                       /*!< 2 */
}lmx2492_ramp_trigger_index_t;

/** @brief Trigger Source index. */
typedef enum {
    NeverTriggerIndex,                       /*!< 0 */
    Trigger1RisingEdgeIndex,                 /*!< 1 */
    Trigger2RisingEdgeIndex,                 /*!< 2 */
    MODRisingEdgeIndex,                      /*!< 3 */
    DLDRisingEdgeIndex,                      /*!< 4 */
    CMP0DetectedIndex,                       /*!< 5 */
    RampCPGRisingEdgeIndex,                  /*!< 6 */
    RampFlag0RisingEdgeIndex,                /*!< 7 */
    AlwaysTriggerIndex,                      /*!< 8 */
    Trigger1FallingEdgeIndex,                /*!< 9 */
    Trigger2FallingEdgeIndex,                /*!< 10 */
    MODFallingEdgeIndex,                     /*!< 11 */
    DLDFallingEdgeIndex,                     /*!< 12 */
    CMP1DetectedIndex,                       /*!< 13 */
    RampCPGFallingEdgeIndex,                 /*!< 14 */
    RampFlag0FallingEdgeIndex                /*!< 15 */
}lmx2492_ramp_trigger_source_index_t;

/** @brief FSK Trigger Source index. */
typedef enum {
    TriggerAlwaysIndex,                      /*!< Always Triggered  */
    TriggerAIndex,                           /*!< Trigger A */
    TriggerBIndex,                           /*!< Trigger B */
    TriggerCIndex                            /*!< Trigger C */
}lmx2492_fsk_trigger_source_index_t;

/** @brief ramp config parameter index, it's applied to RAMP_CMP0, RAMP_CMP1, FSK_DEV, RAMP_LIMIT_LOW and RAMP_LIMIT_HIGH. */
typedef enum {
    RampComparator0Index,                    /*!< Ramp Comparator 0 */
    RampComparator1Index,                    /*!< Ramp Comparator 1 */
    FSKDeviatonIndex,                        /*!< FSK_DEV */
    RampLimitLow,                            /*!< low threshold */
    RampLimitHigh                            /*!< high threshold */
}lmx2492_ramp_config_parameter_index_t;

/** @brief Increment Trigger source index. */
typedef enum {
    IncrementOnRampTransitionIndex,          /*!< Always Triggered  */
    IncrementOnTriggerAIndex,                /*!< Trigger A */
    IncrementOnTriggerBIndex,                /*!< Trigger B */
    IncrementOnTriggerCIndex                 /*!< Trigger C */
}lmx2492_increment_trigger_source_index_t;


/** @brief General purpose flags sent out of RAMP index. */
typedef enum {
    RampFlagBothZeroIndex,                   /*!< Always Triggered  */
    RampFlagFlag0SetIndex,                   /*!< Trigger A */
    RampFlagFlag1SetIndex,                   /*!< Trigger B */
    RampFlagBothSetIndex                     /*!< Trigger C */
}lmx2492_ramp_flag_set_index_t;

/** @brief Ramp next trigger source index. */
typedef enum {
    RampNextTriggerRampLenIndex,             /*!< Always Triggered  */
    RampNextTriggerTrigAIndex,               /*!< Trigger A */
    RampNextTriggerTrigBIndex,               /*!< Trigger B */
    RampNextTriggerTrigCIndex                /*!< Trigger C */
}lmx2492_ramp_next_trigger_index_t;

#endif /* LMX2492QPG_COMMON_H_ */
