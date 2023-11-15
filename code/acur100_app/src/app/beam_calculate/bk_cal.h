#ifndef BK_CAL_H
#define BK_CAL_H

#include "stdint.h"
#include "stdbool.h"
#include "../../drv/awmf_0165/awmf_0165_chip.h"


#define n_azbwn  31
#define n_fybwn   4
#define az_bw_step 4
#define el_bw_step 8

#define AWMF_TX_A_CHANNEL_REGISTER			0X01
#define AWMF_RX_A_CHANNEL_REGISTER			0X02
#define AWMF_TX_B_CHANNEL_REGISTER			0X22
#define AWMF_RX_B_CHANNEL_REGISTER			0X23

#define AWMF_DISABLE_ALL_CHANNEL_VALUE		0XF000

#define AMP_CODE_VALID_VALUE_MASK        	0XF

#define AWMF_CHANNEL_ENABLE_MASK			0X0FFF
#define AWMF_COMMON_CHANNEL_GAIN_MASK		0X0F00
#define AWMF_COMMON_CHANNEL_GAIN_SHIFT		8
#define AWMF_RF4_CHANNEL_GAIN_MASK			0X00F0
#define AWMF_RF3_CHANNEL_GAIN_MASK			0X000F
#define AWMF_RF4_CHANNEL_GAIN_SHIFT			4
#define AWMF_RF3_CHANNEL_GAIN_SHIFT			0

#define AWMF_RF2_CHANNEL_GAIN_MASK			0XF0000000
#define AWMF_RF1_CHANNEL_GAIN_MASK			0X0F000000
#define AWMF_RF2_CHANNEL_GAIN_SHIFT			28
#define AWMF_RF1_CHANNEL_GAIN_SHIFT			24
#define AWMF_RF4_CHANNEL_PHASE_MASK			0X00FC0000
#define AWMF_RF3_CHANNEL_PHASE_MASK			0X0003F000
#define AWMF_RF2_CHANNEL_PHASE_MASK			0X00000FC0
#define AWMF_RF1_CHANNEL_PHASE_MASK			0X0000003F
#define AWMF_RF4_CHANNEL_PHASE_SHIFT		18
#define AWMF_RF3_CHANNEL_PHASE_SHIFT		12
#define AWMF_RF2_CHANNEL_PHASE_SHIFT		6
#define AWMF_RF1_CHANNEL_PHASE_SHIFT		0

// for T_Amp_Code and R_Amp_Code
#define AMP_CODE_RF4_GAIN_MASK			0XF000
#define AMP_CODE_RF3_GAIN_MASK			0X0F00
#define AMP_CODE_RF2_GAIN_MASK			0X00F0
#define AMP_CODE_RF1_GAIN_MASK			0X000F
#define AMP_CODE_RF4_GAIN_SHIFT			12U
#define AMP_CODE_RF3_GAIN_SHIFT			8U
#define AMP_CODE_RF2_GAIN_SHIFT			4U
#define AMP_CODE_RF1_GAIN_SHIFT			0U

#define CHANNEL_0_1_INIT_VALUE			59U
#define CHANNEL_1_0_INIT_VALUE			54U


typedef struct host_ctl_command
{
	uint8_t FreDot;
	uint8_t Beam_Numb;
	int16_t test_model;
	int16_t Beam_EL_SIN;	// ����
	int16_t Beam_EL_COS;	// ����
	int16_t Beam_AZ_SIN;	// ��λ

}host_ctl_command_t;

typedef enum
{
	// Tx
	chip_8  = 8,
	chip_9,
	chip_10,
	chip_11,
	chip_12,
	chip_13,
	chip_14,
	chip_15,
	chip_16,
	// Rx
	chip_17,
	chip_18,
	chip_19,
	chip_20,
	chip_21,
	chip_22,
	chip_23,
	chip_24
}chip_id_t;

typedef struct channel_index
{
	uint8_t row;
	uint8_t col;
}channel_index_t;

typedef struct tx_amp_value
{
	bool isSetValid;
	uint16_t comVal;
	uint16_t chnVal;
}tx_amp_value_t;

typedef struct chip_channel_index
{
	channel_index_t rf1_a;
	channel_index_t rf2_a;
	channel_index_t rf3_a;
	channel_index_t rf4_a;
	channel_index_t rf1_b;
	channel_index_t rf2_b;
	channel_index_t rf3_b;
	channel_index_t rf4_b;
}chip_channel_index_t;

typedef struct awmf_0165_bk_data {
	awmf_0165_dev_chain_data_t rfA_data; // 0165 rfA
	awmf_0165_dev_chain_data_t rfB_data; // 0165 rfB
}awmf_0165_bk_data_t;

void bk_cak_func(uint8_t frePoint, int16_t theta_az, int16_t phi_el);
void bk_tran_func(bool isTx);
void tx_amp_code_setting(uint16_t com_value, uint16_t rf_value);
ret_code_t beam_control_qureying(int16_t *theta_azimuth, int16_t *phi_elevation);

#endif /* BK_CAL_H */
