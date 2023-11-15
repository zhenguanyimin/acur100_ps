/*************************************************************************************************
* input:
* HostCtlCmd.FreDot, frequency number:0-2, representing: 24.085GHz, 24.15GHz, 24.215GHz
*
* HostCtlCmd.Beam_EL_SIN: The sine of the pitch angle (theta), quantized in 32768
* HostCtlCmd.Beam_EL_COS: Cosine of the pitch angle (theta), quantized in 32768
* HostCtlCmd.Beam_AZ_SIN: The sine of the azimuth angle (phi), quantized in 32768
* dx: Azimuth-dimensional array element spacing, unit: m (0.0058)
* dy: The pitch element spacing, unit: m (0.0106)
* Init_PHASE_DATA:	Initial phase calibration data, divided into 3 frequency points, a total of 384 data;
* 					128 data for each frequency point; the first 64 bits are transmit phase data,
* 					the last 64 bits receive phase data
* INIT_AMP_DATA:	Initial amplitude calibration data, a total of 384 data, divided into 3 frequency points;
* 					128 data for each frequency point; the first 64 bits are transmit attenuation data,
* 					the last 64 bits receive attenuation data;
* 				 	Each channel occupies 1 byte, the upper four bits represent common attenuation,
* 				 	and the lower four bits represent channel attenuation
*
* Function: Calculate the phase code and attenuation code of the two-dimensional active array
*
* output:
* T_phi_A_Code[4][16]: transmitter phase code
* R_phi_A_Code[4][16]: receiver phase code
* T_Amp_Code[4][16]: transmitter amplitude code
* R_Amp_Code[4][16]: receiver phase code
*************************************************************************************************/

#include "math.h"
#include "bk_cal.h"
#include "xil_io.h"
#include "../../drv/awmf_0165/awmf_0165_drv.h"
#include "../../drv/awmf_0165/awmf_0165_def.h"


static float dx = 0.0058;
static float dy = 0.0106;
static uint16_t tx_com_Amp_Code[4] = {0x0000, 0x0000, 0x0000, 0x0000};
static uint16_t rx_u19_Amp_Code[4] = {0x0000, 0x0C00, 0x0000, 0x0C00};
static uint16_t rx_u23_Amp_Code[4] = {0x0C00, 0x0000, 0x0C00, 0x0000};
static uint32_t	T_Amp_Code[4][16];		// transmitter amplitude code
static uint32_t	R_Amp_Code[4][16];		// receiver amplitude code
static uint32_t T_phi_A_Code[4][16]; 	// transmitter phase code
static uint32_t	R_phi_A_Code[4][16];	// receiver phase code

static tx_amp_value_t amp_val;


volatile host_ctl_command_t  HostCtlCmd;

int16_t INIT_PHASE_DATA[384] =
{
	#include "INIT_PHASE_DATA.txt"
};

int16_t INIT_AMP_DATA[384] =
{
	#include "INIT_AMP_DATA.txt"
};

static uint8_t tx_u12_chip_id[4] = {chip_12, chip_10, chip_9,  chip_11};
static uint8_t tx_u16_chip_id[4] = {chip_16, chip_14, chip_13, chip_15};
static uint8_t rx_u19_chip_id[4] = {chip_19, chip_17, chip_18, chip_20};
static uint8_t rx_u23_chip_id[4] = {chip_23, chip_21, chip_22, chip_24};

static awmf_0165_bk_data_t tx_data_u12_chain;
static awmf_0165_bk_data_t tx_data_u16_chain;
static awmf_0165_bk_data_t rx_data_u19_chain;
static awmf_0165_bk_data_t rx_data_u23_chain;

// chain sequence: u12-u10-u9-u11, u16-u14-u13-u15
static chip_channel_index_t tx_chan_u12_idx[4] =
{
	{{0, 14}, {1, 14}, {1, 13}, {0, 13}, {0, 15}, {1, 15}, {1, 12}, {0, 12}},	// u12
	{{2, 10}, {3, 10}, {3, 9 }, {2, 9 }, {2, 11}, {3, 11}, {3,  8}, {2, 8 }},	// u10
	{{2, 14}, {3, 14}, {3, 13}, {2, 13}, {2, 15}, {3, 15}, {3, 12}, {2, 12}},	// u9
	{{0, 10}, {1, 10}, {1, 9 }, {0, 9 }, {0, 11}, {1, 11}, {1, 8 }, {0, 8 }},	// u11
};

static chip_channel_index_t tx_chan_u16_idx[4] =
{
	{{0, 2 }, {1, 2 }, {1, 1 }, {0, 1 }, {0, 3 }, {1, 3 }, {1, 0 }, {0, 0 }},	// u16
	{{2, 6 }, {3, 6 }, {3, 5 }, {2, 5 }, {2, 7 }, {3, 7 }, {3, 4 }, {2, 4 }},	// u14
	{{2, 2 }, {3, 2 }, {3, 1 }, {2, 1 }, {2, 3 }, {3, 3 }, {3, 0 }, {2, 0 }},	// u13
	{{0, 6 }, {1, 6 }, {1, 5 }, {0, 5 }, {0, 7 }, {1, 7 }, {1, 4 }, {0, 4 }}	// u15
};

// chain sequence: u19-u17-u18-u20, u23-u21-u22-u24
static chip_channel_index_t rx_chan_u19_idx[4] =
{
	{{2, 10}, {3, 10}, {3, 9 }, {2, 9 }, {2, 11}, {3, 11}, {3, 8 }, {2, 8 }},	// u19
	{{0, 14}, {1, 14}, {1, 13}, {0, 13}, {0, 15}, {1, 15}, {1, 12}, {0, 12}},	// u17
	{{0, 10}, {1, 10}, {1, 9 }, {0, 9 }, {0, 11}, {1, 11}, {1,  8}, {0, 8 }},	// u18
	{{2, 14}, {3, 14}, {3, 13}, {2, 13}, {2, 15}, {3, 15}, {3, 12}, {2, 12}},	// u20
};

static chip_channel_index_t rx_chan_u23_idx[4] =
{
	{{2, 2 }, {3, 2 }, {3, 1 }, {2, 1 }, {2, 3 }, {3, 3 }, {3, 0 }, {2, 0 }},	// u23
	{{0, 6 }, {1, 6 }, {1, 5 }, {0, 5 }, {0, 7 }, {1, 7 }, {1, 4 }, {0, 4 }},	// u21
	{{0, 2 }, {1, 2 }, {1, 1 }, {0, 1 }, {0, 3 }, {1, 3 }, {1, 0 }, {0, 0 }},	// u22
	{{2, 6 }, {3, 6 }, {3, 5 }, {2, 5 }, {2, 7 }, {3, 7 }, {3, 4 }, {2, 4 }}	// u24
};


/* Transmitter Amp code configuration, batch processing */
void tx_amp_code_setting(uint16_t com_value, uint16_t rf_value)
{
	amp_val.isSetValid = TRUE;
	amp_val.comVal = com_value;
	amp_val.chnVal = rf_value;
}


static void channel_enable_setting(bool isTxChannel)
{
	int16_t chip_idx;
	uint16_t high_val = AWMF_DISABLE_ALL_CHANNEL_VALUE;
	uint32_t low_val  = 0;

	if (isTxChannel)
	{
		for (chip_idx = 17; chip_idx <= 24; chip_idx++)
		{
			awmf_0165_drv_reg_set(chip_idx, AWMF_RX_A_CHANNEL_REGISTER, high_val, low_val);
			awmf_0165_drv_reg_set(chip_idx, AWMF_RX_B_CHANNEL_REGISTER, high_val, low_val);
		}
	}
	else
	{
		for (chip_idx = 8; chip_idx <= 16; chip_idx++)
		{
			awmf_0165_drv_reg_set(chip_idx, AWMF_TX_A_CHANNEL_REGISTER, high_val, low_val);
			awmf_0165_drv_reg_set(chip_idx, AWMF_TX_B_CHANNEL_REGISTER, high_val, low_val);
		}
	}
}


static void bk_tx_tran_data(chip_channel_index_t *chan_idx_arr, uint8_t *chip_id_arr, awmf_0165_bk_data_t *data_chain, const uint16_t *amp_code)
{
	uint16_t data_temp_H = 0;	// high 16bit
	uint32_t data_temp_L = 0;	// low 32bit
	int idx;

	uint8_t a1Row, a1Col, a2Row, a2Col, a3Row, a3Col, a4Row, a4Col, b1Row, b1Col, b2Row, b2Col, b3Row, b3Col, b4Row, b4Col;

	// enable all transmit channel
	data_temp_H &= AWMF_CHANNEL_ENABLE_MASK;

	for (idx = 0; idx < 4; idx++)
	{
		a1Row = chan_idx_arr[idx].rf1_a.row;
		a1Col = chan_idx_arr[idx].rf1_a.col;
		a2Row = chan_idx_arr[idx].rf2_a.row;
		a2Col = chan_idx_arr[idx].rf2_a.col;
		a3Row = chan_idx_arr[idx].rf3_a.row;
		a3Col = chan_idx_arr[idx].rf3_a.col;
		a4Row = chan_idx_arr[idx].rf4_a.row;
		a4Col = chan_idx_arr[idx].rf4_a.col;
		b1Row = chan_idx_arr[idx].rf1_b.row;
		b1Col = chan_idx_arr[idx].rf1_b.col;
		b2Row = chan_idx_arr[idx].rf2_b.row;
		b2Col = chan_idx_arr[idx].rf2_b.col;
		b3Row = chan_idx_arr[idx].rf3_b.row;
		b3Col = chan_idx_arr[idx].rf3_b.col;
		b4Row = chan_idx_arr[idx].rf4_b.row;
		b4Col = chan_idx_arr[idx].rf4_b.col;

		data_chain->rfA_data.chip_data[idx].chip_id = chip_id_arr[idx];
		data_chain->rfB_data.chip_data[idx].chip_id = chip_id_arr[idx];
		data_chain->rfA_data.chip_data[idx].addr_offset = AWMF_TX_A_CHANNEL_REGISTER;
		data_chain->rfB_data.chip_data[idx].addr_offset = AWMF_TX_B_CHANNEL_REGISTER;
		// rfA
		data_temp_H = amp_code[idx];
		data_temp_L = 0;
		data_temp_H |= (T_Amp_Code[a4Row][a4Col] << AWMF_RF4_CHANNEL_GAIN_SHIFT) | (T_Amp_Code[a3Row][a3Col] << AWMF_RF3_CHANNEL_GAIN_SHIFT);
		data_temp_L |= (T_Amp_Code[a2Row][a2Col] << AWMF_RF2_CHANNEL_GAIN_SHIFT) | (T_Amp_Code[a1Row][a1Col] << AWMF_RF1_CHANNEL_GAIN_SHIFT   ) |
					   (T_phi_A_Code[a4Row][a4Col] << AWMF_RF4_CHANNEL_PHASE_SHIFT) | (T_phi_A_Code[a3Row][a3Col] << AWMF_RF3_CHANNEL_PHASE_SHIFT) |
					   (T_phi_A_Code[a2Row][a2Col] << AWMF_RF2_CHANNEL_PHASE_SHIFT) | (T_phi_A_Code[a1Row][a1Col] << AWMF_RF1_CHANNEL_PHASE_SHIFT);
		data_chain->rfA_data.chip_data[idx].data_high = data_temp_H;
		data_chain->rfA_data.chip_data[idx].data_low  = data_temp_L;

		// rfB
		data_temp_H = amp_code[idx];
		data_temp_L = 0;
		data_temp_H |= (T_Amp_Code[b4Row][b4Col] << AWMF_RF4_CHANNEL_GAIN_SHIFT) | (T_Amp_Code[b3Row][b3Col] << AWMF_RF3_CHANNEL_GAIN_SHIFT);
		data_temp_L |= (T_Amp_Code[b2Row][b2Col] << AWMF_RF2_CHANNEL_GAIN_SHIFT) | (T_Amp_Code[b1Row][b1Col] << AWMF_RF1_CHANNEL_GAIN_SHIFT) |
					   (T_phi_A_Code[b4Row][b4Col] << AWMF_RF4_CHANNEL_PHASE_SHIFT) | (T_phi_A_Code[b3Row][b3Col] << AWMF_RF3_CHANNEL_PHASE_SHIFT) |
					   (T_phi_A_Code[b2Row][b2Col] << AWMF_RF2_CHANNEL_PHASE_SHIFT) | (T_phi_A_Code[b1Row][b1Col] << AWMF_RF1_CHANNEL_PHASE_SHIFT);
		data_chain->rfB_data.chip_data[idx].data_high = data_temp_H;
		data_chain->rfB_data.chip_data[idx].data_low  = data_temp_L;
	}
}

static void bk_rx_tran_data(chip_channel_index_t *chan_idx_arr, uint8_t *chip_id_arr, awmf_0165_bk_data_t *data_chain, const uint16_t *amp_code)
{
	uint16_t data_temp_H = 0;	// high 16bit
	uint32_t data_temp_L = 0;	// low 32bit
	int idx;

	uint8_t a1Row, a1Col, a2Row, a2Col, a3Row, a3Col, a4Row, a4Col, b1Row, b1Col, b2Row, b2Col, b3Row, b3Col, b4Row, b4Col;

	// enable all transmit channel
	data_temp_H &= AWMF_CHANNEL_ENABLE_MASK;

	for (idx = 0; idx < 4; idx++)
	{
		a1Row = chan_idx_arr[idx].rf1_a.row;
		a1Col = chan_idx_arr[idx].rf1_a.col;
		a2Row = chan_idx_arr[idx].rf2_a.row;
		a2Col = chan_idx_arr[idx].rf2_a.col;
		a3Row = chan_idx_arr[idx].rf3_a.row;
		a3Col = chan_idx_arr[idx].rf3_a.col;
		a4Row = chan_idx_arr[idx].rf4_a.row;
		a4Col = chan_idx_arr[idx].rf4_a.col;
		b1Row = chan_idx_arr[idx].rf1_b.row;
		b1Col = chan_idx_arr[idx].rf1_b.col;
		b2Row = chan_idx_arr[idx].rf2_b.row;
		b2Col = chan_idx_arr[idx].rf2_b.col;
		b3Row = chan_idx_arr[idx].rf3_b.row;
		b3Col = chan_idx_arr[idx].rf3_b.col;
		b4Row = chan_idx_arr[idx].rf4_b.row;
		b4Col = chan_idx_arr[idx].rf4_b.col;

		data_chain->rfA_data.chip_data[idx].chip_id = chip_id_arr[idx];
		data_chain->rfB_data.chip_data[idx].chip_id = chip_id_arr[idx];
		data_chain->rfA_data.chip_data[idx].addr_offset = AWMF_RX_A_CHANNEL_REGISTER;
		data_chain->rfB_data.chip_data[idx].addr_offset = AWMF_RX_B_CHANNEL_REGISTER;
		// rfA
		data_temp_H = amp_code[idx];
		data_temp_L = 0;
		data_temp_H |= (R_Amp_Code[a4Row][a4Col] << AWMF_RF4_CHANNEL_GAIN_SHIFT) | (R_Amp_Code[a3Row][a3Col] << AWMF_RF3_CHANNEL_GAIN_SHIFT);
		data_temp_L |= (R_Amp_Code[a2Row][a2Col] << AWMF_RF2_CHANNEL_GAIN_SHIFT) | (R_Amp_Code[a1Row][a1Col] << AWMF_RF1_CHANNEL_GAIN_SHIFT   ) |
					   (R_phi_A_Code[a4Row][a4Col] << AWMF_RF4_CHANNEL_PHASE_SHIFT) | (R_phi_A_Code[a3Row][a3Col] << AWMF_RF3_CHANNEL_PHASE_SHIFT) |
					   (R_phi_A_Code[a2Row][a2Col] << AWMF_RF2_CHANNEL_PHASE_SHIFT) | (R_phi_A_Code[a1Row][a1Col] << AWMF_RF1_CHANNEL_PHASE_SHIFT);
		data_chain->rfA_data.chip_data[idx].data_high = data_temp_H;
		data_chain->rfA_data.chip_data[idx].data_low  = data_temp_L;

		// rfB
		data_temp_H = amp_code[idx];
		data_temp_L = 0;
		data_temp_H |= (R_Amp_Code[b4Row][b4Col] << AWMF_RF4_CHANNEL_GAIN_SHIFT) | (R_Amp_Code[b3Row][b3Col] << AWMF_RF3_CHANNEL_GAIN_SHIFT);
		data_temp_L |= (R_Amp_Code[b2Row][b2Col] << AWMF_RF2_CHANNEL_GAIN_SHIFT) | (R_Amp_Code[b1Row][b1Col] << AWMF_RF1_CHANNEL_GAIN_SHIFT) |
					   (R_phi_A_Code[b4Row][b4Col] << AWMF_RF4_CHANNEL_PHASE_SHIFT) | (R_phi_A_Code[b3Row][b3Col] << AWMF_RF3_CHANNEL_PHASE_SHIFT) |
					   (R_phi_A_Code[b2Row][b2Col] << AWMF_RF2_CHANNEL_PHASE_SHIFT) | (R_phi_A_Code[b1Row][b1Col] << AWMF_RF1_CHANNEL_PHASE_SHIFT);
		data_chain->rfB_data.chip_data[idx].data_high = data_temp_H;
		data_chain->rfB_data.chip_data[idx].data_low  = data_temp_L;
	}
}

// Transmitter wave control code transmission
static void tx_bk_tran_func(void)
{
	memset(&tx_data_u12_chain, 0, sizeof(tx_data_u12_chain));
	memset(&tx_data_u16_chain, 0, sizeof(tx_data_u16_chain));

	tx_data_u12_chain.rfA_data.chain_id = AWMF_0165_TX_CHAIN_U12_U10_U09_U11;
	tx_data_u12_chain.rfB_data.chain_id = AWMF_0165_TX_CHAIN_U12_U10_U09_U11;
	bk_tx_tran_data(tx_chan_u12_idx, tx_u12_chip_id, &tx_data_u12_chain, tx_com_Amp_Code);
	tx_data_u16_chain.rfA_data.chain_id = AWMF_0165_TX_CHAIN_U16_U14_U13_U15;
	tx_data_u16_chain.rfB_data.chain_id = AWMF_0165_TX_CHAIN_U16_U14_U13_U15;
	bk_tx_tran_data(tx_chan_u16_idx, tx_u16_chip_id, &tx_data_u16_chain, tx_com_Amp_Code);

	awmf_0165_drv_chain_serial_write(&tx_data_u12_chain.rfA_data);
	awmf_0165_drv_chain_serial_write(&tx_data_u12_chain.rfB_data);
	awmf_0165_drv_chain_serial_write(&tx_data_u16_chain.rfA_data);
	awmf_0165_drv_chain_serial_write(&tx_data_u16_chain.rfB_data);

}

// Receiver wave control code transmission
static void rx_bk_tran_func(void)
{
	memset(&rx_data_u19_chain, 0, sizeof(rx_data_u19_chain));
	memset(&rx_data_u23_chain, 0, sizeof(rx_data_u23_chain));

	rx_data_u19_chain.rfA_data.chain_id = AWMF_0165_RX_CHAIN_U19_U17_U18_U20;
	rx_data_u19_chain.rfB_data.chain_id = AWMF_0165_RX_CHAIN_U19_U17_U18_U20;
	bk_rx_tran_data(rx_chan_u19_idx, rx_u19_chip_id, &rx_data_u19_chain, rx_u19_Amp_Code);
	rx_data_u23_chain.rfA_data.chain_id = AWMF_0165_RX_CHAIN_U23_U21_U22_U24;
	rx_data_u23_chain.rfB_data.chain_id = AWMF_0165_RX_CHAIN_U23_U21_U22_U24;
	bk_rx_tran_data(rx_chan_u23_idx, rx_u23_chip_id, &rx_data_u23_chain, rx_u23_Amp_Code);

	awmf_0165_drv_chain_serial_write(&rx_data_u19_chain.rfA_data);
	awmf_0165_drv_chain_serial_write(&rx_data_u19_chain.rfB_data);
	awmf_0165_drv_chain_serial_write(&rx_data_u23_chain.rfA_data);
	awmf_0165_drv_chain_serial_write(&rx_data_u23_chain.rfB_data);
}

// Wave control code calculation
void bk_cak_func(uint8_t frePoint, int16_t theta_az, int16_t phi_el)
{
	float T_phi_A[4][16];
	float R_phi_A[4][16];
	float phi_delta_A = 0, phi_delta_B = 0;

	int32_t ii, jj;
	int32_t addr_A;
	int32_t temp_A = 0;

    HostCtlCmd.FreDot = frePoint;
	HostCtlCmd.Beam_AZ_SIN = sinf(theta_az * M_PI/180) * 32768.0;
	HostCtlCmd.Beam_EL_SIN = sinf(phi_el * M_PI/180) * 32768.0;
	HostCtlCmd.Beam_EL_COS = cosf(phi_el * M_PI/180) * 32768.0;

	for(ii = 0; ii < 4; ii++)
	{
		phi_delta_B = 1.2 * dy * (HostCtlCmd.FreDot * 65 + 24000) * (HostCtlCmd.Beam_EL_SIN) / 32768.0;
		for(jj = 0; jj < 16; jj++)
		{
			phi_delta_A = 1.2 * dx * (HostCtlCmd.FreDot * 65 + 24000) * (HostCtlCmd.Beam_EL_COS) / 32768.0 * (HostCtlCmd.Beam_AZ_SIN) / 32768.0;
			// Calculate the transmit phase value
			addr_A = (HostCtlCmd.FreDot) * 128 + 16 * ii + jj;
			temp_A =  INIT_PHASE_DATA[addr_A];
			T_phi_A[ii][jj] = (phi_delta_A * jj + phi_delta_B * ii + 5.625 * temp_A) - ((int)((phi_delta_A * jj + phi_delta_B * ii + 5.625 * temp_A) / 360)) * 360;

			if(T_phi_A[ii][jj] < 0)
			{
				T_phi_A[ii][jj] = 360.0+T_phi_A[ii][jj];
			}
			// Calculate the receive phase value
			temp_A = INIT_PHASE_DATA[addr_A+64];
			R_phi_A[ii][jj] = (phi_delta_A * jj + phi_delta_B * ii + 5.625 * temp_A) - ((int)((phi_delta_A * jj + phi_delta_B * ii + 5.625 * temp_A) / 360)) * 360;

			if(R_phi_A[ii][jj] < 0)
			{
				R_phi_A[ii][jj] = 360.0 + R_phi_A[ii][jj];
			}
		}
	}

	// Calculate the transmitter and receiver transmitter amplitude codes
	for(ii = 0; ii < 4; ii++)
	{
		for(jj = 0; jj < 16; jj++)
		{
			T_phi_A_Code[ii][jj] = ((char)(T_phi_A[ii][jj] / 5.625 + 0.5)) & 0x3F;
			R_phi_A_Code[ii][jj] = ((char)(R_phi_A[ii][jj] / 5.625 + 0.5)) & 0x3F;
		}
	}

	// Calculate transmitter and receiver amplitude code (just take it out from the file)
	for(ii = 0; ii < 4; ii++)
	{
		if (amp_val.isSetValid)
		{
			tx_com_Amp_Code[ii] = (amp_val.comVal & AMP_CODE_VALID_VALUE_MASK) << AWMF_COMMON_CHANNEL_GAIN_SHIFT;
		}

		for(jj = 0; jj < 16; jj++)
		{
			addr_A = (HostCtlCmd.FreDot) * 128 + 16 * ii + jj;
			if (amp_val.isSetValid)
			{
				T_Amp_Code[ii][jj] = INIT_AMP_DATA[addr_A] + amp_val.chnVal;
			}
			else
			{
				T_Amp_Code[ii][jj] = INIT_AMP_DATA[addr_A];
			}
			R_Amp_Code[ii][jj] = INIT_AMP_DATA[addr_A+64];
		}
	}
}


// Calculate the phase code and attenuation code of the two-dimensional active array
void bk_tran_func(bool isTx)
{

	channel_enable_setting(isTx);

	if (isTx)		// Tx
	{
		tx_bk_tran_func();
	}
	else			// Rx
	{
		rx_bk_tran_func();
	}

}


/*******************************************************************************************
 * beam control point querying module
 ******************************************************************************************/
ret_code_t get_two_channel_phase_value(uint8_t *value0,  uint8_t *value1)
{
	ret_code_t ret = RET_OK;
	uint32_t regValhigh = 0, reg_val_low0,  reg_val_low1;

	ret = awmf_0165_drv_reg_get(AWMF_0165_TX_U16, AWMF_TX_A_CHANNEL_REGISTER, &regValhigh, &reg_val_low0);	// [0,1], U16-RF4A
	ret = awmf_0165_drv_reg_get(AWMF_0165_TX_U16, AWMF_TX_B_CHANNEL_REGISTER, &regValhigh, &reg_val_low1);	// [1,0], U16-RF3B

	*value0 = (reg_val_low0 & AWMF_RF4_CHANNEL_PHASE_MASK) >> AWMF_RF4_CHANNEL_PHASE_SHIFT;
	*value1 = (reg_val_low1 & AWMF_RF3_CHANNEL_PHASE_MASK) >> AWMF_RF3_CHANNEL_PHASE_SHIFT;

	return ret;

}

void calculate_angle(float delta_A, float delta_B, int16_t *theta_az, int16_t *phi_el)
{
	host_ctl_command_t ctl_cmd;
	memset(&ctl_cmd, 0, sizeof(ctl_cmd));

	ctl_cmd.Beam_EL_SIN = (int16_t)(delta_B * 32768.0 / 1.2 / dy / 24000);
	*phi_el = (int16_t)(asinf(ctl_cmd.Beam_EL_SIN));
	ctl_cmd.Beam_EL_COS = (int16_t)(cosf(ctl_cmd.Beam_EL_SIN));
	ctl_cmd.Beam_AZ_SIN = (int16_t)(delta_A * 32768.0 * 32768.0 / 1.2 / dx / 24000 / ctl_cmd.Beam_EL_COS);
	*theta_az = (int16_t)(asinf(ctl_cmd.Beam_AZ_SIN));

}

ret_code_t beam_control_qureying(int16_t *theta_azimuth, int16_t *phi_elevation)
{
	ret_code_t ret = RET_OK;
	uint8_t chn_0_1_val = 0,  chn_1_0_val = 0;
	float chn_0_1_phase = 0.0, chn_1_0_phase = 0.0, phi_delta_A = 0.0, phi_delta_B = 0.0;

	ret = get_two_channel_phase_value(&chn_0_1_val, &chn_1_0_val);
	chn_0_1_phase = chn_0_1_val * 5.625 + 0.5;
	chn_1_0_phase = chn_1_0_val * 5.625 + 0.5;

	phi_delta_A = chn_0_1_phase - 5.625 * CHANNEL_0_1_INIT_VALUE;
	phi_delta_B = chn_1_0_phase - 5.625 * CHANNEL_1_0_INIT_VALUE;

	calculate_angle(phi_delta_A, phi_delta_B, theta_azimuth, phi_elevation);

	return ret;
}
