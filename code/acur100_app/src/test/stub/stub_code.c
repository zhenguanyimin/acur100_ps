
#include "stub_code.h"

#define TX_CHN_ROW_CNT (4)
#define TX_CHN_COL_CNT (16)
#define RX_CHN_ROW_CNT (4)
#define RX_CHN_COL_CNT (16)

typedef struct stub_code_sys_info {
	uint32_t freq_num;
	float az_deg;
	float el_deg;
} stub_code_sys_info_t;

typedef struct stub_code_tx_chn_info {
	uint32_t chn_state;
	uint32_t phase_code;
} stub_code_tx_chn_info_t;

typedef struct stub_code_rx_chn_info {
	uint32_t chn_state;
	uint32_t phase_code;
	uint32_t amp_com_code;
	uint32_t amp_chn_code;
} stub_code_rx_chn_info_t;

typedef struct stub_code_mcb {
	stub_code_sys_info_t sys_info;
	stub_code_tx_chn_info_t tx_chn_info[TX_CHN_ROW_CNT][TX_CHN_COL_CNT];
	stub_code_rx_chn_info_t rx_chn_info[RX_CHN_ROW_CNT][RX_CHN_COL_CNT];
} stub_code_mcb_t;

static stub_code_mcb_t g_stub_mcb = {0};

ret_t stub_code_freq_num_set(uint32_t freq_num)
{
	g_stub_mcb.sys_info.freq_num = freq_num;

	return STUB_RET_OK;
}

ret_t stub_code_freq_num_get(uint32_t *freq_num)
{
	*freq_num = g_stub_mcb.sys_info.freq_num;

	return STUB_RET_OK;
}

ret_t stub_code_tx_chn_set(uint32_t row_num, uint32_t col_num, uint32_t chn_state, uint32_t phase_code)
{
	g_stub_mcb.tx_chn_info[row_num][col_num].chn_state = chn_state;
	g_stub_mcb.tx_chn_info[row_num][col_num].phase_code = phase_code;

	return STUB_RET_OK;
}

ret_t stub_code_tx_chn_get(uint32_t row_num, uint32_t col_num, uint32_t *chn_state, uint32_t *phase_code)
{
	*chn_state = g_stub_mcb.tx_chn_info[row_num][col_num].chn_state;
	*phase_code = g_stub_mcb.tx_chn_info[row_num][col_num].phase_code;

	return STUB_RET_OK;
}

ret_t stub_code_rx_chn_set(uint32_t row_num, uint32_t col_num, uint32_t chn_state, uint32_t phase_code, uint32_t amp_com_code, uint32_t amp_chn_code)
{
	g_stub_mcb.rx_chn_info[row_num][col_num].chn_state = chn_state;
	g_stub_mcb.rx_chn_info[row_num][col_num].phase_code = phase_code;
	g_stub_mcb.rx_chn_info[row_num][col_num].amp_com_code = amp_com_code;
	g_stub_mcb.rx_chn_info[row_num][col_num].amp_chn_code = amp_chn_code;

	return STUB_RET_OK;
}

ret_t stub_code_rx_chn_get(uint32_t row_num, uint32_t col_num, uint32_t *chn_state, uint32_t *phase_code, uint32_t *amp_com_code, uint32_t *amp_chn_code)
{
	*chn_state = g_stub_mcb.rx_chn_info[row_num][col_num].chn_state;
	*phase_code = g_stub_mcb.rx_chn_info[row_num][col_num].phase_code;
	*amp_com_code = g_stub_mcb.rx_chn_info[row_num][col_num].amp_com_code;
	*amp_chn_code = g_stub_mcb.rx_chn_info[row_num][col_num].amp_chn_code;

	return STUB_RET_OK;
}

ret_t stub_code_close_all_chn(void)
{
	uint32_t row_num, col_num;

	for (row_num = 0; row_num < TX_CHN_ROW_CNT; row_num++)
	{
		for (col_num = 0; col_num < TX_CHN_COL_CNT; col_num++)
		{
			g_stub_mcb.tx_chn_info[row_num][col_num].chn_state = 0;
			g_stub_mcb.tx_chn_info[row_num][col_num].phase_code = 0;
		}
	}

	for (row_num = 0; row_num < RX_CHN_ROW_CNT; row_num++)
	{
		for (col_num = 0; col_num < RX_CHN_COL_CNT; col_num++)
		{
			g_stub_mcb.rx_chn_info[row_num][col_num].chn_state = 0;
			g_stub_mcb.rx_chn_info[row_num][col_num].phase_code = 0;
			g_stub_mcb.rx_chn_info[row_num][col_num].amp_com_code = 0;
			g_stub_mcb.rx_chn_info[row_num][col_num].amp_chn_code = 0;
		}
	}

	return STUB_RET_OK;
}

ret_t stub_code_tx_ant_test(uint32_t freq_num, float az_deg, float el_deg)
{
	uint32_t row_num, col_num;

	g_stub_mcb.sys_info.freq_num = freq_num;
	g_stub_mcb.sys_info.az_deg = az_deg;
	g_stub_mcb.sys_info.el_deg = el_deg;

	for (row_num = 0; row_num < TX_CHN_ROW_CNT; row_num++)
	{
		for (col_num = 0; col_num < TX_CHN_COL_CNT; col_num++)
		{
			g_stub_mcb.tx_chn_info[row_num][col_num].chn_state = 1;
			// g_stub_mcb.tx_chn_info[row_num][col_num].phase_code = 0;
		}
	}

	return STUB_RET_OK;
}

ret_t stub_code_rx_ant_test(uint32_t freq_num, float az_deg, float el_deg)
{
	uint32_t row_num, col_num;

	g_stub_mcb.sys_info.freq_num = freq_num;
	g_stub_mcb.sys_info.az_deg = az_deg;
	g_stub_mcb.sys_info.el_deg = el_deg;

	for (row_num = 0; row_num < RX_CHN_ROW_CNT; row_num++)
	{
		for (col_num = 0; col_num < RX_CHN_COL_CNT; col_num++)
		{
			g_stub_mcb.rx_chn_info[row_num][col_num].chn_state = 1;
			// g_stub_mcb.rx_chn_info[row_num][col_num].phase_code = 0;
			// g_stub_mcb.rx_chn_info[row_num][col_num].amp_com_code = 0;
			// g_stub_mcb.rx_chn_info[row_num][col_num].amp_chn_code = 0;
		}
	}

	return STUB_RET_OK;
}
