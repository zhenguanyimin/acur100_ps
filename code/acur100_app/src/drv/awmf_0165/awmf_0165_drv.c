
#include "awmf_0165_drv.h"
#include "awmf_0165_chip.h"
#include "awmf_0165_io.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#define AWMF_CHN_INFO_FAST_READ (0)

typedef struct awmf_0165_ant_to_chip_chn_map {
	uint32_t ant_row_id;
	uint32_t ant_col_id;
	uint32_t chip_id; // ref awmf_0165_chip_id_t
	uint32_t chn_id; // ref awmf_0165_chn_id_t
} awmf_0165_ant_to_chip_chn_map_t;

typedef struct awmf_0165_drv_mcb {
	bool inited;

	awmf_0165_chn_info_t tx_chn_info[RADAR_TX_CHN_ROW_CNT][RADAR_TX_CHN_COL_CNT];
	awmf_0165_chn_info_t rx_chn_info[RADAR_RX_CHN_ROW_CNT][RADAR_RX_CHN_COL_CNT];
} awmf_0165_drv_mcb_t;

STATIC awmf_0165_ant_to_chip_chn_map_t g_tx_ant_map_tbl[] = {
	{0, 0, AWMF_0165_TX_U16, AWMF_0165_CHN_4_B},
	{0, 1, AWMF_0165_TX_U16, AWMF_0165_CHN_4_A},
	{0, 2, AWMF_0165_TX_U16, AWMF_0165_CHN_1_A},
	{0, 3, AWMF_0165_TX_U16, AWMF_0165_CHN_1_B},
	{0, 4, AWMF_0165_TX_U15, AWMF_0165_CHN_4_B},
	{0, 5, AWMF_0165_TX_U15, AWMF_0165_CHN_4_A},
	{0, 6, AWMF_0165_TX_U15, AWMF_0165_CHN_1_A},
	{0, 7, AWMF_0165_TX_U15, AWMF_0165_CHN_1_B},
	{0, 8, AWMF_0165_TX_U11, AWMF_0165_CHN_4_B},
	{0, 9, AWMF_0165_TX_U11, AWMF_0165_CHN_4_A},
	{0, 10, AWMF_0165_TX_U11, AWMF_0165_CHN_1_A},
	{0, 11, AWMF_0165_TX_U11, AWMF_0165_CHN_1_B},
	{0, 12, AWMF_0165_TX_U12, AWMF_0165_CHN_4_B},
	{0, 13, AWMF_0165_TX_U12, AWMF_0165_CHN_4_A},
	{0, 14, AWMF_0165_TX_U12, AWMF_0165_CHN_1_A},
	{0, 15, AWMF_0165_TX_U12, AWMF_0165_CHN_1_B},
	{1, 0, AWMF_0165_TX_U16, AWMF_0165_CHN_3_B},
	{1, 1, AWMF_0165_TX_U16, AWMF_0165_CHN_3_A},
	{1, 2, AWMF_0165_TX_U16, AWMF_0165_CHN_2_A},
	{1, 3, AWMF_0165_TX_U16, AWMF_0165_CHN_2_B},
	{1, 4, AWMF_0165_TX_U15, AWMF_0165_CHN_3_B},
	{1, 5, AWMF_0165_TX_U15, AWMF_0165_CHN_3_A},
	{1, 6, AWMF_0165_TX_U15, AWMF_0165_CHN_2_A},
	{1, 7, AWMF_0165_TX_U15, AWMF_0165_CHN_2_B},
	{1, 8, AWMF_0165_TX_U11, AWMF_0165_CHN_3_B},
	{1, 9, AWMF_0165_TX_U11, AWMF_0165_CHN_3_A},
	{1, 10, AWMF_0165_TX_U11, AWMF_0165_CHN_2_A},
	{1, 11, AWMF_0165_TX_U11, AWMF_0165_CHN_2_B},
	{1, 12, AWMF_0165_TX_U12, AWMF_0165_CHN_3_B},
	{1, 13, AWMF_0165_TX_U12, AWMF_0165_CHN_3_A},
	{1, 14, AWMF_0165_TX_U12, AWMF_0165_CHN_2_A},
	{1, 15, AWMF_0165_TX_U12, AWMF_0165_CHN_2_B},
	{2, 0, AWMF_0165_TX_U13, AWMF_0165_CHN_4_B},
	{2, 1, AWMF_0165_TX_U13, AWMF_0165_CHN_4_A},
	{2, 2, AWMF_0165_TX_U13, AWMF_0165_CHN_1_A},
	{2, 3, AWMF_0165_TX_U13, AWMF_0165_CHN_1_B},
	{2, 4, AWMF_0165_TX_U14, AWMF_0165_CHN_4_B},
	{2, 5, AWMF_0165_TX_U14, AWMF_0165_CHN_4_A},
	{2, 6, AWMF_0165_TX_U14, AWMF_0165_CHN_1_A},
	{2, 7, AWMF_0165_TX_U14, AWMF_0165_CHN_1_B},
	{2, 8, AWMF_0165_TX_U10, AWMF_0165_CHN_4_B},
	{2, 9, AWMF_0165_TX_U10, AWMF_0165_CHN_4_A},
	{2, 10, AWMF_0165_TX_U10, AWMF_0165_CHN_1_A},
	{2, 11, AWMF_0165_TX_U10, AWMF_0165_CHN_1_B},
	{2, 12, AWMF_0165_TX_U09, AWMF_0165_CHN_4_B},
	{2, 13, AWMF_0165_TX_U09, AWMF_0165_CHN_4_A},
	{2, 14, AWMF_0165_TX_U09, AWMF_0165_CHN_1_A},
	{2, 15, AWMF_0165_TX_U09, AWMF_0165_CHN_1_B},
	{3, 0, AWMF_0165_TX_U13, AWMF_0165_CHN_3_B},
	{3, 1, AWMF_0165_TX_U13, AWMF_0165_CHN_3_A},
	{3, 2, AWMF_0165_TX_U13, AWMF_0165_CHN_2_A},
	{3, 3, AWMF_0165_TX_U13, AWMF_0165_CHN_2_B},
	{3, 4, AWMF_0165_TX_U14, AWMF_0165_CHN_3_B},
	{3, 5, AWMF_0165_TX_U14, AWMF_0165_CHN_3_A},
	{3, 6, AWMF_0165_TX_U14, AWMF_0165_CHN_2_A},
	{3, 7, AWMF_0165_TX_U14, AWMF_0165_CHN_2_B},
	{3, 8, AWMF_0165_TX_U10, AWMF_0165_CHN_3_B},
	{3, 9, AWMF_0165_TX_U10, AWMF_0165_CHN_3_A},
	{3, 10, AWMF_0165_TX_U10, AWMF_0165_CHN_2_A},
	{3, 11, AWMF_0165_TX_U10, AWMF_0165_CHN_2_B},
	{3, 12, AWMF_0165_TX_U09, AWMF_0165_CHN_3_B},
	{3, 13, AWMF_0165_TX_U09, AWMF_0165_CHN_3_A},
	{3, 14, AWMF_0165_TX_U09, AWMF_0165_CHN_2_A},
	{3, 15, AWMF_0165_TX_U09, AWMF_0165_CHN_2_B},
};

STATIC awmf_0165_ant_to_chip_chn_map_t g_rx_ant_map_tbl[] = {
	{0, 0, AWMF_0165_RX_U22, AWMF_0165_CHN_4_B},
	{0, 1, AWMF_0165_RX_U22, AWMF_0165_CHN_4_A},
	{0, 2, AWMF_0165_RX_U22, AWMF_0165_CHN_1_A},
	{0, 3, AWMF_0165_RX_U22, AWMF_0165_CHN_1_B},
	{0, 4, AWMF_0165_RX_U21, AWMF_0165_CHN_4_B},
	{0, 5, AWMF_0165_RX_U21, AWMF_0165_CHN_4_A},
	{0, 6, AWMF_0165_RX_U21, AWMF_0165_CHN_1_A},
	{0, 7, AWMF_0165_RX_U21, AWMF_0165_CHN_1_B},
	{0, 8, AWMF_0165_RX_U18, AWMF_0165_CHN_4_B},
	{0, 9, AWMF_0165_RX_U18, AWMF_0165_CHN_4_A},
	{0, 10, AWMF_0165_RX_U18, AWMF_0165_CHN_1_A},
	{0, 11, AWMF_0165_RX_U18, AWMF_0165_CHN_1_B},
	{0, 12, AWMF_0165_RX_U17, AWMF_0165_CHN_4_B},
	{0, 13, AWMF_0165_RX_U17, AWMF_0165_CHN_4_A},
	{0, 14, AWMF_0165_RX_U17, AWMF_0165_CHN_1_A},
	{0, 15, AWMF_0165_RX_U17, AWMF_0165_CHN_1_B},
	{1, 0, AWMF_0165_RX_U22, AWMF_0165_CHN_3_B},
	{1, 1, AWMF_0165_RX_U22, AWMF_0165_CHN_3_A},
	{1, 2, AWMF_0165_RX_U22, AWMF_0165_CHN_2_A},
	{1, 3, AWMF_0165_RX_U22, AWMF_0165_CHN_2_B},
	{1, 4, AWMF_0165_RX_U21, AWMF_0165_CHN_3_B},
	{1, 5, AWMF_0165_RX_U21, AWMF_0165_CHN_3_A},
	{1, 6, AWMF_0165_RX_U21, AWMF_0165_CHN_2_A},
	{1, 7, AWMF_0165_RX_U21, AWMF_0165_CHN_2_B},
	{1, 8, AWMF_0165_RX_U18, AWMF_0165_CHN_3_B},
	{1, 9, AWMF_0165_RX_U18, AWMF_0165_CHN_3_A},
	{1, 10, AWMF_0165_RX_U18, AWMF_0165_CHN_2_A},
	{1, 11, AWMF_0165_RX_U18, AWMF_0165_CHN_2_B},
	{1, 12, AWMF_0165_RX_U17, AWMF_0165_CHN_3_B},
	{1, 13, AWMF_0165_RX_U17, AWMF_0165_CHN_3_A},
	{1, 14, AWMF_0165_RX_U17, AWMF_0165_CHN_2_A},
	{1, 15, AWMF_0165_RX_U17, AWMF_0165_CHN_2_B},
	{2, 0, AWMF_0165_RX_U23, AWMF_0165_CHN_4_B},
	{2, 1, AWMF_0165_RX_U23, AWMF_0165_CHN_4_A},
	{2, 2, AWMF_0165_RX_U23, AWMF_0165_CHN_1_A},
	{2, 3, AWMF_0165_RX_U23, AWMF_0165_CHN_1_B},
	{2, 4, AWMF_0165_RX_U24, AWMF_0165_CHN_4_B},
	{2, 5, AWMF_0165_RX_U24, AWMF_0165_CHN_4_A},
	{2, 6, AWMF_0165_RX_U24, AWMF_0165_CHN_1_A},
	{2, 7, AWMF_0165_RX_U24, AWMF_0165_CHN_1_B},
	{2, 8, AWMF_0165_RX_U19, AWMF_0165_CHN_4_B},
	{2, 9, AWMF_0165_RX_U19, AWMF_0165_CHN_4_A},
	{2, 10, AWMF_0165_RX_U19, AWMF_0165_CHN_1_A},
	{2, 11, AWMF_0165_RX_U19, AWMF_0165_CHN_1_B},
	{2, 12, AWMF_0165_RX_U20, AWMF_0165_CHN_4_B},
	{2, 13, AWMF_0165_RX_U20, AWMF_0165_CHN_4_A},
	{2, 14, AWMF_0165_RX_U20, AWMF_0165_CHN_1_A},
	{2, 15, AWMF_0165_RX_U20, AWMF_0165_CHN_1_B},
	{3, 0, AWMF_0165_RX_U23, AWMF_0165_CHN_3_B},
	{3, 1, AWMF_0165_RX_U23, AWMF_0165_CHN_3_A},
	{3, 2, AWMF_0165_RX_U23, AWMF_0165_CHN_2_A},
	{3, 3, AWMF_0165_RX_U23, AWMF_0165_CHN_2_B},
	{3, 4, AWMF_0165_RX_U24, AWMF_0165_CHN_3_B},
	{3, 5, AWMF_0165_RX_U24, AWMF_0165_CHN_3_A},
	{3, 6, AWMF_0165_RX_U24, AWMF_0165_CHN_2_A},
	{3, 7, AWMF_0165_RX_U24, AWMF_0165_CHN_2_B},
	{3, 8, AWMF_0165_RX_U19, AWMF_0165_CHN_3_B},
	{3, 9, AWMF_0165_RX_U19, AWMF_0165_CHN_3_A},
	{3, 10, AWMF_0165_RX_U19, AWMF_0165_CHN_2_A},
	{3, 11, AWMF_0165_RX_U19, AWMF_0165_CHN_2_B},
	{3, 12, AWMF_0165_RX_U20, AWMF_0165_CHN_3_B},
	{3, 13, AWMF_0165_RX_U20, AWMF_0165_CHN_3_A},
	{3, 14, AWMF_0165_RX_U20, AWMF_0165_CHN_2_A},
	{3, 15, AWMF_0165_RX_U20, AWMF_0165_CHN_2_B},
};

STATIC awmf_0165_drv_mcb_t g_awmf_0165_drv_mcb;

PRIVATE ret_code_t awmf_0165_drv_get_tx_chn_id_by_ant_id(awmf_0165_ant_to_chip_chn_map_t *data)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;

	for (i = 0; i < ARRAY_SIZE(g_tx_ant_map_tbl); i++)
	{
		if ((data->ant_row_id == g_tx_ant_map_tbl[i].ant_row_id) && (data->ant_col_id == g_tx_ant_map_tbl[i].ant_col_id))
		{
			data->chip_id = g_tx_ant_map_tbl[i].chip_id;
		    data->chn_id = g_tx_ant_map_tbl[i].chn_id;
		    break;
		}
	}

	if (i >= ARRAY_SIZE(g_tx_ant_map_tbl))
	{
		ret = RET_GENERAL_ERR;
	}

	return ret;
}

PRIVATE ret_code_t awmf_0165_drv_get_rx_chn_id_by_ant_id(awmf_0165_ant_to_chip_chn_map_t *data)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;

	for (i = 0; i < ARRAY_SIZE(g_rx_ant_map_tbl); i++)
	{
		if ((data->ant_row_id == g_rx_ant_map_tbl[i].ant_row_id) && (data->ant_col_id == g_rx_ant_map_tbl[i].ant_col_id))
		{
			data->chip_id = g_rx_ant_map_tbl[i].chip_id;
		    data->chn_id = g_rx_ant_map_tbl[i].chn_id;
		    break;
		}
	}

	if (i >= ARRAY_SIZE(g_rx_ant_map_tbl))
	{
		ret = RET_GENERAL_ERR;
	}

	return ret;
}

ret_code_t awmf_0165_drv_init(void)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;
	awmf_0165_chip_id_t chip_ids[] = {
		AWMF_0165_TX_U08,
		AWMF_0165_TX_U12, AWMF_0165_TX_U10, AWMF_0165_TX_U09, AWMF_0165_TX_U11,
		AWMF_0165_TX_U16, AWMF_0165_TX_U14, AWMF_0165_TX_U13, AWMF_0165_TX_U15,
		AWMF_0165_RX_U19, AWMF_0165_RX_U17, AWMF_0165_RX_U18, AWMF_0165_RX_U20,
		AWMF_0165_RX_U23, AWMF_0165_RX_U21, AWMF_0165_RX_U22, AWMF_0165_RX_U24,
	};

	if (g_awmf_0165_drv_mcb.inited)
	{
		goto out;
	}

	ret = awmf_0165_drv_io_init();
	if (ret != RET_OK)
	{
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(chip_ids); i++)
	{
		ret = awmf_0165_drv_chip_init(chip_ids[i]);
		if (ret != RET_OK)
		{
			goto out;
		}
		vTaskDelay(1);

		ret = awmf_0165_drv_chip_reset(chip_ids[i]);
		if (ret != RET_OK)
		{
			goto out;
		}
		vTaskDelay(1);
	}

	g_awmf_0165_drv_mcb.inited = true;

out:
	return ret;
}

ret_code_t awmf_0165_drv_open(void)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;
	awmf_0165_chip_id_t tx_chip_ids[] = {
		AWMF_0165_TX_U08,
		AWMF_0165_TX_U12, AWMF_0165_TX_U10, AWMF_0165_TX_U09, AWMF_0165_TX_U11,
		AWMF_0165_TX_U16, AWMF_0165_TX_U14, AWMF_0165_TX_U13, AWMF_0165_TX_U15,
	};
	awmf_0165_chip_id_t rx_chip_ids[] = {
		AWMF_0165_RX_U19, AWMF_0165_RX_U17, AWMF_0165_RX_U18, AWMF_0165_RX_U20,
		AWMF_0165_RX_U23, AWMF_0165_RX_U21, AWMF_0165_RX_U22, AWMF_0165_RX_U24,
	};

	for (i = 0; i < ARRAY_SIZE(tx_chip_ids); i++)
	{
		ret = awmf_0165_drv_io_chip_ctrl(tx_chip_ids[i], AWMF_0165_OPM_TX_MODE);
		if (ret != RET_OK)
		{
			goto out;
		}
	}

	for (i = 0; i < ARRAY_SIZE(rx_chip_ids); i++)
	{
		ret = awmf_0165_drv_io_chip_ctrl(rx_chip_ids[i], AWMF_0165_OPM_RX_MODE);
		if (ret != RET_OK)
		{
			goto out;
		}
	}

	// TODO: open U8 channels which is needed by others TX chips

out:
	return ret;
}

ret_code_t awmf_0165_drv_close(void)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;
	awmf_0165_chip_id_t chip_ids[] = {
		AWMF_0165_TX_U08,
		AWMF_0165_TX_U12, AWMF_0165_TX_U10, AWMF_0165_TX_U09, AWMF_0165_TX_U11,
		AWMF_0165_TX_U16, AWMF_0165_TX_U14, AWMF_0165_TX_U13, AWMF_0165_TX_U15,
		AWMF_0165_RX_U19, AWMF_0165_RX_U17, AWMF_0165_RX_U18, AWMF_0165_RX_U20,
		AWMF_0165_RX_U23, AWMF_0165_RX_U21, AWMF_0165_RX_U22, AWMF_0165_RX_U24,
	};

	for (i = 0; i < ARRAY_SIZE(chip_ids); i++)
	{
		ret = awmf_0165_drv_io_chip_ctrl(chip_ids[i], AWMF_0165_OPM_STANDBY);
		if (ret != RET_OK)
		{
			goto out;
		}

		ret = awmf_0165_drv_chip_reset(chip_ids[i]);
		if (ret != RET_OK)
		{
			goto out;
		}
	}

out:
	return ret;
}

PRIVATE bool awmf_0165_drv_is_chn_info_valid(awmf_0165_chn_info_t *chn_info)
{
	bool ret = true;

	do {
		if (chn_info == NULL)
		{
			ret = false;
			break;
		}

		if (!IS_0165_CHN_STATE_VALID(chn_info->chn_state))
		{
			ret = false;
			break;
		}

		if (!IS_0165_PHASE_CODE_VALID(chn_info->phase_code))
		{
			ret = false;
			break;
		}

		if (!IS_0165_AMP_COM_CODE_VALID(chn_info->amp_com_code))
		{
			ret = false;
			break;
		}

		if (!IS_0165_AMP_CHN_CODE_VALID(chn_info->amp_chn_code))
		{
			ret = false;
			break;
		}
	} while (0);

	return ret;
}

ret_code_t awmf_0165_drv_tx_chn_set(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info)
{
	ret_code_t ret = RET_OK;
	awmf_0165_ant_to_chip_chn_map_t map_data = {0};
	awmf_0165_dev_chip_data_t chip_data = {0};
	uint64_t reg_val = 0;
	uint32_t old_data_high = 0, old_data_low = 0;

	if ((row_num > RADAR_TX_CHN_ROW_CNT) || (col_num > RADAR_TX_CHN_COL_CNT))
	{
		ret = RET_INVALID_PARAM;
		goto out;
	}

	if (!awmf_0165_drv_is_chn_info_valid(chn_info))
	{
		ret = RET_INVALID_PARAM;
		goto out;
	}

	map_data.ant_row_id = row_num;
	map_data.ant_col_id = col_num;
	ret = awmf_0165_drv_get_tx_chn_id_by_ant_id(&map_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	chip_data.chip_id = map_data.chip_id;
	chip_data.odd_parity_en = 0;
	if (map_data.chn_id < AWMF_0165_CHN_1_B)
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_TX_A;
	}
	else
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_TX_B;
		map_data.chn_id -= AWMF_0165_CHN_1_B; // re-index the channel id
	}

	// read back the old data firstly
	ret = awmf_0165_drv_reg_get(chip_data.chip_id, chip_data.addr_offset, &old_data_high, &old_data_low);
	if (ret != RET_OK)
	{
		goto out;
	}
	reg_val = ((uint64_t)old_data_high << 32) + old_data_low;

	// set channel state
	if (chn_info->chn_state == 0)
	{
		reg_val |= (uint64_t)1 << (44 + map_data.chn_id);
	}
	else
	{
		reg_val &= ~((uint64_t)1 << (44 + map_data.chn_id));
	}
	// set channel phase
	reg_val &= ~((uint64_t)((1 << 6) - 1) << (map_data.chn_id * 6));
	reg_val |= (chn_info->phase_code << (map_data.chn_id * 6));
	// set channel attenuation
	reg_val &= ~((uint64_t)((1 << 4) - 1) << (map_data.chn_id * 4 + 24));
	reg_val |= ((uint64_t)chn_info->amp_chn_code << (map_data.chn_id * 4 + 24));
	// set common arm attenuation
	reg_val &= ~((uint64_t)((1 << 4) - 1) << (40));
	reg_val |= ((uint64_t)chn_info->amp_com_code << (40));

	chip_data.data_high = reg_val >> 32;
	chip_data.data_low = reg_val & 0xFFFFFFFF;
	ret = awmf_0165_drv_chip_serial_write(&chip_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].chn_state = chn_info->chn_state;
	g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].phase_code = chn_info->phase_code;
	g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].amp_com_code = chn_info->amp_com_code;
	g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].amp_chn_code = chn_info->amp_chn_code;

out:
	return ret;
}

ret_code_t awmf_0165_drv_tx_chn_get(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info)
{
	ret_code_t ret = RET_OK;
#if !AWMF_CHN_INFO_FAST_READ
	awmf_0165_ant_to_chip_chn_map_t map_data = {0};
	awmf_0165_dev_chip_data_t chip_data = {0};
	uint32_t data_high = 0, data_low = 0;
	uint64_t reg_val = 0;
#endif /* AWMF_CHN_INFO_FAST_READ */

	if ((row_num > RADAR_TX_CHN_ROW_CNT) || (col_num > RADAR_TX_CHN_COL_CNT) || (chn_info == NULL))
	{
		ret = RET_INVALID_PARAM;
		goto out;
	}

#if !AWMF_CHN_INFO_FAST_READ
	map_data.ant_row_id = row_num;
	map_data.ant_col_id = col_num;
	ret = awmf_0165_drv_get_tx_chn_id_by_ant_id(&map_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	chip_data.chip_id = map_data.chip_id;
	chip_data.odd_parity_en = 0;
	if (map_data.chn_id < AWMF_0165_CHN_1_B)
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_TX_A;
	}
	else
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_TX_B;
		map_data.chn_id -= AWMF_0165_CHN_1_B; // re-index the channel id
	}

	ret = awmf_0165_drv_reg_get(chip_data.chip_id, chip_data.addr_offset, &data_high, &data_low);
	if (ret != RET_OK)
	{
		goto out;
	}
	reg_val = ((uint64_t)data_high << 32) + data_low;

	// get channel state
	chn_info->chn_state = (reg_val >> (44 + map_data.chn_id)) == 0 ? 1 : 0;
	// get channel phase
	chn_info->phase_code = (reg_val >> (map_data.chn_id * 6)) & ((1 << 6) - 1);
	// get channel attenuation
	chn_info->amp_chn_code = (reg_val >> (map_data.chn_id * 4 + 24)) & ((1 << 4) - 1);
	// get common arm attenuation
	chn_info->amp_com_code = (reg_val >> (40)) & ((1 << 4) - 1);
#else
	chn_info->chn_state = g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].chn_state;
	chn_info->phase_code = g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].phase_code;
	chn_info->amp_com_code = g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].amp_com_code;
	chn_info->amp_chn_code = g_awmf_0165_drv_mcb.tx_chn_info[row_num][col_num].amp_chn_code;
#endif /* AWMF_CHN_INFO_FAST_READ */

out:
	return ret;
}

ret_code_t awmf_0165_drv_rx_chn_set(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info)
{
	ret_code_t ret = RET_OK;
	awmf_0165_ant_to_chip_chn_map_t map_data = {0};
	awmf_0165_dev_chip_data_t chip_data = {0};
	uint64_t reg_val = 0;
	uint32_t old_data_high = 0, old_data_low = 0;

	if ((row_num > RADAR_RX_CHN_ROW_CNT) || (col_num > RADAR_RX_CHN_COL_CNT))
	{
		ret = RET_INVALID_PARAM;
		goto out;
	}

	if (!awmf_0165_drv_is_chn_info_valid(chn_info))
	{
		ret = RET_INVALID_PARAM;
		goto out;
	}

	map_data.ant_row_id = row_num;
	map_data.ant_col_id = col_num;
	ret = awmf_0165_drv_get_rx_chn_id_by_ant_id(&map_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	chip_data.chip_id = map_data.chip_id;
	chip_data.odd_parity_en = 0;
	if (map_data.chn_id < AWMF_0165_CHN_1_B)
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_RX_A;
	}
	else
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_RX_B;
		map_data.chn_id -= AWMF_0165_CHN_1_B; // re-index the channel id
	}

	// read back the old data firstly
	ret = awmf_0165_drv_reg_get(chip_data.chip_id, chip_data.addr_offset, &old_data_high, &old_data_low);
	if (ret != RET_OK)
	{
		goto out;
	}
	reg_val = ((uint64_t)old_data_high << 32) + old_data_low;

	// set channel state
	if (chn_info->chn_state == 0)
	{
		reg_val |= (uint64_t)1 << (44 + map_data.chn_id);
	}
	else
	{
		reg_val &= ~((uint64_t)1 << (44 + map_data.chn_id));
	}
	// set channel phase
	reg_val &= ~((uint64_t)((1 << 6) - 1) << (map_data.chn_id * 6));
	reg_val |= (chn_info->phase_code << (map_data.chn_id * 6));
	// set channel attenuation
	reg_val &= ~((uint64_t)((1 << 4) - 1) << (map_data.chn_id * 4 + 24));
	reg_val |= ((uint64_t)chn_info->amp_chn_code << (map_data.chn_id * 4 + 24));
	// set common arm attenuation
	reg_val &= ~((uint64_t)((1 << 4) - 1) << (40));
	reg_val |= ((uint64_t)chn_info->amp_com_code << (40));

	chip_data.data_high = reg_val >> 32;
	chip_data.data_low = reg_val & 0xFFFFFFFF;
	ret = awmf_0165_drv_chip_serial_write(&chip_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].chn_state = chn_info->chn_state;
	g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].phase_code = chn_info->phase_code;
	g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].amp_com_code = chn_info->amp_com_code;
	g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].amp_chn_code = chn_info->amp_chn_code;

out:
	return ret;
}

ret_code_t awmf_0165_drv_rx_chn_get(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info)
{
	ret_code_t ret = RET_OK;
#if !AWMF_CHN_INFO_FAST_READ
	awmf_0165_ant_to_chip_chn_map_t map_data = {0};
	awmf_0165_dev_chip_data_t chip_data = {0};
	uint32_t data_high = 0, data_low = 0;
	uint64_t reg_val = 0;
#endif /* AWMF_CHN_INFO_FAST_READ */

	if ((row_num > RADAR_RX_CHN_ROW_CNT) || (col_num > RADAR_RX_CHN_COL_CNT) || (chn_info == NULL))
	{
		ret = RET_INVALID_PARAM;
		goto out;
	}

#if !AWMF_CHN_INFO_FAST_READ
	map_data.ant_row_id = row_num;
	map_data.ant_col_id = col_num;
	ret = awmf_0165_drv_get_rx_chn_id_by_ant_id(&map_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	chip_data.chip_id = map_data.chip_id;
	chip_data.odd_parity_en = 0;
	if (map_data.chn_id < AWMF_0165_CHN_1_B)
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_RX_A;
	}
	else
	{
		chip_data.addr_offset = AWMF_0165_REG_BW_RX_B;
		map_data.chn_id -= AWMF_0165_CHN_1_B; // re-index the channel id
	}

	ret = awmf_0165_drv_reg_get(chip_data.chip_id, chip_data.addr_offset, &data_high, &data_low);
	if (ret != RET_OK)
	{
		goto out;
	}
	reg_val = ((uint64_t)data_high << 32) + data_low;

	// get channel state
	chn_info->chn_state = (reg_val >> (44 + map_data.chn_id)) == 0 ? 1 : 0;
	// get channel phase
	chn_info->phase_code = (reg_val >> (map_data.chn_id * 6)) & ((1 << 6) - 1);
	// get channel attenuation
	chn_info->amp_chn_code = (reg_val >> (map_data.chn_id * 4 + 24)) & ((1 << 4) - 1);
	// get common arm attenuation
	chn_info->amp_com_code = (reg_val >> (40)) & ((1 << 4) - 1);
#else
	chn_info->chn_state = g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].chn_state;
	chn_info->phase_code = g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].phase_code;
	chn_info->amp_com_code = g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].amp_com_code;
	chn_info->amp_chn_code = g_awmf_0165_drv_mcb.rx_chn_info[row_num][col_num].amp_chn_code;
#endif /* AWMF_CHN_INFO_FAST_READ */

out:
	return ret;
}

ret_code_t awmf_0165_drv_close_all_chn(void)
{
	ret_code_t ret = RET_OK;
	uint32_t row_num = 0;
	uint32_t col_num = 0;
	awmf_0165_chn_info_t chn_info = {0};

	chn_info.chn_state = 0;
	chn_info.phase_code = 0;
	chn_info.amp_com_code = (1 << 4) - 1; // the max value of the attenuation
	chn_info.amp_chn_code = (1 << 4) - 1; // the max value of the attenuation

	// close all tx channels
	for (row_num = 0; row_num < RADAR_TX_CHN_ROW_CNT; row_num++)
	{
		for (col_num = 0; col_num < RADAR_TX_CHN_COL_CNT; col_num++)
		{
			ret = awmf_0165_drv_tx_chn_set(row_num, col_num, &chn_info);
			if (ret != RET_OK)
			{
				goto out;
			}
			vTaskDelay(1);
		}
	}

	// close all rx channels
	for (row_num = 0; row_num < RADAR_RX_CHN_ROW_CNT; row_num++)
	{
		for (col_num = 0; col_num < RADAR_RX_CHN_COL_CNT; col_num++)
		{
			ret = awmf_0165_drv_rx_chn_set(row_num, col_num, &chn_info);
			if (ret != RET_OK)
			{
				goto out;
			}
			vTaskDelay(1);
		}
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_workmode_set(uint32_t chip_id, uint32_t work_mode)
{
	ret_code_t ret = RET_OK;

	ret = awmf_0165_drv_io_chip_ctrl(chip_id, work_mode);

	return ret;
}

ret_code_t awmf_0165_drv_workmode_get(uint32_t chip_id, uint32_t *work_mode)
{
	ret_code_t ret = RET_OK;

	ret = awmf_0165_drv_io_chip_query(chip_id, work_mode);

	return ret;
}

ret_code_t awmf_0165_drv_reg_set(uint32_t chip_id, uint32_t reg_addr, uint32_t data_high, uint32_t data_low)
{
	ret_code_t ret = RET_OK;

	ret = awmf_0165_drv_chip_reg_write(chip_id, reg_addr, data_high, data_low);

	return ret;
}

ret_code_t awmf_0165_drv_reg_get(uint32_t chip_id, uint32_t reg_addr, uint32_t *data_high, uint32_t *data_low)
{
	ret_code_t ret = RET_OK;

	ret = awmf_0165_drv_chip_reg_read(chip_id, reg_addr, data_high, data_low);

	return ret;
}
