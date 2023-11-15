
#include "awmf_0165_io.h"

// #include "xparameters.h"
#include "xgpio.h"
#include "xil_printf.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

typedef struct awmf_0165_chain_ctrl_io_info {
	/* chain id */
	uint32_t chain_id; // ref awmf_0165_chain_id_t
	/* Wr/Rd output io */
	uint16_t wr_io_chn_id;
	uint16_t wr_io_shift;
	uint16_t wr_io_width;
	uint32_t wr_io_init_val;
	/* Mode[1:0] output io */
	uint16_t mode_io_chn_id;
	uint16_t mode_io_shift;
	uint16_t mode_io_width;
	uint32_t mode_io_init_val;
} awmf_0165_chain_ctrl_io_info_t;

typedef struct awmf_0165_chip_ctrl_io_info {
	/* chip id */
	uint32_t chip_id; // ref awmf_0165_chip_id_t
	/* [tx_en rx_en] output io */
	uint16_t workmode_io_chn_id;
	uint16_t workmode_io_shift;
	uint16_t workmode_io_width;
	uint32_t workmode_io_init_val;
} awmf_0165_chip_ctrl_io_info_t;

typedef struct awmf_0165_io_mcb {
	XGpio gpio;
	uint32_t ch_data[AWMF_0165_AXI_GPIO_CHN_CNT];
} awmf_0165_io_mcb_t;

STATIC awmf_0165_chain_ctrl_io_info_t g_chain_ctrl_io_info_tbl[] = {
	{
		AWMF_0165_GAIN_CHAIN_U08,
		WR_IO_CHN_ID_CHAIN_U08, WR_IO_SHIFT_CHAIN_U08, WR_IO_WIDTH_CHAIN_U08, WR_IO_INIT_VAL_CHAIN_U08,
		MODE_IO_CHN_ID_CHAIN_U08, MODE_IO_SHIFT_CHAIN_U08, MODE_IO_WIDTH_CHAIN_U08, MODE_IO_INIT_VAL_CHAIN_U08,
	},
	{
		AWMF_0165_TX_CHAIN_U12_U10_U09_U11,
		WR_IO_CHN_ID_CHAIN_U12_U10_U09_U11, WR_IO_SHIFT_CHAIN_U12_U10_U09_U11, WR_IO_WIDTH_CHAIN_U12_U10_U09_U11, WR_IO_INIT_VAL_CHAIN_U12_U10_U09_U11,
		MODE_IO_CHN_ID_CHAIN_U12_U10_U09_U11, MODE_IO_SHIFT_CHAIN_U12_U10_U09_U11, MODE_IO_WIDTH_CHAIN_U12_U10_U09_U11, MODE_IO_INIT_VAL_CHAIN_U12_U10_U09_U11,
	},
	{
		AWMF_0165_TX_CHAIN_U16_U14_U13_U15,
		WR_IO_CHN_ID_CHAIN_U16_U14_U13_U15, WR_IO_SHIFT_CHAIN_U16_U14_U13_U15, WR_IO_WIDTH_CHAIN_U16_U14_U13_U15, WR_IO_INIT_VAL_CHAIN_U16_U14_U13_U15,
		MODE_IO_CHN_ID_CHAIN_U16_U14_U13_U15, MODE_IO_SHIFT_CHAIN_U16_U14_U13_U15, MODE_IO_WIDTH_CHAIN_U16_U14_U13_U15, MODE_IO_INIT_VAL_CHAIN_U16_U14_U13_U15,
	},
	{
		AWMF_0165_RX_CHAIN_U19_U17_U18_U20,
		WR_IO_CHN_ID_CHAIN_U19_U17_U18_U20, WR_IO_SHIFT_CHAIN_U19_U17_U18_U20, WR_IO_WIDTH_CHAIN_U19_U17_U18_U20, WR_IO_INIT_VAL_CHAIN_U19_U17_U18_U20,
		MODE_IO_CHN_ID_CHAIN_U19_U17_U18_U20, MODE_IO_SHIFT_CHAIN_U19_U17_U18_U20, MODE_IO_WIDTH_CHAIN_U19_U17_U18_U20, MODE_IO_INIT_VAL_CHAIN_U19_U17_U18_U20,
	},
	{
		AWMF_0165_RX_CHAIN_U23_U21_U22_U24,
		WR_IO_CHN_ID_CHAIN_U23_U21_U22_U24, WR_IO_SHIFT_CHAIN_U23_U21_U22_U24, WR_IO_WIDTH_CHAIN_U23_U21_U22_U24, WR_IO_INIT_VAL_CHAIN_U23_U21_U22_U24,
		MODE_IO_CHN_ID_CHAIN_U23_U21_U22_U24, MODE_IO_SHIFT_CHAIN_U23_U21_U22_U24, MODE_IO_WIDTH_CHAIN_U23_U21_U22_U24, MODE_IO_INIT_VAL_CHAIN_U23_U21_U22_U24,
	},
};

STATIC awmf_0165_chip_ctrl_io_info_t g_chip_ctrl_io_info_tbl[] = {
	{
		AWMF_0165_TX_U08,
		WORKMODE_IO_CHN_ID_CHIP_U08, WORKMODE_IO_SHIFT_CHIP_U08, WORKMODE_IO_WIDTH_CHIP_U08, WORKMODE_IO_INIT_VAL_CHIP_U08,
	},
	{
		AWMF_0165_TX_U12,
		WORKMODE_IO_CHN_ID_CHIP_U12, WORKMODE_IO_SHIFT_CHIP_U12, WORKMODE_IO_WIDTH_CHIP_U12, WORKMODE_IO_INIT_VAL_CHIP_U12,
	},
	{
		AWMF_0165_TX_U10,
		WORKMODE_IO_CHN_ID_CHIP_U10, WORKMODE_IO_SHIFT_CHIP_U10, WORKMODE_IO_WIDTH_CHIP_U10, WORKMODE_IO_INIT_VAL_CHIP_U10,
	},
	{
		AWMF_0165_TX_U09,
		WORKMODE_IO_CHN_ID_CHIP_U09, WORKMODE_IO_SHIFT_CHIP_U09, WORKMODE_IO_WIDTH_CHIP_U09, WORKMODE_IO_INIT_VAL_CHIP_U09,
	},
	{
		AWMF_0165_TX_U11,
		WORKMODE_IO_CHN_ID_CHIP_U11, WORKMODE_IO_SHIFT_CHIP_U11, WORKMODE_IO_WIDTH_CHIP_U11, WORKMODE_IO_INIT_VAL_CHIP_U11,
	},
	{
		AWMF_0165_TX_U16,
		WORKMODE_IO_CHN_ID_CHIP_U16, WORKMODE_IO_SHIFT_CHIP_U16, WORKMODE_IO_WIDTH_CHIP_U16, WORKMODE_IO_INIT_VAL_CHIP_U16,
	},
	{
		AWMF_0165_TX_U14,
		WORKMODE_IO_CHN_ID_CHIP_U14, WORKMODE_IO_SHIFT_CHIP_U14, WORKMODE_IO_WIDTH_CHIP_U14, WORKMODE_IO_INIT_VAL_CHIP_U14,
	},
	{
		AWMF_0165_TX_U13,
		WORKMODE_IO_CHN_ID_CHIP_U13, WORKMODE_IO_SHIFT_CHIP_U13, WORKMODE_IO_WIDTH_CHIP_U13, WORKMODE_IO_INIT_VAL_CHIP_U13,
	},
	{
		AWMF_0165_TX_U15,
		WORKMODE_IO_CHN_ID_CHIP_U15, WORKMODE_IO_SHIFT_CHIP_U15, WORKMODE_IO_WIDTH_CHIP_U15, WORKMODE_IO_INIT_VAL_CHIP_U15,
	},
	{
		AWMF_0165_RX_U19,
		WORKMODE_IO_CHN_ID_CHIP_U19, WORKMODE_IO_SHIFT_CHIP_U19, WORKMODE_IO_WIDTH_CHIP_U19, WORKMODE_IO_INIT_VAL_CHIP_U19,
	},
	{
		AWMF_0165_RX_U17,
		WORKMODE_IO_CHN_ID_CHIP_U17, WORKMODE_IO_SHIFT_CHIP_U17, WORKMODE_IO_WIDTH_CHIP_U17, WORKMODE_IO_INIT_VAL_CHIP_U17,
	},
	{
		AWMF_0165_RX_U18,
		WORKMODE_IO_CHN_ID_CHIP_U18, WORKMODE_IO_SHIFT_CHIP_U18, WORKMODE_IO_WIDTH_CHIP_U18, WORKMODE_IO_INIT_VAL_CHIP_U18,
	},
	{
		AWMF_0165_RX_U20,
		WORKMODE_IO_CHN_ID_CHIP_U20, WORKMODE_IO_SHIFT_CHIP_U20, WORKMODE_IO_WIDTH_CHIP_U20, WORKMODE_IO_INIT_VAL_CHIP_U20,
	},
	{
		AWMF_0165_RX_U23,
		WORKMODE_IO_CHN_ID_CHIP_U23, WORKMODE_IO_SHIFT_CHIP_U23, WORKMODE_IO_WIDTH_CHIP_U23, WORKMODE_IO_INIT_VAL_CHIP_U23,
	},
	{
		AWMF_0165_RX_U21,
		WORKMODE_IO_CHN_ID_CHIP_U21, WORKMODE_IO_SHIFT_CHIP_U21, WORKMODE_IO_WIDTH_CHIP_U21, WORKMODE_IO_INIT_VAL_CHIP_U21,
	},
	{
		AWMF_0165_RX_U22,
		WORKMODE_IO_CHN_ID_CHIP_U22, WORKMODE_IO_SHIFT_CHIP_U22, WORKMODE_IO_WIDTH_CHIP_U22, WORKMODE_IO_INIT_VAL_CHIP_U22,
	},
	{
		AWMF_0165_RX_U24,
		WORKMODE_IO_CHN_ID_CHIP_U24, WORKMODE_IO_SHIFT_CHIP_U24, WORKMODE_IO_WIDTH_CHIP_U24, WORKMODE_IO_INIT_VAL_CHIP_U24,
	},
};

STATIC awmf_0165_io_mcb_t g_io_mcb;
STATIC XGpio sModeCtrl = {0 };

PRIVATE void awmf_0165_io_set_field_val(uint32_t chn_id, uint32_t field_val, uint32_t shift, uint32_t width)
{
	uint32_t reg_val = 0;
	XGpio *gpio = &g_io_mcb.gpio;

	reg_val = XGpio_DiscreteRead(gpio, chn_id);
	reg_val &= ~(((1 << width) - 1) << shift);
	reg_val |= (field_val & ((1 << width) - 1)) << shift;
	XGpio_DiscreteWrite(gpio, chn_id, reg_val);
	g_io_mcb.ch_data[chn_id - 1] = reg_val;

	return;
}

PRIVATE uint32_t awmf_0165_io_get_field_val(uint32_t chn_id, uint32_t shift, uint32_t width)
{
	uint32_t reg_val = 0;
	uint32_t field_val = 0;
	XGpio *gpio = &g_io_mcb.gpio;

	reg_val = XGpio_DiscreteRead(gpio, chn_id);
	field_val = (reg_val >> shift) & ((1 << width) - 1);

	return field_val;
}

PRIVATE ret_code_t awmf_0165_io_dev_init(void)
{
	ret_code_t ret = RET_OK;
	int32_t status = 0;
	XGpio *gpio = &g_io_mcb.gpio;
	static bool inited = false;

	if (inited)
	{
		goto out;
	}

	status = XGpio_Initialize(gpio, AWMF_0165_AXI_GPIO_DEV_ID);
	if (status != XST_SUCCESS)
	{
		xil_printf("AWMF0165 gpio initialization failed\r\n");
		ret = RET_GENERAL_ERR;
		goto out;
	}

	status = XGpio_Initialize(&sModeCtrl, XPAR_AXI_GPIO_AWMF_MODE_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		xil_printf("AWMF0165 mode gpio initialization failed\r\n");
		ret = RET_GENERAL_ERR;
		goto out;
	}

	XGpio_SetDataDirection(gpio, 1, IO_DIR_OUT);
	XGpio_SetDataDirection(gpio, 2, IO_DIR_OUT);
	XGpio_SetDataDirection(&sModeCtrl, 1, IO_DIR_OUT);
	XGpio_DiscreteWrite(&sModeCtrl, 1, 1);

	inited = true;

out:
	return ret;
}

ret_code_t awmf_0165_drv_io_init(void)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;

	ret = awmf_0165_io_dev_init();
	if (ret != RET_OK)
	{
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(g_chain_ctrl_io_info_tbl); i++)
	{
		// XGpio_SetDataDirection(gpio, g_chain_ctrl_io_info_tbl[i].wr_io_chn_id, IO_DIR_OUT);
		awmf_0165_io_set_field_val(g_chain_ctrl_io_info_tbl[i].wr_io_chn_id,
			g_chain_ctrl_io_info_tbl[i].wr_io_init_val,
			g_chain_ctrl_io_info_tbl[i].wr_io_shift,
			g_chain_ctrl_io_info_tbl[i].wr_io_width);

		// XGpio_SetDataDirection(gpio, g_chain_ctrl_io_info_tbl[i].mode_io_chn_id, IO_DIR_OUT);
		awmf_0165_io_set_field_val(g_chain_ctrl_io_info_tbl[i].mode_io_chn_id,
			g_chain_ctrl_io_info_tbl[i].mode_io_init_val,
			g_chain_ctrl_io_info_tbl[i].mode_io_shift,
			g_chain_ctrl_io_info_tbl[i].mode_io_width);
	}

	for (i = 0; i < ARRAY_SIZE(g_chip_ctrl_io_info_tbl); i++)
	{
		// XGpio_SetDataDirection(gpio, g_chip_ctrl_io_info_tbl[i].workmode_io_chn_id, IO_DIR_OUT);
		awmf_0165_io_set_field_val(g_chip_ctrl_io_info_tbl[i].workmode_io_chn_id,
			g_chip_ctrl_io_info_tbl[i].workmode_io_init_val,
			g_chip_ctrl_io_info_tbl[i].workmode_io_shift,
			g_chip_ctrl_io_info_tbl[i].workmode_io_width);
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_io_chain_ctrl(uint32_t chain_id, uint32_t wr_rd, uint32_t mode)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;
	awmf_0165_chain_ctrl_io_info_t *item = NULL;

	for (i = 0; i < ARRAY_SIZE(g_chain_ctrl_io_info_tbl); i++)
	{
		if (chain_id == g_chain_ctrl_io_info_tbl[i].chain_id)
		{
			item = &g_chain_ctrl_io_info_tbl[i];
			break;
		}
	}

	if (item == NULL)
	{
		ret = RET_GENERAL_ERR;
		goto out;
	}

	awmf_0165_io_set_field_val(item->wr_io_chn_id, wr_rd, item->wr_io_shift, item->wr_io_width);
	awmf_0165_io_set_field_val(item->mode_io_chn_id, mode, item->mode_io_shift, item->mode_io_width);

out:
	return ret;
}

ret_code_t awmf_0165_drv_io_chip_ctrl(uint32_t chip_id, uint32_t work_mode)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;
	awmf_0165_chip_ctrl_io_info_t *item = NULL;

	for (i = 0; i < ARRAY_SIZE(g_chip_ctrl_io_info_tbl); i++)
	{
		if (chip_id == g_chip_ctrl_io_info_tbl[i].chip_id)
		{
			item = &g_chip_ctrl_io_info_tbl[i];
			break;
		}
	}

	if (item == NULL)
	{
		ret = RET_GENERAL_ERR;
		goto out;
	}

	awmf_0165_io_set_field_val(item->workmode_io_chn_id, work_mode, item->workmode_io_shift, item->workmode_io_width);

out:
	return ret;
}

ret_code_t awmf_0165_drv_io_chip_query(uint32_t chip_id, uint32_t *work_mode)
{
	ret_code_t ret = RET_OK;
	uint32_t i = 0;
	awmf_0165_chip_ctrl_io_info_t *item = NULL;

	for (i = 0; i < ARRAY_SIZE(g_chip_ctrl_io_info_tbl); i++)
	{
		if (chip_id == g_chip_ctrl_io_info_tbl[i].chip_id)
		{
			item = &g_chip_ctrl_io_info_tbl[i];
			break;
		}
	}

	if (item == NULL)
	{
		ret = RET_GENERAL_ERR;
		goto out;
	}

	*work_mode = awmf_0165_io_get_field_val(item->workmode_io_chn_id, item->workmode_io_shift, item->workmode_io_width);

out:
	return ret;
}
