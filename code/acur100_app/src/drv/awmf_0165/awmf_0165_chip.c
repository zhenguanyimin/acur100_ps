
#include "awmf_0165_chip.h"
#include "awmf_0165_io.h"

#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#define AWMF_0165_REG_READBACK_CHECK_ENABLE (0)

typedef struct awmf_0165_reg_init_data {
	uint32_t reg_addr;
	uint32_t data_high;
	uint32_t data_low;
} awmf_0165_reg_init_data_t;

// see 'Table 11: AWMF-0165 Initialization Command Sequence (evm7L)' at page 21 of the ¡¶AWMF-0165-SPI-UG_rev2.pdf¡·
STATIC awmf_0165_reg_init_data_t g_reg_init_data[] = {
	{AWMF_0165_REG_MODE, 0x0000, 0x00010000},
	{AWMF_0165_REG_BW_TX_A, 0x0000, 0x00000000},
	{AWMF_0165_REG_BW_RX_A, 0x0000, 0x00000000},
	{AWMF_0165_REG_ATTTC, 0x0000, 0x0005a98b},
	{0x0006, 0x0000, 0x02186186},
	{0x0007, 0x0000, 0x02186186},
	{0x0008, 0x0000, 0x00444443},
	{0x0009, 0x0000, 0x00444485},
	{0x000a, 0x0000, 0x0965b292},
	{0x000b, 0x0000, 0x004923e0},
	{0x000c, 0x0000, 0x10440400},
	{0x000d, 0x0000, 0x00a80000},
	{0x000e, 0x0000, 0x28140a05},
	{0x000f, 0x0000, 0x28140a05},
	{0x0010, 0x0000, 0x00000000},
	{0x0011, 0x0000, 0x00000000},
	{0x0012, 0x0000, 0x00000000},
	{0x0017, 0x0000, 0x02186186},
	{0x0018, 0x0000, 0x02186186},
	{0x0019, 0x0000, 0x28140a05},
	{0x001a, 0x0000, 0x28140a05},
	{0x001b, 0x0000, 0x00000000},
	{0x001c, 0x0000, 0x00000000},
	{0x001d, 0x0000, 0x220887de},
	{0x001e, 0x0000, 0x243c243f},
	{0x001f, 0x0000, 0x00000fa0},
	{0x0020, 0x0000, 0x1fc00004},
	{0x0021, 0x0000, 0x0000775f},
	{AWMF_0165_REG_BW_TX_B, 0x0000, 0x00000000},
	{AWMF_0165_REG_BW_RX_B, 0x0000, 0x00000000},
	{AWMF_0165_REG_RADDR, 0x0000, 0x00000030},
	{0x004e, 0x0000, 0x00000000},
};

PRIVATE uint32_t awmf_0165_drv_make_high_32_bits(uint32_t odd_parity_en, uint32_t addr_offset, uint32_t data_high)
{
	uint32_t result = 0;

	result = data_high & 0xFFFF; // the data length is 16 bits
	result += addr_offset << 16; // the address length is 10 bits
	if (odd_parity_en)
	{
		result += 1 << 26;
	}

	return result;
}

PRIVATE awmf_0165_chain_id_t awmf_0165_drv_get_chain_id_by_chip_id(awmf_0165_chip_id_t chip_id)
{
	awmf_0165_chain_id_t chain_id = 0;

	switch (chip_id)
	{
	case AWMF_0165_TX_U08:
		chain_id = AWMF_0165_GAIN_CHAIN_U08;
		break;
	case AWMF_0165_TX_U12:
	case AWMF_0165_TX_U10:
	case AWMF_0165_TX_U09:
	case AWMF_0165_TX_U11:
		chain_id = AWMF_0165_TX_CHAIN_U12_U10_U09_U11;
		break;
	case AWMF_0165_TX_U16:
	case AWMF_0165_TX_U14:
	case AWMF_0165_TX_U13:
	case AWMF_0165_TX_U15:
		chain_id = AWMF_0165_TX_CHAIN_U16_U14_U13_U15;
		break;
	case AWMF_0165_RX_U19:
	case AWMF_0165_RX_U17:
	case AWMF_0165_RX_U18:
	case AWMF_0165_RX_U20:
		chain_id = AWMF_0165_RX_CHAIN_U19_U17_U18_U20;
		break;
	case AWMF_0165_RX_U23:
	case AWMF_0165_RX_U21:
	case AWMF_0165_RX_U22:
	case AWMF_0165_RX_U24:
		chain_id = AWMF_0165_RX_CHAIN_U23_U21_U22_U24;
		break;
	default:
		break;
	}

	return chain_id;
}

PRIVATE uint32_t awmf_0165_drv_get_axi_base_addr_by_chain_id(awmf_0165_chain_id_t chain_id)
{
	uint32_t axi_base_addr = 0;

	switch (chain_id)
	{
	case AWMF_0165_GAIN_CHAIN_U08:
		axi_base_addr = XPAR_AXI2REG0165_0_BASEADDR;
		break;
	case AWMF_0165_TX_CHAIN_U12_U10_U09_U11:
		axi_base_addr = XPAR_AXI2REG0165_2_BASEADDR;
		break;
	case AWMF_0165_TX_CHAIN_U16_U14_U13_U15:
		axi_base_addr = XPAR_AXI2REG0165_1_BASEADDR;
		break;
	case AWMF_0165_RX_CHAIN_U19_U17_U18_U20:
		axi_base_addr = XPAR_AXI2REG0165_4_BASEADDR;
		break;
	case AWMF_0165_RX_CHAIN_U23_U21_U22_U24:
		axi_base_addr = XPAR_AXI2REG0165_3_BASEADDR;
		break;
	default:
		break;
	}

	return axi_base_addr;
}

PRIVATE uint32_t awmf_0165_drv_get_axi_tx_addr_by_chain_id(awmf_0165_chain_id_t chain_id)
{
	uint32_t axi_tx_addr = 0;

	axi_tx_addr = awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_id);
	axi_tx_addr += 32; // 32 is the fixed offset value

	return axi_tx_addr;
}

PRIVATE ret_code_t awmf_0165_drv_write_single_chip_in_chain(uint32_t total_chips_in_chain, uint32_t pos_idx, const awmf_0165_dev_chip_data_t *chip_data)
{
	ret_code_t ret = RET_OK;
	uint32_t chain_id = awmf_0165_drv_get_chain_id_by_chip_id(chip_data->chip_id);
	uint32_t mode = AWMF_0165_CHAIN_SERIAL_4_CHIP;
	int32_t i = 0;

	if (chain_id == AWMF_0165_GAIN_CHAIN_U08)
	{
		mode = AWMF_0165_CHAIN_SERIAL_1_CHIP;
	}
	ret = awmf_0165_drv_io_chain_ctrl(chain_id, AWMF_0165_CHAIN_WRITE, mode);
	if (ret != RET_OK)
	{
		goto out;
	}

	for (i = total_chips_in_chain; i > 0; i--)
	{
		if (i == pos_idx + 1)
		{
			Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_id), awmf_0165_drv_make_high_32_bits(chip_data->odd_parity_en, chip_data->addr_offset, chip_data->data_high));
			Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_id), chip_data->data_low);
		}
		else
		{
			Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_id), awmf_0165_drv_make_high_32_bits(chip_data->odd_parity_en, AWMF_0165_REG_SPARE_TST, AWMF_0165_DUMMY_DATA_H16));
			Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_id), AWMF_0165_DUMMY_DATA_L32);
		}
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_chip_serial_write(const awmf_0165_dev_chip_data_t *chip_data)
{
	ret_code_t ret = RET_OK;

	// note: chip_data isn't checked since it is only used inner the 0165 driver module only

	switch (chip_data->chip_id)
	{
	case AWMF_0165_TX_U08: /* single chip in a chain */
		ret = awmf_0165_drv_write_single_chip_in_chain(1, 0, chip_data);
		break;
	case AWMF_0165_TX_U12: /* first chip in a chain */
	case AWMF_0165_TX_U16:
	case AWMF_0165_RX_U19:
	case AWMF_0165_RX_U23:
		ret = awmf_0165_drv_write_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 0, chip_data);
		break;
	case AWMF_0165_TX_U10: /* second chip in a chain */
	case AWMF_0165_TX_U14:
	case AWMF_0165_RX_U17:
	case AWMF_0165_RX_U21:
		ret = awmf_0165_drv_write_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 1, chip_data);
		break;
	case AWMF_0165_TX_U09: /* third chip in a chain */
	case AWMF_0165_TX_U13:
	case AWMF_0165_RX_U18:
	case AWMF_0165_RX_U22:
		ret = awmf_0165_drv_write_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 2, chip_data);
		break;
	case AWMF_0165_TX_U11: /* fourth chip in a chain */
	case AWMF_0165_TX_U15:
	case AWMF_0165_RX_U20:
	case AWMF_0165_RX_U24:
		ret = awmf_0165_drv_write_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 3, chip_data);
		break;
	default:
		ret = RET_INVALID_PARAM;
		break;
	}

	return ret;
}

ret_code_t awmf_0165_drv_chip_reg_write(awmf_0165_chip_id_t chip_id, uint32_t addr_offset, uint32_t data_high, uint32_t data_low)
{
	ret_code_t ret = RET_OK;
	awmf_0165_dev_chip_data_t chip_data = {0};
#if AWMF_0165_REG_READBACK_CHECK_ENABLE
	uint32_t data_high_rb = 0;
	uint32_t data_low_rb = 0;
#endif /* AWMF_0165_REG_READBACK_CHECK_ENABLE */

	chip_data.chip_id = chip_id;
	chip_data.odd_parity_en = 0;

	chip_data.addr_offset = addr_offset;
	chip_data.data_high = data_high & 0xFFFF;
	chip_data.data_low = data_low;
	ret = awmf_0165_drv_chip_serial_write(&chip_data);

#if AWMF_0165_REG_READBACK_CHECK_ENABLE
	(void)awmf_0165_drv_chip_reg_read(chip_id, addr_offset, &data_high_rb, &data_low_rb);
	if ((data_high_rb != data_high) || (data_low_rb != data_low))
	{
		xil_printf("Warning: chip_id=0x%x, addr_offset=0x%x, data_high=0x%x, data_low=0x%x, data_high_rb=0x%x, data_low_rb=0x%x\r\n",
			chip_id, addr_offset, data_high, data_low, data_high_rb, data_low_rb);
		// ret = RET_SYSTEM_ERR;
	}
#endif /* AWMF_0165_REG_READBACK_CHECK_ENABLE */

	return ret;
}

ret_code_t awmf_0165_drv_chain_serial_write(const awmf_0165_dev_chain_data_t *chain_data)
{
	ret_code_t ret = RET_OK;
	uint32_t mode = AWMF_0165_CHAIN_SERIAL_4_CHIP;
	int32_t i = 0;

	if (chain_data->chain_id == AWMF_0165_GAIN_CHAIN_U08)
	{
		mode = AWMF_0165_CHAIN_SERIAL_1_CHIP;
	}
	ret = awmf_0165_drv_io_chain_ctrl(chain_data->chain_id, AWMF_0165_CHAIN_WRITE, mode);
	if (ret != RET_OK)
	{
		goto out;
	}

	switch (chain_data->chain_id)
	{
	case AWMF_0165_GAIN_CHAIN_U08:
		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_data->chain_id), awmf_0165_drv_make_high_32_bits(
			chain_data->chip_data[i].odd_parity_en, chain_data->chip_data[i].addr_offset, chain_data->chip_data[i].data_high));
		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_data->chain_id), chain_data->chip_data[i].data_low);
		break;
	case AWMF_0165_TX_CHAIN_U12_U10_U09_U11:
	case AWMF_0165_TX_CHAIN_U16_U14_U13_U15:
	case AWMF_0165_RX_CHAIN_U19_U17_U18_U20:
	case AWMF_0165_RX_CHAIN_U23_U21_U22_U24:
		for (i = AWMF_0165_MAX_CHIP_NUM_PER_CHAIN - 1; i >= 0; i--)
		{
			Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_data->chain_id), awmf_0165_drv_make_high_32_bits(
				chain_data->chip_data[i].odd_parity_en, chain_data->chip_data[i].addr_offset, chain_data->chip_data[i].data_high));
			Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_data->chain_id), chain_data->chip_data[i].data_low);
		}
		break;
	default:
		ret = RET_INVALID_PARAM;
		break;
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_chain_fast_parallel_write(const awmf_0165_dev_fast_par_data_t *fast_par_data)
{
	ret_code_t ret = RET_OK;
	uint32_t data_high = 0;
	uint32_t data_low = 0;
	int32_t i = 0;

	ret = awmf_0165_drv_io_chain_ctrl(fast_par_data->chain_id, AWMF_0165_CHAIN_WRITE, AWMF_0165_CHAIN_FAST_PARALLEL);
	if (ret != RET_OK)
	{
		goto out;
	}

	data_high = 3; // it's a fixed value

	data_low = fast_par_data->fbs_addr_a & ((1 << 9) - 1);
	data_low |= (fast_par_data->fbs_addr_b & ((1 << 9) - 1)) << 9;
	data_low |= (fast_par_data->tdbs_addr_a & ((1 << 6) - 1)) << (9 + 9);
	data_low |= (fast_par_data->tdbs_addr_b & ((1 << 6) - 1)) << (9 + 9 + 6);

    if (fast_par_data->type == AWMF_FAST_PAR_WRITE_TX)
    {
    	data_low |= 3 << (9 + 9 + 6 + 6);
    }
    else // if (fast_par_data->type == AWMF_FAST_PAR_WRITE_RX)
    {
    	data_low |= 2 << (9 + 9 + 6 + 6);
    }

    if (fast_par_data->chain_id != AWMF_0165_GAIN_CHAIN_U08)
    {
    	/* write dummy data */
    	for (i = 0; i < AWMF_0165_MAX_CHIP_NUM_PER_CHAIN - 1; i++)
    	{
    		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(fast_par_data->chain_id), AWMF_0165_DUMMY_DATA_H32);
    		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(fast_par_data->chain_id), AWMF_0165_DUMMY_DATA_L32);
    	}
    }

    /* write the actual 34 bit data */
    Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(fast_par_data->chain_id), data_high);
    Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(fast_par_data->chain_id), data_low);

    // note:  mode register value should be change by awmf_0165_drv_chip_fbs_tdbs_select so that this function can take effect

out:
	return ret;
}

PRIVATE ret_code_t awmf_0165_drv_read_single_chip_in_chain(uint32_t total_chips_in_chain, uint32_t pos_idx, awmf_0165_dev_chip_data_t *chip_data)
{
	ret_code_t ret = RET_OK;
	uint32_t chain_id = awmf_0165_drv_get_chain_id_by_chip_id(chip_data->chip_id);
	uint32_t mode = AWMF_0165_CHAIN_SERIAL_4_CHIP;
	int32_t i = 0;

	if (chain_id == AWMF_0165_GAIN_CHAIN_U08)
	{
		mode = AWMF_0165_CHAIN_SERIAL_1_CHIP;
	}
	ret = awmf_0165_drv_io_chain_ctrl(chain_id, AWMF_0165_CHAIN_READ, mode);
	if (ret != RET_OK)
	{
		goto out;
	}

	for (i = 0; i < total_chips_in_chain; i++)
	{
		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_id), awmf_0165_drv_make_high_32_bits(chip_data->odd_parity_en, AWMF_0165_REG_RADDR, 0));
		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_id), chip_data->addr_offset);
	}
	usleep(1000);

	chip_data->data_low = Xil_In32(awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_id) + 4*(2*(total_chips_in_chain - 1 - pos_idx)));
	chip_data->data_high = Xil_In32(awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_id) + 4*(2*(total_chips_in_chain - 1 - pos_idx) + 1));

out:
	return ret;
}

ret_code_t awmf_0165_drv_chip_serial_read(awmf_0165_dev_chip_data_t *chip_data)
{
	ret_code_t ret = RET_OK;

	switch (chip_data->chip_id)
	{
	case AWMF_0165_TX_U08: /* single chip in a chain */
		ret = awmf_0165_drv_read_single_chip_in_chain(1, 0, chip_data);
		break;
	case AWMF_0165_TX_U12: /* first chip in a chain */
	case AWMF_0165_TX_U16:
	case AWMF_0165_RX_U19:
	case AWMF_0165_RX_U23:
		ret = awmf_0165_drv_read_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 0, chip_data);
		break;
	case AWMF_0165_TX_U10: /* second chip in a chain */
	case AWMF_0165_TX_U14:
	case AWMF_0165_RX_U17:
	case AWMF_0165_RX_U21:
		ret = awmf_0165_drv_read_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 1, chip_data);
		break;
	case AWMF_0165_TX_U09: /* third chip in a chain */
	case AWMF_0165_TX_U13:
	case AWMF_0165_RX_U18:
	case AWMF_0165_RX_U22:
		ret = awmf_0165_drv_read_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 2, chip_data);
		break;
	case AWMF_0165_TX_U11: /* fourth chip in a chain */
	case AWMF_0165_TX_U15:
	case AWMF_0165_RX_U20:
	case AWMF_0165_RX_U24:
		ret = awmf_0165_drv_read_single_chip_in_chain(AWMF_0165_MAX_CHIP_NUM_PER_CHAIN, 3, chip_data);
		break;
	default:
		ret = RET_INVALID_PARAM;
		break;
	}

	return ret;
}

ret_code_t awmf_0165_drv_chip_reg_read(awmf_0165_chip_id_t chip_id, uint32_t addr_offset, uint32_t *data_high, uint32_t *data_low)
{
	ret_code_t ret = RET_OK;
	awmf_0165_dev_chip_data_t chip_data = {0};

	chip_data.chip_id = chip_id;
	chip_data.odd_parity_en = 0;

	chip_data.addr_offset = addr_offset;
	ret = awmf_0165_drv_chip_serial_read(&chip_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	*data_high = chip_data.data_high;
	*data_low = chip_data.data_low;

out:
	return ret;
}

ret_code_t awmf_0165_drv_chain_serial_read(awmf_0165_dev_chain_data_t *chain_data)
{
	ret_code_t ret = RET_OK;
	uint32_t mode = AWMF_0165_CHAIN_SERIAL_4_CHIP;
	uint32_t total_chips_in_chain = AWMF_0165_MAX_CHIP_NUM_PER_CHAIN;
	int32_t i = 0;

	if (chain_data->chain_id == AWMF_0165_GAIN_CHAIN_U08)
	{
		mode = AWMF_0165_CHAIN_SERIAL_1_CHIP;
		total_chips_in_chain = 1;
	}
	ret = awmf_0165_drv_io_chain_ctrl(chain_data->chain_id, AWMF_0165_CHAIN_READ, mode);
	if (ret != RET_OK)
	{
		goto out;
	}

	for (i = total_chips_in_chain - 1; i >= 0; i--)
	{
		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_data->chain_id), awmf_0165_drv_make_high_32_bits(chain_data->chip_data[i].odd_parity_en, AWMF_0165_REG_RADDR, 0));
		Xil_Out32(awmf_0165_drv_get_axi_tx_addr_by_chain_id(chain_data->chain_id), chain_data->chip_data[i].addr_offset);
	}
	usleep(1000);

	switch (chain_data->chain_id)
	{
	case AWMF_0165_GAIN_CHAIN_U08:
		i = 0;
		chain_data->chip_data[i].data_low = Xil_In32(awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_data->chain_id) + 0*4);
		chain_data->chip_data[i].data_high = Xil_In32(awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_data->chain_id) + 1*4);
		break;
	case AWMF_0165_TX_CHAIN_U12_U10_U09_U11:
	case AWMF_0165_TX_CHAIN_U16_U14_U13_U15:
	case AWMF_0165_RX_CHAIN_U19_U17_U18_U20:
	case AWMF_0165_RX_CHAIN_U23_U21_U22_U24:
		for (i = 0; i < total_chips_in_chain; i++)
		{
			chain_data->chip_data[i].data_low = Xil_In32(awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_data->chain_id) + 4*(2*(total_chips_in_chain - 1 - i)));
			chain_data->chip_data[i].data_high = Xil_In32(awmf_0165_drv_get_axi_base_addr_by_chain_id(chain_data->chain_id) + 4*(2*(total_chips_in_chain - 1 - i) + 1));
		}
		break;
	default:
		ret = RET_INVALID_PARAM;
		break;
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_chip_init(awmf_0165_chip_id_t chip_id)
{
	ret_code_t ret = RET_OK;
	int32_t i = 0;

	for (i = 0; i < ARRAY_SIZE(g_reg_init_data); i++)
	{
		ret = awmf_0165_drv_chip_reg_write(chip_id, g_reg_init_data[i].reg_addr, g_reg_init_data[i].data_high, g_reg_init_data[i].data_low);
		if (ret != RET_OK)
		{
			goto out;
		}
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_chip_reset(awmf_0165_chip_id_t chip_id)
{
	ret_code_t ret = RET_OK;
	awmf_0165_dev_chip_data_t chip_data = {0};

	chip_data.chip_id = chip_id;
	chip_data.odd_parity_en = 0;
	chip_data.addr_offset = AWMF_0165_REG_MODE;

	// reset the chip
	chip_data.data_high = 0;
	chip_data.data_low = (1 << 16) | (1 << 24); // bit 24 is sw_reset
	ret = awmf_0165_drv_chip_serial_write(&chip_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	// usleep(1000);

	chip_data.data_low = (1 << 16) | (0 << 24);  // bit 24 is sw_reset
	ret = awmf_0165_drv_chip_serial_write(&chip_data);
	if (ret != RET_OK)
	{
		goto out;
	}

	// close all TX and RX channels in the chip, also set the attenuation and phase-shift to default (0)
	ret = awmf_0165_drv_chip_reg_write(chip_id, AWMF_0165_REG_BW_TX_A, 0xF000, 0);
	if (ret != RET_OK)
	{
		goto out;
	}

	ret = awmf_0165_drv_chip_reg_write(chip_id, AWMF_0165_REG_BW_RX_A, 0xF000, 0);
	if (ret != RET_OK)
	{
		goto out;
	}

	ret = awmf_0165_drv_chip_reg_write(chip_id, AWMF_0165_REG_BW_TX_B, 0xF000, 0);
	if (ret != RET_OK)
	{
		goto out;
	}

	ret = awmf_0165_drv_chip_reg_write(chip_id, AWMF_0165_REG_BW_RX_B, 0xF000, 0);
	if (ret != RET_OK)
	{
		goto out;
	}

out:
	return ret;
}

ret_code_t awmf_0165_drv_chip_fbs_tdbs_select(awmf_0165_chip_id_t chip_id, awmf_0165_fbs_tdbs_location_t fbs_sel, awmf_0165_fbs_tdbs_location_t tdbs_sel)
{
	ret_code_t ret = RET_OK;
	uint32_t data_high = 0;
	uint32_t data_low = 0;

	ret = awmf_0165_drv_chip_reg_read(chip_id, AWMF_0165_REG_MODE, &data_high, &data_low);
	if (ret != RET_OK)
	{
		goto out;
	}

	data_low &= ~(((1 << 2) - 1) << 1);
	if (fbs_sel == AWMF_0165_FBS_TDBS_LOC_MEM)
	{
		data_low |= 1 << 2;
	}
	if (tdbs_sel == AWMF_0165_FBS_TDBS_LOC_MEM)
	{
		data_low |= 1 << 1;
	}

	ret = awmf_0165_drv_chip_reg_write(chip_id, AWMF_0165_REG_MODE, data_high, data_low);
	if (ret != RET_OK)
	{
		goto out;
	}

out:
	return ret;
}
