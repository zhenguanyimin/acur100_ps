
#ifndef AWMF_0165_DEF_H
#define AWMF_0165_DEF_H

#include "../../inc/radar_common.h"
#include "xparameters.h"

#define AWMF_0165_MAX_CHIP_NUM_PER_CHAIN (4)

typedef enum awmf_0165_operating_mode {
	AWMF_0165_OPM_STANDBY = 0, /* RF circuits powered down. Fast switching to Tx/Rx modes. */
	AWMF_0165_OPM_TX_MODE = 1, /* Transmit mode. */
	AWMF_0165_OPM_RX_MODE = 2, /* Receive mode. */
	AWMF_0165_OPM_SLEEP = 3 /* RF/bias circuits powered down. Low power dissipation. Slow switching to Tx/Rx modes. */
} awmf_0165_operating_mode_t;
#define IS_0165_WORKMODE_VALID(val) (((val) >= 0) && ((val) <= 3))

typedef enum awmf_0165_chain_wr_rd {
	AWMF_0165_CHAIN_READ = 0,
	AWMF_0165_CHAIN_WRITE = 1
} awmf_0165_chain_wr_rd_t;

typedef enum awmf_0165_chain_mode {
	AWMF_0165_CHAIN_SERIAL_4_CHIP = 1, // b01: 240 bits SDO
	AWMF_0165_CHAIN_SERIAL_1_CHIP = 3, // b11: 60 bits SDO
	AWMF_0165_CHAIN_FAST_PARALLEL = 2, // b10: 34 bits PDO
} awmf_0165_chain_mode_t;

typedef enum awmf_0165_chain_status {
	AWMF_0165_CHAIN_IDLE = 0,
	AWMF_0165_CHAIN_BUSY = 1
} awmf_0165_chain_status_t;

typedef enum awmf_0165_chain_id {
	AWMF_0165_GAIN_CHAIN_U08 = 0, /* SPI1 */
	AWMF_0165_TX_CHAIN_U12_U10_U09_U11 = 2, /* SPI2 */
	AWMF_0165_TX_CHAIN_U16_U14_U13_U15 = 1, /* SPI3 */
	AWMF_0165_RX_CHAIN_U19_U17_U18_U20 = 4, /* SPI4 */
	AWMF_0165_RX_CHAIN_U23_U21_U22_U24 = 3, /* SPI5 */
} awmf_0165_chain_id_t;

typedef enum awmf_0165_chip_id {
	/* SPI1 */
	AWMF_0165_TX_U08 = 0x00, // U8
	/* SPI2 */
	AWMF_0165_TX_U12 = 0x11, // U12
	AWMF_0165_TX_U10 = 0x12, // U10
	AWMF_0165_TX_U09 = 0x13, // U9
	AWMF_0165_TX_U11 = 0x14, // U11
	/* SPI3 */
	AWMF_0165_TX_U16 = 0x21, // U16
	AWMF_0165_TX_U14 = 0x22, // U14
	AWMF_0165_TX_U13 = 0x23, // U13
	AWMF_0165_TX_U15 = 0x24, // U15
	/* SPI4 */
	AWMF_0165_RX_U19 = 0x31, // U19
	AWMF_0165_RX_U17 = 0x32, // U17
	AWMF_0165_RX_U18 = 0x33, // U18
	AWMF_0165_RX_U20 = 0x34, // U20
	/* SPI5 */
	AWMF_0165_RX_U23 = 0x41, // U23
	AWMF_0165_RX_U21 = 0x42, // U21
	AWMF_0165_RX_U22 = 0x43, // U22
	AWMF_0165_RX_U24 = 0x44, // U24
} awmf_0165_chip_id_t;

typedef enum awmf_0165_chn_id {
	AWMF_0165_CHN_1_A = 0,
	AWMF_0165_CHN_2_A = 1,
	AWMF_0165_CHN_3_A = 2,
	AWMF_0165_CHN_4_A = 3,
	AWMF_0165_CHN_1_B = 4,
	AWMF_0165_CHN_2_B = 5,
	AWMF_0165_CHN_3_B = 6,
	AWMF_0165_CHN_4_B = 7,
} awmf_0165_chn_id_t;

typedef enum awmf_0165_reg_addr {
	AWMF_0165_REG_MODE = 0x00, /* Mode [r/w] */
	AWMF_0165_REG_BW_TX_A = 0x01, /* Beam Weight Tx, A [r/w] */
	AWMF_0165_REG_BW_RX_A = 0x02, /* Beam Weight Rx, A [r/w] */
	AWMF_0165_REG_FBS_RX_ADDR = 0x03, /* FDB/TDBS B/A Rx Memory Addresses [r] */
	AWMF_0165_REG_FBS_TX_ADDR = 0x04, /* FDB/TDBS B/A Tx Memory Addresses [r] */
	AWMF_0165_REG_ATTTC = 0x05, /* SPI Override Tx/Rx B/A Attenuator Temp Comp - Open Loop [r/w] */
	AWMF_0165_REG_BW_TX_B = 0x22, /* Beam Weight Tx, B [r/w] */
	AWMF_0165_REG_BW_RX_B = 0x23, /* Beam Weight Rx, B [r/w] */
	AWMF_0165_REG_TELEM0 = 0x30, /* Telemetry 0 (default) [r] */
	AWMF_0165_REG_TELEM1 = 0x31, /* Telemetry 1 [r] */
	AWMF_0165_REG_TELEM2 = 0x32, /* Telemetry 2 [r] */
	AWMF_0165_REG_TELEM3 = 0x33, /* Telemetry 3 [r] */
	AWMF_0165_REG_SPARE_TST = 0x3A, /* Spare register for testing, or to write dummy data to get read-back data [r/w] */
	AWMF_0165_REG_RADDR = 0x3D, /* Serial read address [r/w] */
	AWMF_0165_REG_PROD_VID = 0x3E, /* Product version and id [r] */
	AWMF_0165_REG_VERSION = 0x3F, /* Version [r] */
	/* 0x80-0xFF: reserved for double buffered addresses [w] */
	AWMF_0165_REG_RX_TDBS_ENTRY_BASE = 0x100, /* 0x100-0x11F [r/w] */
	AWMF_0165_REG_TX_TDBS_ENTRY_BASE = 0x140, /* 0x140-0x15F [r/w] */
	AWMF_0165_REG_FBS_ENTRY_BASE = 0x200, /* 0x200-0x2FF [r/w] */
} awmf_0165_reg_addr_t;
#define IS_0165_REG_ADDR_VALID(val) (((val) >= 0) && ((val) <= 0xfff))

typedef enum awmf_0165_fbs_tdbs_location {
	AWMF_0165_FBS_TDBS_LOC_REG = 0,
	AWMF_0165_FBS_TDBS_LOC_MEM = 1
} awmf_0165_fbs_tdbs_location_t;

#define AWMF_0165_DUMMY_DATA_H16 (0xA55A)
#define AWMF_0165_DUMMY_DATA_H32 (0x5A5AA5A5)
#define AWMF_0165_DUMMY_DATA_L32 (0xDEADBEEF)

#define AWMF_FAST_PAR_WRITE_TX (1)
#define AWMF_FAST_PAR_WRITE_RX (0)

/* AXIGPIO */
#define IO_DIR_IN (1)
#define IO_DIR_OUT (0)

#define AWMF_0165_AXI_GPIO_BASE_ADDR (XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_BASEADDR) // unused
#define AWMF_0165_AXI_GPIO_DEV_ID (XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_DEVICE_ID)
#define AWMF_0165_AXI_GPIO_CHN_CNT (2)

// chain_0 AWMF_0165_GAIN_CHAIN_U08 = 0
#define WR_IO_CHN_ID_CHAIN_U08 (2)
#define WR_IO_SHIFT_CHAIN_U08 (2)
#define WR_IO_WIDTH_CHAIN_U08 (1)
#define WR_IO_INIT_VAL_CHAIN_U08 (AWMF_0165_CHAIN_WRITE)
#define MODE_IO_CHN_ID_CHAIN_U08 (2)
#define MODE_IO_SHIFT_CHAIN_U08 (0)
#define MODE_IO_WIDTH_CHAIN_U08 (2)
#define MODE_IO_INIT_VAL_CHAIN_U08 (AWMF_0165_CHAIN_SERIAL_1_CHIP)
// chain_2 AWMF_0165_TX_CHAIN_U12_U10_U09_U11 = 2
#define WR_IO_CHN_ID_CHAIN_U12_U10_U09_U11 (2)
#define WR_IO_SHIFT_CHAIN_U12_U10_U09_U11 (2*3+2)
#define WR_IO_WIDTH_CHAIN_U12_U10_U09_U11 (1)
#define WR_IO_INIT_VAL_CHAIN_U12_U10_U09_U11 (AWMF_0165_CHAIN_WRITE)
#define MODE_IO_CHN_ID_CHAIN_U12_U10_U09_U11 (2)
#define MODE_IO_SHIFT_CHAIN_U12_U10_U09_U11 (2*3+0)
#define MODE_IO_WIDTH_CHAIN_U12_U10_U09_U11 (2)
#define MODE_IO_INIT_VAL_CHAIN_U12_U10_U09_U11 (AWMF_0165_CHAIN_SERIAL_4_CHIP)
// chain_1 AWMF_0165_TX_CHAIN_U16_U14_U13_U15 = 1
#define WR_IO_CHN_ID_CHAIN_U16_U14_U13_U15 (2)
#define WR_IO_SHIFT_CHAIN_U16_U14_U13_U15 (1*3+2)
#define WR_IO_WIDTH_CHAIN_U16_U14_U13_U15 (1)
#define WR_IO_INIT_VAL_CHAIN_U16_U14_U13_U15 (AWMF_0165_CHAIN_WRITE)
#define MODE_IO_CHN_ID_CHAIN_U16_U14_U13_U15 (2)
#define MODE_IO_SHIFT_CHAIN_U16_U14_U13_U15 (1*3+0)
#define MODE_IO_WIDTH_CHAIN_U16_U14_U13_U15 (2)
#define MODE_IO_INIT_VAL_CHAIN_U16_U14_U13_U15 (AWMF_0165_CHAIN_SERIAL_4_CHIP)
// chain_4 AWMF_0165_RX_CHAIN_U19_U17_U18_U20 = 4
#define WR_IO_CHN_ID_CHAIN_U19_U17_U18_U20 (2)
#define WR_IO_SHIFT_CHAIN_U19_U17_U18_U20 (4*3+2)
#define WR_IO_WIDTH_CHAIN_U19_U17_U18_U20 (1)
#define WR_IO_INIT_VAL_CHAIN_U19_U17_U18_U20 (AWMF_0165_CHAIN_WRITE)
#define MODE_IO_CHN_ID_CHAIN_U19_U17_U18_U20 (2)
#define MODE_IO_SHIFT_CHAIN_U19_U17_U18_U20 (4*3+0)
#define MODE_IO_WIDTH_CHAIN_U19_U17_U18_U20 (2)
#define MODE_IO_INIT_VAL_CHAIN_U19_U17_U18_U20 (AWMF_0165_CHAIN_SERIAL_4_CHIP)
// chain_3 AWMF_0165_RX_CHAIN_U23_U21_U22_U24 = 3
#define WR_IO_CHN_ID_CHAIN_U23_U21_U22_U24 (2)
#define WR_IO_SHIFT_CHAIN_U23_U21_U22_U24 (3*3+2)
#define WR_IO_WIDTH_CHAIN_U23_U21_U22_U24 (1)
#define WR_IO_INIT_VAL_CHAIN_U23_U21_U22_U24 (AWMF_0165_CHAIN_WRITE)
#define MODE_IO_CHN_ID_CHAIN_U23_U21_U22_U24 (2)
#define MODE_IO_SHIFT_CHAIN_U23_U21_U22_U24 (3*3+0)
#define MODE_IO_WIDTH_CHAIN_U23_U21_U22_U24 (2)
#define MODE_IO_INIT_VAL_CHAIN_U23_U21_U22_U24 (AWMF_0165_CHAIN_SERIAL_4_CHIP)

// chip work mode control IO
// chip_U08 AWMF_0165_TX_U08 = 0x00
#define WORKMODE_IO_CHN_ID_CHIP_U08 (2)
#define WORKMODE_IO_SHIFT_CHIP_U08 (15)
#define WORKMODE_IO_WIDTH_CHIP_U08 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U08 (AWMF_0165_OPM_STANDBY)
// chip_U12 AWMF_0165_TX_U12 = 0x11
#define WORKMODE_IO_CHN_ID_CHIP_U12 (1)
#define WORKMODE_IO_SHIFT_CHIP_U12 (2*4+2*0)
#define WORKMODE_IO_WIDTH_CHIP_U12 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U12 (AWMF_0165_OPM_STANDBY)
// chip_U10 AWMF_0165_TX_U10 = 0x12
#define WORKMODE_IO_CHN_ID_CHIP_U10 (1)
#define WORKMODE_IO_SHIFT_CHIP_U10 (2*4+2*1)
#define WORKMODE_IO_WIDTH_CHIP_U10 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U10 (AWMF_0165_OPM_STANDBY)
// chip_U09 AWMF_0165_TX_U09 = 0x13
#define WORKMODE_IO_CHN_ID_CHIP_U09 (1)
#define WORKMODE_IO_SHIFT_CHIP_U09 (2*4+2*2)
#define WORKMODE_IO_WIDTH_CHIP_U09 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U09 (AWMF_0165_OPM_STANDBY)
// chip_U11 AWMF_0165_TX_U11 = 0x14
#define WORKMODE_IO_CHN_ID_CHIP_U11 (1)
#define WORKMODE_IO_SHIFT_CHIP_U11 (2*4+2*3)
#define WORKMODE_IO_WIDTH_CHIP_U11 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U11 (AWMF_0165_OPM_STANDBY)
// chip_U16 AWMF_0165_TX_U16 = 0x21
#define WORKMODE_IO_CHN_ID_CHIP_U16 (1)
#define WORKMODE_IO_SHIFT_CHIP_U16 (2*0 + 2*0)
#define WORKMODE_IO_WIDTH_CHIP_U16 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U16 (AWMF_0165_OPM_STANDBY)
// chip_U14 AWMF_0165_TX_U14 = 0x22
#define WORKMODE_IO_CHN_ID_CHIP_U14 (1)
#define WORKMODE_IO_SHIFT_CHIP_U14 (2*0 + 2*1)
#define WORKMODE_IO_WIDTH_CHIP_U14 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U14 (AWMF_0165_OPM_STANDBY)
// chip_U13 AWMF_0165_TX_U13 = 0x23
#define WORKMODE_IO_CHN_ID_CHIP_U13 (1)
#define WORKMODE_IO_SHIFT_CHIP_U13 (2*0 + 2*2)
#define WORKMODE_IO_WIDTH_CHIP_U13 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U13 (AWMF_0165_OPM_STANDBY)
// chip_U15 AWMF_0165_TX_U15 = 0x24
#define WORKMODE_IO_CHN_ID_CHIP_U15 (1)
#define WORKMODE_IO_SHIFT_CHIP_U15 (2*0 + 2*3)
#define WORKMODE_IO_WIDTH_CHIP_U15 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U15 (AWMF_0165_OPM_STANDBY)
// chip_U19 AWMF_0165_RX_U19 = 0x31
#define WORKMODE_IO_CHN_ID_CHIP_U19 (1)
#define WORKMODE_IO_SHIFT_CHIP_U19 (2*12 + 2*0)
#define WORKMODE_IO_WIDTH_CHIP_U19 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U19 (AWMF_0165_OPM_STANDBY)
// chip_U17 AWMF_0165_RX_U17 = 0x32
#define WORKMODE_IO_CHN_ID_CHIP_U17 (1)
#define WORKMODE_IO_SHIFT_CHIP_U17 (2*12 + 2*1)
#define WORKMODE_IO_WIDTH_CHIP_U17 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U17 (AWMF_0165_OPM_STANDBY)
// chip_U18 AWMF_0165_RX_U18 = 0x33
#define WORKMODE_IO_CHN_ID_CHIP_U18 (1)
#define WORKMODE_IO_SHIFT_CHIP_U18 (2*12 + 2*2)
#define WORKMODE_IO_WIDTH_CHIP_U18 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U18 (AWMF_0165_OPM_STANDBY)
// chip_U20 AWMF_0165_RX_U20 = 0x34
#define WORKMODE_IO_CHN_ID_CHIP_U20 (1)
#define WORKMODE_IO_SHIFT_CHIP_U20 (2*12 + 2*3)
#define WORKMODE_IO_WIDTH_CHIP_U20 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U20 (AWMF_0165_OPM_STANDBY)
// chip_U23 AWMF_0165_RX_U23 = 0x41
#define WORKMODE_IO_CHN_ID_CHIP_U23 (1)
#define WORKMODE_IO_SHIFT_CHIP_U23 (2*8 + 2*0)
#define WORKMODE_IO_WIDTH_CHIP_U23 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U23 (AWMF_0165_OPM_STANDBY)
// chip_U21 AWMF_0165_RX_U21 = 0x42
#define WORKMODE_IO_CHN_ID_CHIP_U21 (1)
#define WORKMODE_IO_SHIFT_CHIP_U21 (2*8 + 2*1)
#define WORKMODE_IO_WIDTH_CHIP_U21 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U21 (AWMF_0165_OPM_STANDBY)
// chip_U22 AWMF_0165_RX_U22 = 0x43
#define WORKMODE_IO_CHN_ID_CHIP_U22 (1)
#define WORKMODE_IO_SHIFT_CHIP_U22 (2*8 + 2*2)
#define WORKMODE_IO_WIDTH_CHIP_U22 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U22 (AWMF_0165_OPM_STANDBY)
// chip_U24 AWMF_0165_RX_U24 = 0x44
#define WORKMODE_IO_CHN_ID_CHIP_U24 (1)
#define WORKMODE_IO_SHIFT_CHIP_U24 (2*8 + 2*3)
#define WORKMODE_IO_WIDTH_CHIP_U24 (2)
#define WORKMODE_IO_INIT_VAL_CHIP_U24 (AWMF_0165_OPM_STANDBY)

/* PL Interrupt */
// ref interrupt_handle.h

#endif /* AWMF_0165_DEF_H */
