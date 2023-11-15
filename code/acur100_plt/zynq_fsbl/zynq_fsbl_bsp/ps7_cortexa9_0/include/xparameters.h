#ifndef XPARAMETERS_H   /* prevent circular inclusions */
#define XPARAMETERS_H   /* by using protection macros */

/* Definition for CPU ID */
#define XPAR_CPU_ID 0U

/* Definitions for peripheral PS7_CORTEXA9_0 */
#define XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ 666666687


/******************************************************************/

/* Canonical definitions for peripheral PS7_CORTEXA9_0 */
#define XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ 666666687


/******************************************************************/

#include "xparameters_ps.h"

#define STDIN_BASEADDRESS 0xE0001000
#define STDOUT_BASEADDRESS 0xE0001000

/******************************************************************/

/* Platform specific definitions */
#define PLATFORM_ZYNQ
 
/* Definitions for sleep timer configuration */
#define XSLEEP_TIMER_IS_DEFAULT_TIMER
 
 
/******************************************************************/
/* Definitions for driver AXIDMA */
#define XPAR_XAXIDMA_NUM_INSTANCES 2

/* Definitions for peripheral AXI_DMA_ADC3244 */
#define XPAR_AXI_DMA_ADC3244_DEVICE_ID 0
#define XPAR_AXI_DMA_ADC3244_BASEADDR 0x40400000
#define XPAR_AXI_DMA_ADC3244_HIGHADDR 0x4040FFFF
#define XPAR_AXI_DMA_ADC3244_SG_INCLUDE_STSCNTRL_STRM 0
#define XPAR_AXI_DMA_ADC3244_INCLUDE_MM2S_DRE 0
#define XPAR_AXI_DMA_ADC3244_INCLUDE_S2MM_DRE 0
#define XPAR_AXI_DMA_ADC3244_INCLUDE_MM2S 0
#define XPAR_AXI_DMA_ADC3244_INCLUDE_S2MM 1
#define XPAR_AXI_DMA_ADC3244_M_AXI_MM2S_DATA_WIDTH 32
#define XPAR_AXI_DMA_ADC3244_M_AXI_S2MM_DATA_WIDTH 32
#define XPAR_AXI_DMA_ADC3244_INCLUDE_SG 0
#define XPAR_AXI_DMA_ADC3244_ENABLE_MULTI_CHANNEL 0
#define XPAR_AXI_DMA_ADC3244_NUM_MM2S_CHANNELS 1
#define XPAR_AXI_DMA_ADC3244_NUM_S2MM_CHANNELS 1
#define XPAR_AXI_DMA_ADC3244_MM2S_BURST_SIZE 16
#define XPAR_AXI_DMA_ADC3244_S2MM_BURST_SIZE 256
#define XPAR_AXI_DMA_ADC3244_MICRO_DMA 0
#define XPAR_AXI_DMA_ADC3244_ADDR_WIDTH 32
#define XPAR_AXI_DMA_ADC3244_SG_LENGTH_WIDTH 14


/* Definitions for peripheral AXI_DMA_RDM */
#define XPAR_AXI_DMA_RDM_DEVICE_ID 1
#define XPAR_AXI_DMA_RDM_BASEADDR 0x40410000
#define XPAR_AXI_DMA_RDM_HIGHADDR 0x4041FFFF
#define XPAR_AXI_DMA_RDM_SG_INCLUDE_STSCNTRL_STRM 0
#define XPAR_AXI_DMA_RDM_INCLUDE_MM2S_DRE 0
#define XPAR_AXI_DMA_RDM_INCLUDE_S2MM_DRE 0
#define XPAR_AXI_DMA_RDM_INCLUDE_MM2S 0
#define XPAR_AXI_DMA_RDM_INCLUDE_S2MM 1
#define XPAR_AXI_DMA_RDM_M_AXI_MM2S_DATA_WIDTH 32
#define XPAR_AXI_DMA_RDM_M_AXI_S2MM_DATA_WIDTH 32
#define XPAR_AXI_DMA_RDM_INCLUDE_SG 0
#define XPAR_AXI_DMA_RDM_ENABLE_MULTI_CHANNEL 0
#define XPAR_AXI_DMA_RDM_NUM_MM2S_CHANNELS 1
#define XPAR_AXI_DMA_RDM_NUM_S2MM_CHANNELS 1
#define XPAR_AXI_DMA_RDM_MM2S_BURST_SIZE 16
#define XPAR_AXI_DMA_RDM_S2MM_BURST_SIZE 256
#define XPAR_AXI_DMA_RDM_MICRO_DMA 0
#define XPAR_AXI_DMA_RDM_ADDR_WIDTH 32
#define XPAR_AXI_DMA_RDM_SG_LENGTH_WIDTH 20


/******************************************************************/

/* Canonical definitions for peripheral AXI_DMA_ADC3244 */
#define XPAR_AXIDMA_0_DEVICE_ID XPAR_AXI_DMA_ADC3244_DEVICE_ID
#define XPAR_AXIDMA_0_BASEADDR 0x40400000
#define XPAR_AXIDMA_0_SG_INCLUDE_STSCNTRL_STRM 0
#define XPAR_AXIDMA_0_INCLUDE_MM2S 0
#define XPAR_AXIDMA_0_INCLUDE_MM2S_DRE 0
#define XPAR_AXIDMA_0_M_AXI_MM2S_DATA_WIDTH 32
#define XPAR_AXIDMA_0_INCLUDE_S2MM 1
#define XPAR_AXIDMA_0_INCLUDE_S2MM_DRE 0
#define XPAR_AXIDMA_0_M_AXI_S2MM_DATA_WIDTH 32
#define XPAR_AXIDMA_0_INCLUDE_SG 0
#define XPAR_AXIDMA_0_ENABLE_MULTI_CHANNEL 0
#define XPAR_AXIDMA_0_NUM_MM2S_CHANNELS 1
#define XPAR_AXIDMA_0_NUM_S2MM_CHANNELS 1
#define XPAR_AXIDMA_0_MM2S_BURST_SIZE 16
#define XPAR_AXIDMA_0_S2MM_BURST_SIZE 256
#define XPAR_AXIDMA_0_MICRO_DMA 0
#define XPAR_AXIDMA_0_c_addr_width 32
#define XPAR_AXIDMA_0_c_sg_length_width 14

/* Canonical definitions for peripheral AXI_DMA_RDM */
#define XPAR_AXIDMA_1_DEVICE_ID XPAR_AXI_DMA_RDM_DEVICE_ID
#define XPAR_AXIDMA_1_BASEADDR 0x40410000
#define XPAR_AXIDMA_1_SG_INCLUDE_STSCNTRL_STRM 0
#define XPAR_AXIDMA_1_INCLUDE_MM2S 0
#define XPAR_AXIDMA_1_INCLUDE_MM2S_DRE 0
#define XPAR_AXIDMA_1_M_AXI_MM2S_DATA_WIDTH 32
#define XPAR_AXIDMA_1_INCLUDE_S2MM 1
#define XPAR_AXIDMA_1_INCLUDE_S2MM_DRE 0
#define XPAR_AXIDMA_1_M_AXI_S2MM_DATA_WIDTH 32
#define XPAR_AXIDMA_1_INCLUDE_SG 0
#define XPAR_AXIDMA_1_ENABLE_MULTI_CHANNEL 0
#define XPAR_AXIDMA_1_NUM_MM2S_CHANNELS 1
#define XPAR_AXIDMA_1_NUM_S2MM_CHANNELS 1
#define XPAR_AXIDMA_1_MM2S_BURST_SIZE 16
#define XPAR_AXIDMA_1_S2MM_BURST_SIZE 256
#define XPAR_AXIDMA_1_MICRO_DMA 0
#define XPAR_AXIDMA_1_c_addr_width 32
#define XPAR_AXIDMA_1_c_sg_length_width 20


/******************************************************************/


/* Definitions for peripheral PS7_DDR_0 */
#define XPAR_PS7_DDR_0_S_AXI_BASEADDR 0x00100000
#define XPAR_PS7_DDR_0_S_AXI_HIGHADDR 0x3FFFFFFF


/******************************************************************/

/* Definitions for driver DEVCFG */
#define XPAR_XDCFG_NUM_INSTANCES 1U

/* Definitions for peripheral PS7_DEV_CFG_0 */
#define XPAR_PS7_DEV_CFG_0_DEVICE_ID 0U
#define XPAR_PS7_DEV_CFG_0_BASEADDR 0xF8007000U
#define XPAR_PS7_DEV_CFG_0_HIGHADDR 0xF80070FFU


/******************************************************************/

/* Canonical definitions for peripheral PS7_DEV_CFG_0 */
#define XPAR_XDCFG_0_DEVICE_ID XPAR_PS7_DEV_CFG_0_DEVICE_ID
#define XPAR_XDCFG_0_BASEADDR 0xF8007000U
#define XPAR_XDCFG_0_HIGHADDR 0xF80070FFU


/******************************************************************/

/* Definitions for driver DMAPS */
#define XPAR_XDMAPS_NUM_INSTANCES 2

/* Definitions for peripheral PS7_DMA_NS */
#define XPAR_PS7_DMA_NS_DEVICE_ID 0
#define XPAR_PS7_DMA_NS_BASEADDR 0xF8004000
#define XPAR_PS7_DMA_NS_HIGHADDR 0xF8004FFF


/* Definitions for peripheral PS7_DMA_S */
#define XPAR_PS7_DMA_S_DEVICE_ID 1
#define XPAR_PS7_DMA_S_BASEADDR 0xF8003000
#define XPAR_PS7_DMA_S_HIGHADDR 0xF8003FFF


/******************************************************************/

/* Canonical definitions for peripheral PS7_DMA_NS */
#define XPAR_XDMAPS_0_DEVICE_ID XPAR_PS7_DMA_NS_DEVICE_ID
#define XPAR_XDMAPS_0_BASEADDR 0xF8004000
#define XPAR_XDMAPS_0_HIGHADDR 0xF8004FFF

/* Canonical definitions for peripheral PS7_DMA_S */
#define XPAR_XDMAPS_1_DEVICE_ID XPAR_PS7_DMA_S_DEVICE_ID
#define XPAR_XDMAPS_1_BASEADDR 0xF8003000
#define XPAR_XDMAPS_1_HIGHADDR 0xF8003FFF


/******************************************************************/

/* Definitions for driver EMACPS */
#define XPAR_XEMACPS_NUM_INSTANCES 1

/* Definitions for peripheral PS7_ETHERNET_0 */
#define XPAR_PS7_ETHERNET_0_DEVICE_ID 0
#define XPAR_PS7_ETHERNET_0_BASEADDR 0xE000B000
#define XPAR_PS7_ETHERNET_0_HIGHADDR 0xE000BFFF
#define XPAR_PS7_ETHERNET_0_ENET_CLK_FREQ_HZ 125000000
#define XPAR_PS7_ETHERNET_0_ENET_SLCR_1000MBPS_DIV0 8
#define XPAR_PS7_ETHERNET_0_ENET_SLCR_1000MBPS_DIV1 1
#define XPAR_PS7_ETHERNET_0_ENET_SLCR_100MBPS_DIV0 8
#define XPAR_PS7_ETHERNET_0_ENET_SLCR_100MBPS_DIV1 5
#define XPAR_PS7_ETHERNET_0_ENET_SLCR_10MBPS_DIV0 8
#define XPAR_PS7_ETHERNET_0_ENET_SLCR_10MBPS_DIV1 50
#define XPAR_PS7_ETHERNET_0_ENET_TSU_CLK_FREQ_HZ 0


/******************************************************************/

#define XPAR_PS7_ETHERNET_0_IS_CACHE_COHERENT 0
#define XPAR_XEMACPS_0_IS_CACHE_COHERENT 0
/* Canonical definitions for peripheral PS7_ETHERNET_0 */
#define XPAR_XEMACPS_0_DEVICE_ID XPAR_PS7_ETHERNET_0_DEVICE_ID
#define XPAR_XEMACPS_0_BASEADDR 0xE000B000
#define XPAR_XEMACPS_0_HIGHADDR 0xE000BFFF
#define XPAR_XEMACPS_0_ENET_CLK_FREQ_HZ 125000000
#define XPAR_XEMACPS_0_ENET_SLCR_1000Mbps_DIV0 8
#define XPAR_XEMACPS_0_ENET_SLCR_1000Mbps_DIV1 1
#define XPAR_XEMACPS_0_ENET_SLCR_100Mbps_DIV0 8
#define XPAR_XEMACPS_0_ENET_SLCR_100Mbps_DIV1 5
#define XPAR_XEMACPS_0_ENET_SLCR_10Mbps_DIV0 8
#define XPAR_XEMACPS_0_ENET_SLCR_10Mbps_DIV1 50
#define XPAR_XEMACPS_0_ENET_TSU_CLK_FREQ_HZ 0


/******************************************************************/


/* Peripheral Definitions for peripheral AXI2REG0165_0 */
#define XPAR_AXI2REG0165_0_BASEADDR 0x43C20000
#define XPAR_AXI2REG0165_0_HIGHADDR 0x43C2FFFF


/* Peripheral Definitions for peripheral AXI2REG0165_1 */
#define XPAR_AXI2REG0165_1_BASEADDR 0x43C30000
#define XPAR_AXI2REG0165_1_HIGHADDR 0x43C3FFFF


/* Peripheral Definitions for peripheral AXI2REG0165_2 */
#define XPAR_AXI2REG0165_2_BASEADDR 0x43C40000
#define XPAR_AXI2REG0165_2_HIGHADDR 0x43C4FFFF


/* Peripheral Definitions for peripheral AXI2REG0165_3 */
#define XPAR_AXI2REG0165_3_BASEADDR 0x43C50000
#define XPAR_AXI2REG0165_3_HIGHADDR 0x43C5FFFF


/* Peripheral Definitions for peripheral AXI2REG0165_4 */
#define XPAR_AXI2REG0165_4_BASEADDR 0x43C60000
#define XPAR_AXI2REG0165_4_HIGHADDR 0x43C6FFFF


/* Peripheral Definitions for peripheral AXI2REG2442_0 */
#define XPAR_AXI2REG2442_0_BASEADDR 0x43C00000
#define XPAR_AXI2REG2442_0_HIGHADDR 0x43C0FFFF


/* Peripheral Definitions for peripheral AXI2REG2492_0 */
#define XPAR_AXI2REG2492_0_BASEADDR 0x43C10000
#define XPAR_AXI2REG2492_0_HIGHADDR 0x43C1FFFF


/* Peripheral Definitions for peripheral AXI2REG3244_0 */
#define XPAR_AXI2REG3244_0_BASEADDR 0x43C70000
#define XPAR_AXI2REG3244_0_HIGHADDR 0x43C7FFFF


/* Peripheral Definitions for peripheral AXI2REGTIMING_0 */
#define XPAR_AXI2REGTIMING_0_BASEADDR 0x43C80000
#define XPAR_AXI2REGTIMING_0_HIGHADDR 0x43C8FFFF


/* Peripheral Definitions for peripheral PS7_AFI_0 */
#define XPAR_PS7_AFI_0_S_AXI_BASEADDR 0xF8008000
#define XPAR_PS7_AFI_0_S_AXI_HIGHADDR 0xF8008FFF


/* Peripheral Definitions for peripheral PS7_AFI_1 */
#define XPAR_PS7_AFI_1_S_AXI_BASEADDR 0xF8009000
#define XPAR_PS7_AFI_1_S_AXI_HIGHADDR 0xF8009FFF


/* Peripheral Definitions for peripheral PS7_AFI_2 */
#define XPAR_PS7_AFI_2_S_AXI_BASEADDR 0xF800A000
#define XPAR_PS7_AFI_2_S_AXI_HIGHADDR 0xF800AFFF


/* Peripheral Definitions for peripheral PS7_AFI_3 */
#define XPAR_PS7_AFI_3_S_AXI_BASEADDR 0xF800B000
#define XPAR_PS7_AFI_3_S_AXI_HIGHADDR 0xF800BFFF


/* Peripheral Definitions for peripheral PS7_DDRC_0 */
#define XPAR_PS7_DDRC_0_S_AXI_BASEADDR 0xF8006000
#define XPAR_PS7_DDRC_0_S_AXI_HIGHADDR 0xF8006FFF


/* Peripheral Definitions for peripheral PS7_GLOBALTIMER_0 */
#define XPAR_PS7_GLOBALTIMER_0_S_AXI_BASEADDR 0xF8F00200
#define XPAR_PS7_GLOBALTIMER_0_S_AXI_HIGHADDR 0xF8F002FF


/* Peripheral Definitions for peripheral PS7_GPV_0 */
#define XPAR_PS7_GPV_0_S_AXI_BASEADDR 0xF8900000
#define XPAR_PS7_GPV_0_S_AXI_HIGHADDR 0xF89FFFFF


/* Peripheral Definitions for peripheral PS7_INTC_DIST_0 */
#define XPAR_PS7_INTC_DIST_0_S_AXI_BASEADDR 0xF8F01000
#define XPAR_PS7_INTC_DIST_0_S_AXI_HIGHADDR 0xF8F01FFF


/* Peripheral Definitions for peripheral PS7_IOP_BUS_CONFIG_0 */
#define XPAR_PS7_IOP_BUS_CONFIG_0_S_AXI_BASEADDR 0xE0200000
#define XPAR_PS7_IOP_BUS_CONFIG_0_S_AXI_HIGHADDR 0xE0200FFF


/* Peripheral Definitions for peripheral PS7_L2CACHEC_0 */
#define XPAR_PS7_L2CACHEC_0_S_AXI_BASEADDR 0xF8F02000
#define XPAR_PS7_L2CACHEC_0_S_AXI_HIGHADDR 0xF8F02FFF


/* Peripheral Definitions for peripheral PS7_OCMC_0 */
#define XPAR_PS7_OCMC_0_S_AXI_BASEADDR 0xF800C000
#define XPAR_PS7_OCMC_0_S_AXI_HIGHADDR 0xF800CFFF


/* Peripheral Definitions for peripheral PS7_PL310_0 */
#define XPAR_PS7_PL310_0_S_AXI_BASEADDR 0xF8F02000
#define XPAR_PS7_PL310_0_S_AXI_HIGHADDR 0xF8F02FFF


/* Peripheral Definitions for peripheral PS7_PMU_0 */
#define XPAR_PS7_PMU_0_S_AXI_BASEADDR 0xF8891000
#define XPAR_PS7_PMU_0_S_AXI_HIGHADDR 0xF8891FFF
#define XPAR_PS7_PMU_0_PMU1_S_AXI_BASEADDR 0xF8893000
#define XPAR_PS7_PMU_0_PMU1_S_AXI_HIGHADDR 0xF8893FFF


/* Peripheral Definitions for peripheral PS7_QSPI_LINEAR_0 */
#define XPAR_PS7_QSPI_LINEAR_0_S_AXI_BASEADDR 0xFC000000
#define XPAR_PS7_QSPI_LINEAR_0_S_AXI_HIGHADDR 0xFCFFFFFF


/* Peripheral Definitions for peripheral PS7_RAM_0 */
#define XPAR_PS7_RAM_0_S_AXI_BASEADDR 0x00000000
#define XPAR_PS7_RAM_0_S_AXI_HIGHADDR 0x0003FFFF


/* Peripheral Definitions for peripheral PS7_RAM_1 */
#define XPAR_PS7_RAM_1_S_AXI_BASEADDR 0xFFFC0000
#define XPAR_PS7_RAM_1_S_AXI_HIGHADDR 0xFFFFFFFF


/* Peripheral Definitions for peripheral PS7_SCUC_0 */
#define XPAR_PS7_SCUC_0_S_AXI_BASEADDR 0xF8F00000
#define XPAR_PS7_SCUC_0_S_AXI_HIGHADDR 0xF8F000FC


/* Peripheral Definitions for peripheral PS7_SLCR_0 */
#define XPAR_PS7_SLCR_0_S_AXI_BASEADDR 0xF8000000
#define XPAR_PS7_SLCR_0_S_AXI_HIGHADDR 0xF8000FFF


/* Peripheral Definitions for peripheral PLVERSION */
#define XPAR_PLVERSION_BASEADDR 0x43C90000
#define XPAR_PLVERSION_HIGHADDR 0x43C9FFFF


/******************************************************************/
























































/* Canonical Definitions for peripheral PLVERSION */
#define XPAR_AXI2REGVERSION_0_BASEADDR 0x43C90000
#define XPAR_AXI2REGVERSION_0_HIGHADDR 0x43C9FFFF


/******************************************************************/

/* Definitions for driver GPIO */
#define XPAR_XGPIO_NUM_INSTANCES 5

/* Definitions for peripheral AXI_GPIO_0_RDWR_MODE34_MSELECT */
#define XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_BASEADDR 0x41200000
#define XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_HIGHADDR 0x4120FFFF
#define XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_DEVICE_ID 0
#define XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_IS_DUAL 1


/* Definitions for peripheral AXI_GPIO_2492_RDWR_NUM */
#define XPAR_AXI_GPIO_2492_RDWR_NUM_BASEADDR 0x41230000
#define XPAR_AXI_GPIO_2492_RDWR_NUM_HIGHADDR 0x4123FFFF
#define XPAR_AXI_GPIO_2492_RDWR_NUM_DEVICE_ID 1
#define XPAR_AXI_GPIO_2492_RDWR_NUM_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_2492_RDWR_NUM_IS_DUAL 0


/* Definitions for peripheral AXI_GPIO_DMA_RECEIVE_RDY */
#define XPAR_AXI_GPIO_DMA_RECEIVE_RDY_BASEADDR 0x41210000
#define XPAR_AXI_GPIO_DMA_RECEIVE_RDY_HIGHADDR 0x4121FFFF
#define XPAR_AXI_GPIO_DMA_RECEIVE_RDY_DEVICE_ID 2
#define XPAR_AXI_GPIO_DMA_RECEIVE_RDY_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_DMA_RECEIVE_RDY_IS_DUAL 0


/* Definitions for peripheral AXI_GPIO_DEBUG */
#define XPAR_AXI_GPIO_DEBUG_BASEADDR 0x41220000
#define XPAR_AXI_GPIO_DEBUG_HIGHADDR 0x4122FFFF
#define XPAR_AXI_GPIO_DEBUG_DEVICE_ID 3
#define XPAR_AXI_GPIO_DEBUG_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_DEBUG_IS_DUAL 0


/* Definitions for peripheral AXI_GPIO_AWMF_MODE */
#define XPAR_AXI_GPIO_AWMF_MODE_BASEADDR 0x41240000
#define XPAR_AXI_GPIO_AWMF_MODE_HIGHADDR 0x4124FFFF
#define XPAR_AXI_GPIO_AWMF_MODE_DEVICE_ID 4
#define XPAR_AXI_GPIO_AWMF_MODE_INTERRUPT_PRESENT 0
#define XPAR_AXI_GPIO_AWMF_MODE_IS_DUAL 0


/******************************************************************/

/* Canonical definitions for peripheral AXI_GPIO_0_RDWR_MODE34_MSELECT */
#define XPAR_GPIO_0_BASEADDR 0x41200000
#define XPAR_GPIO_0_HIGHADDR 0x4120FFFF
#define XPAR_GPIO_0_DEVICE_ID XPAR_AXI_GPIO_0_RDWR_MODE34_MSELECT_DEVICE_ID
#define XPAR_GPIO_0_INTERRUPT_PRESENT 0
#define XPAR_GPIO_0_IS_DUAL 1

/* Canonical definitions for peripheral AXI_GPIO_2492_RDWR_NUM */
#define XPAR_GPIO_1_BASEADDR 0x41230000
#define XPAR_GPIO_1_HIGHADDR 0x4123FFFF
#define XPAR_GPIO_1_DEVICE_ID XPAR_AXI_GPIO_2492_RDWR_NUM_DEVICE_ID
#define XPAR_GPIO_1_INTERRUPT_PRESENT 0
#define XPAR_GPIO_1_IS_DUAL 0

/* Canonical definitions for peripheral AXI_GPIO_DMA_RECEIVE_RDY */
#define XPAR_GPIO_2_BASEADDR 0x41210000
#define XPAR_GPIO_2_HIGHADDR 0x4121FFFF
#define XPAR_GPIO_2_DEVICE_ID XPAR_AXI_GPIO_DMA_RECEIVE_RDY_DEVICE_ID
#define XPAR_GPIO_2_INTERRUPT_PRESENT 0
#define XPAR_GPIO_2_IS_DUAL 0

/* Canonical definitions for peripheral AXI_GPIO_DEBUG */
#define XPAR_GPIO_3_BASEADDR 0x41220000
#define XPAR_GPIO_3_HIGHADDR 0x4122FFFF
#define XPAR_GPIO_3_DEVICE_ID XPAR_AXI_GPIO_DEBUG_DEVICE_ID
#define XPAR_GPIO_3_INTERRUPT_PRESENT 0
#define XPAR_GPIO_3_IS_DUAL 0

/* Canonical definitions for peripheral AXI_GPIO_AWMF_MODE */
#define XPAR_GPIO_4_BASEADDR 0x41240000
#define XPAR_GPIO_4_HIGHADDR 0x4124FFFF
#define XPAR_GPIO_4_DEVICE_ID XPAR_AXI_GPIO_AWMF_MODE_DEVICE_ID
#define XPAR_GPIO_4_INTERRUPT_PRESENT 0
#define XPAR_GPIO_4_IS_DUAL 0


/******************************************************************/

/* Definitions for driver GPIOPS */
#define XPAR_XGPIOPS_NUM_INSTANCES 1

/* Definitions for peripheral PS7_GPIO_0 */
#define XPAR_PS7_GPIO_0_DEVICE_ID 0
#define XPAR_PS7_GPIO_0_BASEADDR 0xE000A000
#define XPAR_PS7_GPIO_0_HIGHADDR 0xE000AFFF


/******************************************************************/

/* Canonical definitions for peripheral PS7_GPIO_0 */
#define XPAR_XGPIOPS_0_DEVICE_ID XPAR_PS7_GPIO_0_DEVICE_ID
#define XPAR_XGPIOPS_0_BASEADDR 0xE000A000
#define XPAR_XGPIOPS_0_HIGHADDR 0xE000AFFF


/******************************************************************/

/* Definitions for driver QSPIPS */
#define XPAR_XQSPIPS_NUM_INSTANCES 1

/* Definitions for peripheral PS7_QSPI_0 */
#define XPAR_PS7_QSPI_0_DEVICE_ID 0
#define XPAR_PS7_QSPI_0_BASEADDR 0xE000D000
#define XPAR_PS7_QSPI_0_HIGHADDR 0xE000DFFF
#define XPAR_PS7_QSPI_0_QSPI_CLK_FREQ_HZ 142857132
#define XPAR_PS7_QSPI_0_QSPI_MODE 0
#define XPAR_PS7_QSPI_0_QSPI_BUS_WIDTH 2


/******************************************************************/

/* Canonical definitions for peripheral PS7_QSPI_0 */
#define XPAR_XQSPIPS_0_DEVICE_ID XPAR_PS7_QSPI_0_DEVICE_ID
#define XPAR_XQSPIPS_0_BASEADDR 0xE000D000
#define XPAR_XQSPIPS_0_HIGHADDR 0xE000DFFF
#define XPAR_XQSPIPS_0_QSPI_CLK_FREQ_HZ 142857132
#define XPAR_XQSPIPS_0_QSPI_MODE 0
#define XPAR_XQSPIPS_0_QSPI_BUS_WIDTH 2


/******************************************************************/

/* Definitions for Fabric interrupts connected to ps7_scugic_0 */
#define XPAR_FABRIC_AXI_DMA_ADC3244_S2MM_INTROUT_INTR 68U
#define XPAR_FABRIC_AXI_DMA_RDM_S2MM_INTROUT_INTR 84U

/******************************************************************/

/* Canonical definitions for Fabric interrupts connected to ps7_scugic_0 */
#define XPAR_FABRIC_AXIDMA_0_VEC_ID XPAR_FABRIC_AXI_DMA_ADC3244_S2MM_INTROUT_INTR
#define XPAR_FABRIC_AXIDMA_1_VEC_ID XPAR_FABRIC_AXI_DMA_RDM_S2MM_INTROUT_INTR

/******************************************************************/

/* Definitions for driver SCUGIC */
#define XPAR_XSCUGIC_NUM_INSTANCES 1U

/* Definitions for peripheral PS7_SCUGIC_0 */
#define XPAR_PS7_SCUGIC_0_DEVICE_ID 0U
#define XPAR_PS7_SCUGIC_0_BASEADDR 0xF8F00100U
#define XPAR_PS7_SCUGIC_0_HIGHADDR 0xF8F001FFU
#define XPAR_PS7_SCUGIC_0_DIST_BASEADDR 0xF8F01000U


/******************************************************************/

/* Canonical definitions for peripheral PS7_SCUGIC_0 */
#define XPAR_SCUGIC_0_DEVICE_ID 0U
#define XPAR_SCUGIC_0_CPU_BASEADDR 0xF8F00100U
#define XPAR_SCUGIC_0_CPU_HIGHADDR 0xF8F001FFU
#define XPAR_SCUGIC_0_DIST_BASEADDR 0xF8F01000U


/******************************************************************/

/* Definitions for driver SCUTIMER */
#define XPAR_XSCUTIMER_NUM_INSTANCES 1

/* Definitions for peripheral PS7_SCUTIMER_0 */
#define XPAR_PS7_SCUTIMER_0_DEVICE_ID 0
#define XPAR_PS7_SCUTIMER_0_BASEADDR 0xF8F00600
#define XPAR_PS7_SCUTIMER_0_HIGHADDR 0xF8F0061F


/******************************************************************/

/* Canonical definitions for peripheral PS7_SCUTIMER_0 */
#define XPAR_XSCUTIMER_0_DEVICE_ID XPAR_PS7_SCUTIMER_0_DEVICE_ID
#define XPAR_XSCUTIMER_0_BASEADDR 0xF8F00600
#define XPAR_XSCUTIMER_0_HIGHADDR 0xF8F0061F


/******************************************************************/

/* Definitions for driver SCUWDT */
#define XPAR_XSCUWDT_NUM_INSTANCES 1

/* Definitions for peripheral PS7_SCUWDT_0 */
#define XPAR_PS7_SCUWDT_0_DEVICE_ID 0
#define XPAR_PS7_SCUWDT_0_BASEADDR 0xF8F00620
#define XPAR_PS7_SCUWDT_0_HIGHADDR 0xF8F006FF


/******************************************************************/

/* Canonical definitions for peripheral PS7_SCUWDT_0 */
#define XPAR_SCUWDT_0_DEVICE_ID XPAR_PS7_SCUWDT_0_DEVICE_ID
#define XPAR_SCUWDT_0_BASEADDR 0xF8F00620
#define XPAR_SCUWDT_0_HIGHADDR 0xF8F006FF


/******************************************************************/

/* Definitions for driver UARTPS */
#define XPAR_XUARTPS_NUM_INSTANCES 1

/* Definitions for peripheral PS7_UART_1 */
#define XPAR_PS7_UART_1_DEVICE_ID 0
#define XPAR_PS7_UART_1_BASEADDR 0xE0001000
#define XPAR_PS7_UART_1_HIGHADDR 0xE0001FFF
#define XPAR_PS7_UART_1_UART_CLK_FREQ_HZ 100000000
#define XPAR_PS7_UART_1_HAS_MODEM 0


/******************************************************************/

/* Canonical definitions for peripheral PS7_UART_1 */
#define XPAR_XUARTPS_0_DEVICE_ID XPAR_PS7_UART_1_DEVICE_ID
#define XPAR_XUARTPS_0_BASEADDR 0xE0001000
#define XPAR_XUARTPS_0_HIGHADDR 0xE0001FFF
#define XPAR_XUARTPS_0_UART_CLK_FREQ_HZ 100000000
#define XPAR_XUARTPS_0_HAS_MODEM 0


/******************************************************************/

/* Definition for input Clock */
/* Definitions for driver XADCPS */
#define XPAR_XADCPS_NUM_INSTANCES 1

/* Definitions for peripheral PS7_XADC_0 */
#define XPAR_PS7_XADC_0_DEVICE_ID 0
#define XPAR_PS7_XADC_0_BASEADDR 0xF8007100
#define XPAR_PS7_XADC_0_HIGHADDR 0xF8007120


/******************************************************************/

/* Canonical definitions for peripheral PS7_XADC_0 */
#define XPAR_XADCPS_0_DEVICE_ID XPAR_PS7_XADC_0_DEVICE_ID
#define XPAR_XADCPS_0_BASEADDR 0xF8007100
#define XPAR_XADCPS_0_HIGHADDR 0xF8007120


/******************************************************************/

/* Xilinx FAT File System Library (XilFFs) User Settings */
#define FILE_SYSTEM_USE_MKFS
#define FILE_SYSTEM_NUM_LOGIC_VOL 2
#define FILE_SYSTEM_USE_STRFUNC 0
#define FILE_SYSTEM_SET_FS_RPATH 0
#define FILE_SYSTEM_WORD_ACCESS
#endif  /* end of protection macro */
