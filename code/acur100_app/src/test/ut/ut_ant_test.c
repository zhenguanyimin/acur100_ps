
#include "ut_ant_test.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "../../srv/cli/cli_if.h"

#include "../stub/stub_code.h"
#include "../../app/beam_calculate/bk_cal.h"

static BaseType_t tx_ant_test_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t rx_ant_test_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t tx_amp_set_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t tx_beam_query_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static const CLI_Command_Definition_t tx_ant_test_cmd =
{
	"TxAntTest",
	"\r\nTxAntTest <FreqNum> <AzDeg> <AlDeg>:\r\n Test transmitter antennas\r\n",
	tx_ant_test_cmd_handler,
	3
};

static const CLI_Command_Definition_t rx_ant_test_cmd =
{
	"RxAntTest",
	"\r\nRxAntTest <FreqNum> <AzDeg> <AlDeg>:\r\n Test receiver antennas\r\n",
	rx_ant_test_cmd_handler,
	3
};

static const CLI_Command_Definition_t tx_amp_set_cmd =
{
	"TxAmpSet",
	"\r\nTxAmpSet <comVal> <rfxVal>:\r\n Set 0165 Tx amp code\r\n",
	tx_amp_set_cmd_handler,
	2
};

static const CLI_Command_Definition_t tx_beam_qry_cmd =
{
	"TxbeamQuy",
	"\r\nTxbeamQuy:\r\n Query 0165 transmitting beam control info\r\n",
	tx_beam_query_cmd_handler,
	0
};


static BaseType_t tx_ant_test_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0, len3 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
	uint8_t fre_num = 0;
	int16_t az_deg = 0, al_deg = 0;

	if ((len1 > 0) && (len2 > 0) && (len3 > 0))
	{
		fre_num = (uint8_t)strtoul(param1, NULL, 0);
		az_deg = (int16_t)strtol(param2, NULL, 0);
		al_deg = (int16_t)strtol(param3, NULL, 0);

		bk_cak_func(fre_num, az_deg, al_deg);
		bk_tran_func(TRUE);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t rx_ant_test_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0, len3 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
	uint8_t fre_num = 0;
	int16_t az_deg = 0, al_deg = 0;

	if ((len1 > 0) && (len2 > 0) && (len3 > 0))
	{
		fre_num = (uint8_t)strtoul(param1, NULL, 0);
		az_deg = (int16_t)strtol(param2, NULL, 0);
		al_deg = (int16_t)strtol(param3, NULL, 0);

		bk_cak_func(fre_num, az_deg, al_deg);
		bk_tran_func(FALSE);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t tx_amp_set_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);

	uint16_t com_val = 0, rfx_val = 0;

	if ((len1 > 0) && (len2 > 0))
	{
		com_val = (uint16_t)strtoul(param1, NULL, 0);
		rfx_val = (uint16_t)strtoul(param2, NULL, 0);

		tx_amp_code_setting(com_val, rfx_val);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t tx_beam_query_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	int16_t azimuth_angle = 0, elevation_angle = 0;
	ret_code_t ret = RET_OK;

	ret = beam_control_qureying(&azimuth_angle, &elevation_angle);
	if (ret == RET_OK)
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "Currently azimuth angle is %d, elevation angle is %d.", azimuth_angle, elevation_angle);
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}


	return pdFALSE;
}

void ut_ant_test_init(void)
{
	FreeRTOS_CLIRegisterCommand(&tx_ant_test_cmd);
	FreeRTOS_CLIRegisterCommand(&rx_ant_test_cmd);
	FreeRTOS_CLIRegisterCommand(&tx_amp_set_cmd);
	FreeRTOS_CLIRegisterCommand(&tx_beam_qry_cmd);

	return;
}
