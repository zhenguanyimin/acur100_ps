
#include "ut_chn_ctrl.h"

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

#include "../../drv/awmf_0165/awmf_0165_drv.h"
#include "../stub/stub_code.h"

#define UT_CHN_CTRL_USE_STUB_API (0)

static BaseType_t tx_chn_set_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t tx_chn_get_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t rx_chn_set_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t rx_chn_get_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t close_all_chn_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static const CLI_Command_Definition_t tx_chn_set_cmd =
{
	"TxChnSet",
	"\r\nTxChnSet <RowNum> <ColNum> <ChnState> [<PhaseCode>]:\r\n Set transmitter channel parameters\r\n",
	tx_chn_set_cmd_handler,
	-1
};

static const CLI_Command_Definition_t tx_chn_get_cmd =
{
	"TxChnGet",
	"\r\nTxChnGet <RowNum> <ColNum>:\r\n Get transmitter channel parameters\r\n",
	tx_chn_get_cmd_handler,
	-1
};

static const CLI_Command_Definition_t rx_chn_set_cmd =
{
	"RxChnSet",
	"\r\nRxChnSet <RowNum> <ColNum> <ChnState> [<PhaseCode> <AmpComCode> <AmpChnCode>]:\r\n Set receiver channel parameters\r\n",
	rx_chn_set_cmd_handler,
	-1
};

static const CLI_Command_Definition_t rx_chn_get_cmd =
{
	"RxChnGet",
	"\r\nRxChnGet <RowNum> <ColNum>:\r\n Get receiver channel parameters\r\n",
	rx_chn_get_cmd_handler,
	-1
};

static const CLI_Command_Definition_t close_all_chn_cmd =
{
	"CloseAllChn",
	"\r\nCloseAllChn:\r\n Close all of the transmitter and receiver channels\r\n",
	close_all_chn_cmd_handler,
	0
};

static BaseType_t tx_chn_set_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0, len3 = 0, len4 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
	const char *param4 = FreeRTOS_CLIGetParameter(pcCommandString, 4, &len4);
	uint32_t row_num = 0, col_num = 0, chn_state = 0, phase_code = 0;
	awmf_0165_chn_info_t chn_info = {0};
	ret_code_t ret = RET_OK;

	if ((len1 > 0) && (len2 > 0) && (len3 > 0))
	{
		row_num = (uint32_t)strtol(param1, NULL, 0);
		col_num = (uint32_t)strtol(param2, NULL, 0);
		chn_state = (uint32_t)strtol(param3, NULL, 0);
		if (len4 > 0)
		{
			phase_code = (uint32_t)strtol(param4, NULL, 0);
		}

#if UT_CHN_CTRL_USE_STUB_API
		(void)stub_code_tx_chn_set(row_num, col_num, chn_state, phase_code);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
#else
		chn_info.chn_state = chn_state;
		chn_info.phase_code = phase_code;
		chn_info.amp_com_code = 0;
		chn_info.amp_chn_code = 0;
		ret = awmf_0165_drv_tx_chn_set(row_num, col_num, &chn_info);
		if (ret == RET_OK)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}
		else
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
		}
#endif /* UT_CHN_CTRL_USE_STUB_API */
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t tx_chn_get_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	uint32_t row_num = 0, col_num = 0;
#if UT_CHN_CTRL_USE_STUB_API
	uint32_t chn_state = 0, phase_code = 0;
#else
	awmf_0165_chn_info_t chn_info = {0};
	ret_code_t ret = RET_OK;
#endif /* UT_CHN_CTRL_USE_STUB_API */

	if ((len1 > 0) && (len2 > 0))
	{
		row_num = (uint32_t)strtol(param1, NULL, 0);
		col_num = (uint32_t)strtol(param2, NULL, 0);

#if UT_CHN_CTRL_USE_STUB_API
		(void)stub_code_tx_chn_get(row_num, col_num, &chn_state, &phase_code);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK\r\n%lu %lu %lu %lu", row_num, col_num, chn_state, phase_code);
#else
		ret = awmf_0165_drv_tx_chn_get(row_num, col_num, &chn_info);
		if (ret == RET_OK)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK\r\n%lu %lu %lu %lu", row_num, col_num, chn_info.chn_state, chn_info.phase_code);
		}
		else
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
		}
#endif /* UT_CHN_CTRL_USE_STUB_API */
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t rx_chn_set_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0, len3 = 0, len4 = 0, len5 = 0, len6 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
	const char *param4 = FreeRTOS_CLIGetParameter(pcCommandString, 4, &len4);
	const char *param5 = FreeRTOS_CLIGetParameter(pcCommandString, 5, &len5);
	const char *param6 = FreeRTOS_CLIGetParameter(pcCommandString, 6, &len6);
	uint32_t row_num = 0, col_num = 0, chn_state = 0, phase_code = 0, amp_com_code = 0, amp_chn_code = 0;
	awmf_0165_chn_info_t chn_info = {0};
	ret_code_t ret = RET_OK;

	if ((len1 > 0) && (len2 > 0) && (len3 > 0))
	{
		row_num = (uint32_t)strtol(param1, NULL, 0);
		col_num = (uint32_t)strtol(param2, NULL, 0);
		chn_state = (uint32_t)strtol(param3, NULL, 0);
		if (len4 > 0)
		{
			phase_code = (uint32_t)strtol(param4, NULL, 0);
			if (len5 > 0)
			{
				amp_com_code = (uint32_t)strtol(param5, NULL, 0);
				if (len6 > 0)
				{
					amp_chn_code = (uint32_t)strtol(param6, NULL, 0);
				}
			}
		}

#if UT_CHN_CTRL_USE_STUB_API
		(void)stub_code_rx_chn_set(row_num, col_num, chn_state, phase_code, amp_com_code, amp_chn_code);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
#else
		chn_info.chn_state = chn_state;
		chn_info.phase_code = phase_code;
		chn_info.amp_com_code = amp_com_code;
		chn_info.amp_chn_code = amp_chn_code;
		ret = awmf_0165_drv_rx_chn_set(row_num, col_num, &chn_info);
		if (ret == RET_OK)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}
		else
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
		}
#endif /* UT_CHN_CTRL_USE_STUB_API */
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t rx_chn_get_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	uint32_t row_num = 0, col_num = 0;
#if UT_CHN_CTRL_USE_STUB_API
	uint32_t chn_state = 0, phase_code = 0, amp_com_code = 0, amp_chn_code = 0;
#else
	awmf_0165_chn_info_t chn_info = {0};
	ret_code_t ret = RET_OK;
#endif /* UT_CHN_CTRL_USE_STUB_API */

	if ((len1 > 0) && (len2 > 0))
	{
		row_num = (uint32_t)strtol(param1, NULL, 0);
		col_num = (uint32_t)strtol(param2, NULL, 0);

#if UT_CHN_CTRL_USE_STUB_API
		(void)stub_code_rx_chn_get(row_num, col_num, &chn_state, &phase_code, &amp_com_code, &amp_chn_code);
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK\r\n%lu %lu %lu %lu %lu %lu", row_num, col_num, chn_state, phase_code, amp_com_code, amp_chn_code);
#else
		ret = awmf_0165_drv_rx_chn_get(row_num, col_num, &chn_info);
		if (ret == RET_OK)
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK\r\n%lu %lu %lu %lu %lu %lu", row_num, col_num,
				chn_info.chn_state, chn_info.phase_code, chn_info.amp_com_code, chn_info.amp_chn_code);
		}
		else
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
		}
#endif /* UT_CHN_CTRL_USE_STUB_API */
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE;
}

static BaseType_t close_all_chn_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	ret_code_t ret = RET_OK;

#if UT_CHN_CTRL_USE_STUB_API
	(void)stub_code_close_all_chn();
	snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
#else
	ret = awmf_0165_drv_close_all_chn();
	if (ret == RET_OK)
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
	}
	else
	{
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}
#endif /* UT_CHN_CTRL_USE_STUB_API */

	return pdFALSE;
}

void ut_chn_ctrl_init(void)
{
	FreeRTOS_CLIRegisterCommand(&tx_chn_set_cmd);
	FreeRTOS_CLIRegisterCommand(&tx_chn_get_cmd);
	FreeRTOS_CLIRegisterCommand(&rx_chn_set_cmd);
	FreeRTOS_CLIRegisterCommand(&rx_chn_get_cmd);
	FreeRTOS_CLIRegisterCommand(&close_all_chn_cmd);

	return;
}
