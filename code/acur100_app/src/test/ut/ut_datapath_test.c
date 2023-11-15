

#include "ut_0165_drv.h"

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
#include "../../app/DataPath/data_path.h"

#include "ut_dataPath_test.h"

s32 datapath_output_en_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0 ;
	s32 type = -1;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);

	if(len1>0)
	{
		type = strtoul(param1, NULL, 0);
		if (1==type)
		{
			data_path_start(ADC_DATA); //adc
			data_path_out_en(ADC_DATA);
		}else if ((2==type)||(3==type))
		{
			data_path_start(RDMAP_DATA); //rdm
			data_path_out_en(type);
		}

		if ((1==type)||(2==type)||(3==type))
		{
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}else
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	}else
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	return pdFALSE ;
}

static const CLI_Command_Definition_t datapath_out_enable =
{
	"DatapathOutEn",
	"\r\DatapathOutEn <type>:\r\n type:0 for disable ,1 for adc ,2 for rdmap\r\n",
	datapath_output_en_handler,
	1
};

void ut_datapath_init(void)
{
	FreeRTOS_CLIRegisterCommand(&datapath_out_enable);
}
