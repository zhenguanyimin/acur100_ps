

/* FreeRTOS includes. */
#include "ut_track_test.h"

#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "../../srv/cli/cli_if.h"
#include "../../app/app_init.h"

extern uint32_t gBeamMode ;

s32 track_set_beam_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0 ;
	s32 type = -1;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);

	if(len1>0)
	{
		type = strtoul(param1, NULL, 0);

		if ((1==type)||(2==type))
		{
			gBeamMode = type ;
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}else
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	}else
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	return pdFALSE ;
}

s32 track_set_beam_azi_scan_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0;
	s32 center = -1;
	s32 scope = -1;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);

	if ((len1 > 0) && (len2 > 0))
	{
		center = strtoul(param1, NULL, 0);
		scope = strtoul(param2, NULL, 0);

		if (((center >= -60) && (center <= 60)) && ((scope >= 0) && (scope <= 120)))
		{
			gConfigParmInfo[0].aziScanCenter = center;
			gConfigParmInfo[0].aziScanScope = scope;
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}else
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	}else
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	return pdFALSE ;
}

s32 track_set_beam_ele_scan_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0;
	s32 center = -1;
	s32 scope = -1;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);

	if ((len1 > 0) && (len2 > 0))
	{
		center = strtoul(param1, NULL, 0);
		scope = strtoul(param2, NULL, 0);

		if (((center >= -20) && (center <= 20)) && ((scope >= 0) && (scope <= 40)))
		{
			gConfigParmInfo[0].eleScanCenter = center;
			gConfigParmInfo[0].eleScanScope = scope;
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}else
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	}else
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");

	return pdFALSE ;
}

s32 track_set_fix_beam_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0;
	s32 eleCenter = -1;
	s32 aziCenter = -1;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);

	if ((len1 > 0) && (len2 > 0))
	{
		eleCenter = strtoul(param1, NULL, 0);
		aziCenter = strtoul(param2, NULL, 0);

		if (((eleCenter >= -20) && (eleCenter <= 20)) && ((aziCenter >= -60) && (aziCenter <= 60)))
		{
			gBeamMode = 1;
			gConfigParmInfo[0].eleScanCenter = eleCenter;
			gConfigParmInfo[0].aziScanCenter = aziCenter;
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}
		else
		{
			gBeamMode = 2;
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
		}
	}
	else
	{
		gBeamMode = 2;
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE ;
}

s32 track_set_scan_beam_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t len1 = 0, len2 = 0, len3 = 0, len4 = 0;
	s32 eleCenter = -1;
	s32 eleScope = -1;
	s32 aziCenter = -1;
	s32 aziScope = -1;
	const char *param1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &len1);
	const char *param2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &len2);
	const char *param3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &len3);
	const char *param4 = FreeRTOS_CLIGetParameter(pcCommandString, 4, &len4);

	if ((len1 > 0) && (len2 > 0) && (len3 > 0) && (len4 > 0))
	{
		eleCenter = strtoul(param1, NULL, 0);
		eleScope = strtoul(param2, NULL, 0);
		aziCenter = strtoul(param3, NULL, 0);
		aziScope = strtoul(param4, NULL, 0);

		if (((eleCenter >= -20) && (eleCenter <= 20)) && ((eleScope >= 0) && (eleScope <= 40)) && \
			((aziCenter >= -60) && (aziCenter <= 60)) && ((aziScope >= 0) && (aziScope <= 120)))
		{
			gBeamMode = 2;
			gConfigParmInfo[0].eleScanCenter = eleCenter;
			gConfigParmInfo[0].eleScanScope = eleScope;
			gConfigParmInfo[0].aziScanCenter = aziCenter;
			gConfigParmInfo[0].aziScanScope = aziScope;
			snprintf(pcWriteBuffer, xWriteBufferLen, "OK");
		}
		else
		{
			gBeamMode = 1;
			snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
		}
	}
	else
	{
		gBeamMode = 1;
		snprintf(pcWriteBuffer, xWriteBufferLen, "ERROR");
	}

	return pdFALSE ;
}

static const CLI_Command_Definition_t track_set_beam =
{
	"TrackBeamSet",
	"\r\n TrackBeamSet <type>:\r\n type:1 for fix beamform ,2 for beam scanning\r\n",
	track_set_beam_handler,
	1
};

static const CLI_Command_Definition_t track_set_beam_azi_scan =
{
	"TrackBeamAziScanSet",
	"\r\n TrackBeamAziScanSet <center> <scope>:\r\n set azimuth scanning center and scope\r\n",
	track_set_beam_azi_scan_handler,
	2
};

static const CLI_Command_Definition_t track_set_beam_ele_scan =
{
	"TrackBeamEleScanSet",
	"\r\n TrackBeamEleScanSet <center> <scope>:\r\n set elevation scanning center and scope\r\n",
	track_set_beam_ele_scan_handler,
	2
};

static const CLI_Command_Definition_t track_set_fix_beam =
{
	"TrackFixBeamSet",
	"\r\n TrackFixBeamSet <eleCenter> <aziCenter>:\r\n set fix beam center\r\n",
	track_set_fix_beam_handler,
	2
};

static const CLI_Command_Definition_t track_set_scan_beam =
{
	"TrackScanBeamSet",
	"\r\n TrackScanBeamSet <eleCenter> <eleScope> <aziCenter> <aziScope>:\r\n set scan beam center and scope\r\n",
	track_set_scan_beam_handler,
	4
};

void ut_track_init(void)
{
	FreeRTOS_CLIRegisterCommand(&track_set_beam);
	FreeRTOS_CLIRegisterCommand(&track_set_beam_azi_scan);
	FreeRTOS_CLIRegisterCommand(&track_set_beam_ele_scan);
	FreeRTOS_CLIRegisterCommand(&track_set_fix_beam);
	FreeRTOS_CLIRegisterCommand(&track_set_scan_beam);
}
