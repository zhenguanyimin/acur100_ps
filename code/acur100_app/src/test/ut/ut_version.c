
#include "ut_version.h"

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

#include "../../srv/version/version.h"

static BaseType_t version_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static const CLI_Command_Definition_t version_cmd =
{
	"version",
	"\r\nversion:\r\n Show version info\r\n",
	version_cmd_handler,
	0
};

static BaseType_t version_cmd_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *embed_software_ps_version_string = NULL;

	embed_software_ps_version_string = get_embed_software_ps_version_string();
	snprintf(pcWriteBuffer, xWriteBufferLen, "%s (%s %s)", embed_software_ps_version_string, __DATE__, __TIME__);

	return pdFALSE;
}

void ut_version_init(void)
{
	FreeRTOS_CLIRegisterCommand(&version_cmd);
}
