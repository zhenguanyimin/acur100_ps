
#ifndef CLI_IF_H
#define CLI_IF_H

#include "FreeRTOS_CLI.h"

extern void vRegisterSampleCLICommands(void);

extern void vUARTCommandConsoleStart(uint16_t usStackSize, UBaseType_t uxPriority);

extern void vBasicSocketsCommandInterpreterTask(void *pvParameters);

#endif /* CLI_IF_H */
