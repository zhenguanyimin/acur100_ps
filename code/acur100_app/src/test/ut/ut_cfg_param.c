
#include "ut_cfg_param.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../srv/log/log.h"

/* Service module includes. */
#include "../../srv/protocol/protocol_parameter.h"

PRIVATE void ut_cfg_param_dump(protocol_cfg_param_t *cfg)
{
    LOG_PRINTF("trSwitchCtrl:           %u\r\n", cfg->trSwitchCtrl);
    LOG_PRINTF("workMode:               %u\r\n", cfg->workMode);
    LOG_PRINTF("workWaveCode:           %u\r\n", cfg->workWaveCode);
    LOG_PRINTF("workFrqCode:            %u\r\n", cfg->workFrqCode);
    LOG_PRINTF("prfPeriod:              %u\r\n", cfg->prfPeriod);
    LOG_PRINTF("accuNum:                %u\r\n", cfg->accuNum);

    LOG_PRINTF("noiseCoef:              %u\r\n", cfg->noiseCoef);
    LOG_PRINTF("clutterCoef:            %u\r\n", cfg->clutterCoef);
    LOG_PRINTF("cfarCoef:               %u\r\n", cfg->cfarCoef);

    LOG_PRINTF("focusRangeMin:          %u\r\n", cfg->focusRangeMin);
    LOG_PRINTF("focusRangeMax:          %u\r\n", cfg->focusRangeMax);

    LOG_PRINTF("clutterCurveNum:        %d\r\n", cfg->clutterCurveNum);
    LOG_PRINTF("lobeCompCoef:           %d\r\n", cfg->lobeCompCoef);

    LOG_PRINTF("cohesionVelThre:        %u\r\n", cfg->cohesionVelThre);
    LOG_PRINTF("cohesionRgnThre:        %u\r\n", cfg->cohesionRgnThre);
    LOG_PRINTF("clutterMapSwitch:       %u\r\n", cfg->clutterMapSwitch);
    LOG_PRINTF("clutterMapUpdateCoef:   %u\r\n", cfg->clutterMapUpdateCoef);

    LOG_PRINTF("aziCalcSlope:           %d\r\n", cfg->aziCalcSlope);
    LOG_PRINTF("aziCalcPhase:           %d\r\n", cfg->aziCalcPhase);
    LOG_PRINTF("eleCalcSlope:           %d\r\n", cfg->eleCalcSlope);
    LOG_PRINTF("eleCalcPhase:           %d\r\n", cfg->eleCalcPhase);

    LOG_PRINTF("aziScanCenter:          %d\r\n", cfg->aziScanCenter);
    LOG_PRINTF("aziScanScope:           %u\r\n", cfg->aziScanScope);

    LOG_PRINTF("eleScanCenter:          %d\r\n", cfg->eleScanCenter);
    LOG_PRINTF("eleScanScope:           %u\r\n", cfg->eleScanScope);

    LOG_PRINTF("coherentDetectSwitch:   %u\r\n", cfg->coherentDetectSwitch);

    return;
}

PRIVATE void ut_cfg_param_proc(protocol_cfg_param_t *cfg)
{
    if (cfg != NULL)
    {
        ut_cfg_param_dump(cfg);
    }

    return;
}

void ut_cfg_param_init(void)
{
    // TODO: disable this UT function when the actual module is ready
    (void)protocol_cfg_param_reg_cb(&ut_cfg_param_proc);

    return;
}
