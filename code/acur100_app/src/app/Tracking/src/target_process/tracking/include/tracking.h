
#ifndef TRACKING_H
#define TRACKING_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include "../../../utilities/common_struct.h"
#include "tracking_common.h"
/*********** Right Hand Coordinate System (AUTOSAR) *******
*
*                  ^ x
*                  |
*                  |
*      y           |
*      <------------------------
*                  |
*  Angle/Azimuth: left side is position(+)
*
*************************************************/

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif

typedef struct sTrackingInst {
	sTrackingParams trackingConfig;
	int32_t errorCode;
	void *handle;
	float dt;
	float curTimestamp;		// s
	int frame_cnt;
	MODULE_STATUS runningStatus;
	int enable_confCoef;
	float init_process_noise[SSIZE_XY]; // size: 9
	float init_max_acceleration[3]; // size: 3
	float init_tws_measurement_noise[MSIZE_SPH]; // size: 3 or 4
	float init_tas_measurement_noise[MSIZE_SPH]; // size: 3 or 4
	//measUnitData2D init_roiArea; // Set the roi area in measurement coordination
	sTracking_sceneryParams init_roiArea;
	float egoVel; // the ego-vehicle velocity
	float egoOmega; // The ego-vehicle omega
	float egoAcc; // The ego-vehicle acceleration
	sTracking_insideScratchPad* scratchPadData;
	sTrackingFrame* trajInfoOutput; // the output of tracking module
} sTrackingInst;

int tracking_init(sAlgObjData *algObj);
void tracking_process(sAlgObjData *algObj);
void tracking_preProcess(sAlgObjData* algObj);
void tracking_report(sAlgObjData *algObj, void *handle);
int tracking_unitReport(void *handle, sAlgObjData *algObj, uint16_t num);
void trackingConfigParamLoad(sTrackingParams* trackingConfigParam, sPlatformInfo* platformInfo, \
	sDebugAndCtrlParam* ctrlInfo);

void tracking_step(void *handle, sMeasOutput *point);
// check input data
uint8_t tracking_dataCheck(int input_n);
// reset module
void tracking_reset();
// check running status
MODULE_STATUS tracking_checkRunningStatus();

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif

#endif /* TRACKING_H */
