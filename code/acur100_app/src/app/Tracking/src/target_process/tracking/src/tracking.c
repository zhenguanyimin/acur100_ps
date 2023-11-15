
#include "../include/tracking.h"
#include"../../dispatch/include/dispatch.h"
#include <stdlib.h>
#include <time.h>

#ifdef ENABLE_ADTF_LOG
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#endif

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

float *const_process_noise = NULL;
float *const_max_acceleration = NULL;
float *const_tws_measurement_noise = NULL;
float *const_tas_measurement_noise = NULL;
sTracking_sceneryParams *const_roiArea = NULL;
sTracking_insideScratchPad *gScratchPadData = NULL;
sTrackingInst* gTracking_inst = NULL;

float g_average_dt = 0.075f;
int g_frame_dtCnt = 0;
float g_curTimestamp = 0.f;

//clock_t tracking_startTime;
//clock_t tracking_endTime;
#ifdef AEB_PORTING
extern uint32_t OSIF_GetMilliseconds(void);
extern uint32_t tracking_startTime, tracking_endTime, tracking_deltaTime;
#endif

//sTracking_objectUnit* gUnitTrack_debug[MAX_NUM_TRAJS] = { 0 };

int tracking_init(sAlgObjData *algObj) 
{
	gTracking_inst = (sTrackingInst*)(algObj->tracking_inst);
	if (gTracking_inst == NULL) {
		return -1;
	}
	const_process_noise = gTracking_inst->init_process_noise;
	const_max_acceleration = gTracking_inst->init_max_acceleration;
	const_tws_measurement_noise = gTracking_inst->init_tws_measurement_noise;
	const_tas_measurement_noise = gTracking_inst->init_tas_measurement_noise;
	const_roiArea = &gTracking_inst->init_roiArea;
	gScratchPadData = gTracking_inst->scratchPadData;
	memcpy(&(gTracking_inst->trackingConfig.advParams.sceneryParams), const_roiArea, sizeof(sTracking_sceneryParams));
	gTracking_inst->handle = tracking_create(&gTracking_inst->trackingConfig, \
		&gTracking_inst->errorCode, gScratchPadData);
	if (gTracking_inst->handle == NULL) {
		return -1;
	}
	return 0;
}

// Run it to cluster points from one frame data
void tracking_process(sAlgObjData *algObj)
{
	sTrackingInst* tracking_inst = (sTrackingInst*)(algObj->tracking_inst);
	sMeasProcessInst* measurementProcess_inst = (sMeasProcessInst*)algObj->measurementProcess_inst;
	sMeasOutput *measInfo = measurementProcess_inst->measOutput;
	// TODO: condenceDone???
	/*if (measInfo->condenceDone == 0)
		return;*/
	tracking_preProcess(algObj);
    // Runs a single step of the given algorithm instance with input point cloud data
	tracking_step(tracking_inst->handle, measInfo);

	tracking_report(algObj, tracking_inst->handle);
}

void tracking_preProcess(sAlgObjData* algObj)
{
	int j = 0;
	int i = 0;
	sTracking_moduleInstance *inst = NULL;
	sTrackingTarget* tracker = NULL;
    sTracking_objectUnit *inst_tracker = NULL;
	sMeasurement_SphPoint *measurement = NULL;
	sTrackingInst* tracking_inst = (sTrackingInst*)(algObj->tracking_inst);
	sMeasProcessInst* measurementProcess_inst = (sMeasProcessInst*)algObj->measurementProcess_inst;
	sDispatchInst* dispatch_inst = (sDispatchInst*)algObj->dispatch_inst;
	uint32_t wholeSpaceScanCycleCnt = dispatch_inst->dispatchOutput->wholeSpaceScanCycleCnt;
	sTracking_boundaryBox* boundaryBox = tracking_inst->init_roiArea.boundaryBox;
	sTracking_scanBoundary *scanBoundary = &tracking_inst->trackingConfig.advParams.sceneryParams.scanBoundary;
	inst = (sTracking_moduleInstance*)tracking_inst->handle;
	inst->tasTargetId = dispatch_inst->dispatchOutput->tasObjId;
	inst->wholeSpaceScanCycleCnt = wholeSpaceScanCycleCnt;
	tracking_inst->curTimestamp = (float)(algObj->detectsListOutput->timestamp) / 1000.f;
#ifdef CONST_DELTA_T
	tracking_inst->dt = CONST_DELTA_T;
#endif
	g_curTimestamp = tracking_inst->curTimestamp;
#ifdef DEBUG_LOG_ZQ
	if (measurementProcess_inst->measOutput->num > 0)
	{
		my_printf("Tracking curTime: %.6f", g_curTimestamp);
	}
	
#endif

	tracking_inst->runningStatus = MODULE_OK;
	tracking_inst->frame_cnt++;
	for (i = 0; i < measurementProcess_inst->measOutput->num; i++)
	{
		measurement = &measurementProcess_inst->measOutput->measurement[i];
#ifdef DEBUG_LOG_ZQ
		my_printf("detId %d r a p v [%.2f %.2f %.2f %.2f] x y z[%.2f %.2f %.2f]", i, \
			measurement->vector.range, \
			measurement->vector.azimuthRad*RAD2DEG, \
			measurement->vector.pitchRad*RAD2DEG, \
			measurement->vector.doppler, \
			measurement->x, measurement->y, measurement->z);
#endif
		if (fabsf(measurement->vector.azimuthRad) > scanBoundary->aziScanScope*DEG2RAD || \
			fabsf(measurement->vector.pitchRad) > scanBoundary->eleScanScope*DEG2RAD || \
			measurement->vector.range > 1100.f)
		{
			inst->bestIndex[i] = TRACKING_ID_POINT_BEHIND_THE_WALL;
#ifdef DEBUG_LOG_ZQ
			my_printf("det %d is behind the wall", i);
#endif
		}
		else
		{
			inst->bestIndex[i] = TRACKING_ID_POINT_NOT_ASSOCIATED;
		}
		if (tracking_inst->init_roiArea.numBoundaryBoxes)
		{
			if (measurement->x > boundaryBox[0].x1&&measurement->x < boundaryBox[0].x2&&\
				measurement->y> boundaryBox[0].y1&&measurement->y < boundaryBox[0].y2&&\
				measurement->z> boundaryBox[0].z1&&measurement->z < boundaryBox[0].z2)
			{
				inst->bestIndex[i] = TRACKING_ID_POINT_NOT_ASSOCIATED;
			}
			else
			{
				inst->bestIndex[i] = TRACKING_ID_POINT_BEHIND_THE_WALL;
			}
		}
	}
}

// return tracking result
void tracking_report(sAlgObjData *algObj, void *handle)
{
	sTracking_moduleInstance *inst = NULL;
	sTracking_ListElem *tElem;
	uint16_t uid;
	uint16_t num = 0;
	uint16_t tasTrackNum = 0;
	uint16_t twsTrackNum = 0;
	int ret = -1;
	sTrackingInst* tracking_inst = (sTrackingInst*)(algObj->tracking_inst);
	inst = (sTracking_moduleInstance *)handle;
	tracking_inst->trajInfoOutput->timestamp = (uint32_t)(tracking_inst->curTimestamp*1000.f);
	tracking_inst->trajInfoOutput->frameID = algObj->detectsListOutput->frameID;
	inst->lastWholeSpaceScanCycleCnt = inst->wholeSpaceScanCycleCnt;
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		ret = tracking_unitReport(inst->hTrack[uid], algObj, num);
		if (ret == 0)
		{
			tracking_inst->trajInfoOutput->trajList[num].twsTasFlag = 1;
			tasTrackNum++;
			num++;
		}
		tElem = tracking_listGetNext(tElem);
	}

	tElem = tracking_listGetFirst(&inst->twsTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		ret = tracking_unitReport(inst->hTrack[uid], algObj, num);
		if (ret == 0)
		{
			tracking_inst->trajInfoOutput->trajList[num].twsTasFlag = 0;
			twsTrackNum++;
			num++;
		}
		tElem = tracking_listGetNext(tElem);
	}
	tracking_inst->trajInfoOutput->trackObjNum = num;
	tracking_inst->trajInfoOutput->trackTasNum = tasTrackNum;
	tracking_inst->trajInfoOutput->trackTwsNum = twsTrackNum;
}

int tracking_unitReport(void *handle, sAlgObjData *algObj, uint16_t num)
{
    sTracking_objectUnit *inst;
	sTrackingTarget* tracker = NULL;
	sTrackingInst* tracking_inst; 
	sTracking_moduleInstance *inst_module = NULL;
	sPlatformInfo* platformInfo = algObj->platformInfo;
	float pitching = 0.f;
	float x, z, vx, vz, ax, az;
	float stateBefRotX, stateBefRotZ;
	int i = 0;
	float cosPitch = cosf(pitching);
	float sinPitch = sinf(pitching);
	uint8_t mNum = algObj->detectsListOutput->detectObjNum;
	sDetectPoint* detPointsList = algObj->detectsListOutput->detPointsList;
	tracking_inst = (sTrackingInst*)(algObj->tracking_inst);
	inst_module = (sTracking_moduleInstance *)tracking_inst->handle;
	tracker = &(tracking_inst->trajInfoOutput->trajList[num]);
	inst = (sTracking_objectUnit *)handle;


#ifdef STATE_DETECTION_OUTPUT
	if(1)
#else
	if(inst->objManagement.state==TRACK_STATE_ACTIVE || inst->objManagement.state==TRACK_STATE_NEW_ACTIVE)
#endif
	{
#ifdef TRANSFORM_COORDINATE
		pitching = platformInfo->platformData.pitching*PLATFORM_FORMAT_SCALE*DEG2RAD;  // TODO pitching unit?? int32_t
		cosPitch = cosf(pitching);
		sinPitch = sinf(pitching);
#endif // TRANSFORM_COORDINATE
		stateBefRotX = inst->kalman_state.S_hat[0];
		stateBefRotZ = inst->kalman_state.S_hat[2];
		x = stateBefRotX * cosPitch - stateBefRotZ * sinPitch;
		z = stateBefRotX * sinPitch + stateBefRotZ * cosPitch;
		stateBefRotX = inst->kalman_state.S_hat[3];
		stateBefRotZ = inst->kalman_state.S_hat[5];
		vx = stateBefRotX * cosPitch - stateBefRotZ * sinPitch;
		vz = stateBefRotX * sinPitch + stateBefRotZ * cosPitch;
		stateBefRotX = inst->kalman_state.S_hat[6];
		stateBefRotZ = inst->kalman_state.S_hat[8];
		ax = stateBefRotX * cosPitch - stateBefRotZ * sinPitch;
		az = stateBefRotX * sinPitch + stateBefRotZ * cosPitch;

		tracker->id = (uint16_t)(inst->uid);

		tracker->traj.x = (int32_t)(x * ONE6FORMAT_SCALE);
		tracker->traj.y = (int32_t)(inst->kalman_state.S_hat[1] * ONE6FORMAT_SCALE);
		tracker->traj.z = (int32_t)(z * ONE6FORMAT_SCALE);
		tracker->traj.vx = (int16_t)(vx * ONE6FORMAT_SCALE);
		tracker->traj.vy = (int16_t)(inst->kalman_state.S_hat[4] * ONE6FORMAT_SCALE);
		tracker->traj.vz = (int16_t)(vz * ONE6FORMAT_SCALE);
		tracker->traj.ax = (int16_t)(ax * ONE6FORMAT_SCALE);
		tracker->traj.ay = (int16_t)(inst->kalman_state.S_hat[7] * ONE6FORMAT_SCALE);
		tracker->traj.az = (int16_t)(az * ONE6FORMAT_SCALE);
		for (i = 0; i < SSIZE_XY; i++)
		{
			tracker->variance[i] = (uint16_t)(inst->kalman_state.P_hat[i*(SSIZE_XY + 1)] * ONE6FORMAT_SCALE);
		}
		tracker->variance[8] = (uint16_t)(inst->assoProperty.assMahDistance * ONE6FORMAT_SCALE);
		tracker->range = (uint32_t)(inst->kalman_state.H_s.vector.range*ONE6FORMAT_SCALE);
		tracker->azimuth = (int16_t)(inst->kalman_state.H_s.vector.azimuthRad*RAD2DEG*ONE6FORMAT_SCALE );			
		tracker->elevation = (int16_t)((inst->kalman_state.H_s.vector.pitchRad + pitching)*RAD2DEG*ONE6FORMAT_SCALE);
		tracker->velocity = (int16_t)(inst->kalman_state.H_s.vector.doppler*ONE6FORMAT_SCALE);
		//tracker->mag = (uint16_t)(0.0f*ONE6FORMAT_SCALE);
		tracker->mag = (uint16_t)(inst->objKinematic.maxDistance*ONE6FORMAT_SCALE);
		tracker->ambiguous = 1;
		tracker->alive = inst->objManagement.lifetime;
		switch (inst->objManagement.state)
		{
#ifdef STATE_DETECTION_OUTPUT
			case TRACK_STATE_DETECTION:
			{
				tracker->stateType = UNSTABLE_TRAJ;
				break;
			}
#endif
			case TRACK_STATE_NEW_ACTIVE:
			{
			}
			case TRACK_STATE_ACTIVE:
			{
				tracker->stateType = STABLE_TRAJ;
				break;
			}
			default:
			{
				tracker->stateType = FREE_TRAJ;
				break;
			}
		}
		tracker->assocBit0 = 0;
		tracker->assocBit1 = 0;
		if (inst->assoProperty.assoCondenceMeasId >= 0)
		{
			tracker->associationNum = inst->assoProperty.assoCondenceMeasId;
			for (i = 0; i < mNum; i++) {
				if (inst->assoProperty.bitIndicatorsOfDetAsso[i >> 3] & (0x1 << (i & 0x7))) {
					if (detPointsList[i].id < 32) {
						tracker->assocBit0 |= (0x1 << detPointsList[i].id);
					}
					else if (detPointsList[i].id < 64) {
						tracker->assocBit1 |= (0x1 << (detPointsList[i].id - 32));
					}
#ifdef DEBUG_LOG_ZQ
					//my_printf("id %d bit0 bit1 %d %d", tracker->id, tracker->assocBit0, tracker->assocBit1);
#endif // DEBUG_LOG_ZQ

				}
			}

		}
		else
		{
			tracker->associationNum = 255;
			tracker->assocBit0 = 0;
			tracker->assocBit1 = 0;
		}
		tracker->forcastFrameNum = (uint16_t)(inst->objManagement.forcastFrameNum);
		return 0;
	}
	return -1;
}

// check input data
uint8_t tracking_dataCheck(int input_n)
{
	if (input_n == 0) {
		return false;
	}
	return true;
}

// reset this module
void tracking_reset()
{
}

// check the running status
MODULE_STATUS tracking_checkRunningStatus()
{
	return gTracking_inst->runningStatus;
}
