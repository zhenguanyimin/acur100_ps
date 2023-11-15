
#include <string.h>
#include <math.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"


/**
*  @b Description
*  @n
*		tracking Module calls this function to start target tracking. This function is called during modules' allocation step, 
*		once new set of points passes allocation thresholds 
*
*  @param[in]  handle
*		This is handle to tracking unit
*  @param[in]  timeStamp
*		This is an allocation time stamp
*  @param[in]  tid
*		This is a target identifier given to a unit
*  @param[in]  uCenter
*		This is a pointer to the centroid in measurement coordinates
*
*  \ingroup tracking_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

int tracking_unitStart(void *handle,  sTracking_platformInfo *platformInfo, sMeasurement_SphPoint *measurement,\
	uint16_t* bestIndex, uint32_t wholeSpaceScanCycleCnt)
{
    sTracking_objectUnit *inst;
    sTracking_measurementSphUnion u;
    int n, m;
	float gC[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	int i;
	sTracking_gatingParams gatingParams;

    inst = (sTracking_objectUnit *)handle;
	// init current target
//#ifdef DEMO_ACUR
//	if (measurement->array[0] > 225 && measurement->array[0] < 241) // 此区间内不允许建航
//	{
//		return 1;
//	}
//#endif
	memcpy(u.array, measurement->array, sizeof(measurement->array));
	// init state values
	tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u.array, inst->kalman_state.S_hat);
	/* P_apriori_hat = diag([0,0,0.5,0.5,1,1]) */
	memset(inst->kalman_state.P_hat, 0, sizeof(inst->kalman_state.P_hat));
	for (i = 0; i < SSIZE_XY; i++)
	{
		inst->kalman_state.P_hat[i*(SSIZE_XY + 1)] = pInit[i];
	}
	/* P_apriori_hat = diag([0,0,0.5,0.5,1,1]) */
    memcpy(inst->kalman_state.H_s.array, u.array, sizeof(u.array)); /* Initialize Hs to measurment vector */

	//init association properties
	tracking_calcMeasurementLimits(u.vector.range, &inst->tracking_params->advParams.gatingParams.limits, \
		&inst->assoProperty.H_limits.vector);
	inst->assoProperty.G = inst->tracking_params->advParams.gatingParams.gain;
	memset(gC, 0, sizeof(float)*MSIZE_SPH*MSIZE_SPH);
	for (n = 0; n < MSIZE_SPH; n++) {
		gC[n*MSIZE_SPH + n] = 2.f;
	}
	tracking_matrixInv(gC, &inst->assoProperty.gC_det, inst->assoProperty.gC_inv);
	memset(inst->assoProperty.bitIndicatorsOfDetAsso, 0, sizeof(inst->assoProperty.bitIndicatorsOfDetAsso));
	for (n = 0; n < measurement->detectionNum; n++)
	{
		m = measurement->detectionId[n];
		inst->assoProperty.bitIndicatorsOfDetAsso[m >> 3] |= (1 << (m & 0x7));
	}
	bestIndex[measurement->detId] = inst->uid;
	inst->assoProperty.assoCondenceMeasId = measurement->detId;
	memcpy(&gatingParams, &inst->tracking_params->advParams.gatingParams, sizeof(sTracking_gatingParams));
	getGatingParams(inst->kalman_state.H_s.array, &gatingParams, &inst->tracking_params->advParams, platformInfo);
	inst->assoProperty.assocGating.depth = gatingParams.limits.depth;
	inst->assoProperty.assocGating.width = gatingParams.limits.width;
	inst->assoProperty.assocGating.vel = gatingParams.limits.vel;
	inst->assoProperty.assocGating.height = gatingParams.limits.height;
	inst->assoProperty.assocGating.depth = gatingParams.limits.depth;
	inst->assoProperty.assocGating.width = gatingParams.limits.width;
	inst->assoProperty.assocGating.vel = gatingParams.limits.vel;
	inst->assoProperty.assocGating.height = gatingParams.limits.height;
	inst->assoProperty.assocDynamicGating.depth = inst->assoProperty.assocGating.depth*0.7f;
	inst->assoProperty.assocDynamicGating.width = inst->assoProperty.assocGating.width*0.7f;
	inst->assoProperty.assocDynamicGating.vel = inst->assoProperty.assocGating.vel*0.7f;
	inst->assoProperty.assocDynamicGating.height = inst->assoProperty.assocGating.height*0.7f;
	tracking_calcMeasurementLimits(u.vector.range, &inst->assoProperty.assocDynamicGating, \
		&inst->assoProperty.assocDynamic_H_limits.vector);
	inst->assoProperty.assMahDistance = 0.f;
	inst->assoProperty.lastSeenTime = g_curTimestamp;
	inst->assoProperty.lastSeenWholeScanId = wholeSpaceScanCycleCnt;
	//inst->assoProperty.assoDistribution.uSectorCountMaxIndex = 0U;
	//inst->assoProperty.assoDistribution.uSectorOccupationSum = 0U;
	//memset(inst->assoProperty.assoDistribution.uSectorOccupation, 0, sizeof(uint8_t)*TRACKING_NUM_SECTORS);
	inst->timestamp = g_curTimestamp;
	// Init object management
	inst->objManagement.heartBeatCount = 1;
	inst->objManagement.allocationTime = g_curTimestamp;
	inst->objManagement.allocationRange = measurement->vector.range;
	inst->objManagement.allocationVelocity = measurement->vector.doppler;
	inst->objManagement.estNumOfPoints = 1;
	inst->objManagement.detect2activeCount = 0;
	inst->objManagement.detect2freeCount = 0;
	inst->objManagement.active2freeCount = 0;
	inst->objManagement.lifetime = 0;
	inst->objManagement.updatePast8frame = 0;/* The update status in the past 8 frames,by hxj */
	inst->objManagement.state = TRACK_STATE_DETECTION;
	inst->objManagement.isNewTracker = 1;
	inst->objManagement.forcastFrameNum = 0;
//	inst->objManagement.abnormalTrackerCnter = 0;

	// Init object kinematic parameters
	/* Radial Velocity initialization */
	/* Radial Velocity handling is set to start with range rate filtering */
	inst->objKinematic.velocityHandling = VELOCITY_LOCKED;
	inst->objKinematic.maxDistance = 0.f;
	inst->objKinematic.initPosition.x0 = measurement->x;
	inst->objKinematic.initPosition.y0 = measurement->y;
	inst->objKinematic.initPosition.z0 = measurement->z;

	return 0;
}