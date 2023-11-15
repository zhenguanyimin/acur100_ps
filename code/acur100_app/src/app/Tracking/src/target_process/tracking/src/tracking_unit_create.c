
#include <string.h>
#include <math.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

/**
*  @b Description
*  @n
*		TRACKING Module calls this function to instantiate TRACKING Unit with desired configuration parameters. 
*		Function returns a handle, which is used my module to call units' methods
*
*  @param[in]  params
*		This is a pointer to the configuration structure. The structure contains all parameters that are exposed by TRACKING alrorithm.
*		The configuration does not need to persist.
*  @param[out] errCode
*      Error code populated on error, see \ref TRACKING_ERROR_CODE
*  \ingroup TRACKING_ALG_UNIT_FUNCTION
*
*  @retval
*      Handle to TRACKING unit
*/

void *tracking_unitCreate(sTrackingParams *params, int32_t *errCode, sTracking_objectUnit *unitInst)
{
    sTracking_objectUnit *inst;
		
	*errCode = TRACKING_EOK;
	inst = &unitInst[params->uid];
	if(inst == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	inst->tracking_params = params;

	inst->tid = 0;
	inst->uid = params->uid;
	
	// init kalman state
	memset(inst->kalman_state.S_hat, 0, sizeof(inst->kalman_state.S_hat));
	tracking_matrixTriangleEye(TRACKING_STATE_VECTOR_SIZE, inst->kalman_state.P_hat);
	memset(&inst->kalman_state.H_s, 0, sizeof(inst->kalman_state.H_s));
	memcpy(inst->kalman_state.Rc, const_tws_measurement_noise, sizeof(inst->kalman_state.Rc));
	inst->kalman_state.fVabsX = 0.f;
	inst->kalman_state.fVabsY = 0.f;

	// init current target
	memset(&inst->curTarget.unionX, 0, sizeof(inst->curTarget.unionX));
	memset(&inst->curTarget.unionXVar, 0, sizeof(inst->curTarget.unionXVar));
	memset(&inst->curTarget.unionY, 0, sizeof(inst->curTarget.unionY));
	memset(&inst->curTarget.unionYVar, 0, sizeof(inst->curTarget.unionYVar));
	memset(&inst->curTarget.unionSph, 0, sizeof(inst->curTarget.unionSph));
	memset(&inst->curTarget.unionSphVar, 0, sizeof(inst->curTarget.unionSphVar));
	inst->curTarget.fPDH0 = 0.f;
	inst->curTarget.fRCS = 0.f;
	inst->curTarget.fMag = 0.f;
	inst->curTarget.fSNR = 0.f;
	inst->curTarget.fIdealStatVrad = 0.f;
	inst->curTarget.uNumOfDetections = 0U;
	memset(&inst->curTarget.estSpread, 0, sizeof(inst->curTarget.estSpread));
	memcpy(inst->curTarget.Rm, const_tws_measurement_noise, sizeof(inst->curTarget.Rm));
	memcpy(inst->curTarget.gD, const_tws_measurement_noise, sizeof(inst->curTarget.gD));
	inst->curTarget.uFlagNearOrFarScan = 0U;
	inst->curTarget.uFlagFastOrSlowScan = 0U;
	inst->curTarget.uFlagMoving = 0U;
	inst->curTarget.mdScore = 100.f;


	//init dynamic property
	inst->eDynamicProperty = TRACKING_DYNAMIC_UNKNOWN;

	//init association properties
	memset(&inst->assoProperty.H_limits, 0, sizeof(inst->assoProperty.H_limits));
	memset(inst->assoProperty.gC_inv, 0, sizeof(inst->assoProperty.gC_inv));
	inst->assoProperty.assMahDistance = 0.f;
	inst->assoProperty.gC_det = 0.f;
	inst->assoProperty.G = 0.f;
	inst->assoProperty.lastSeenTime = g_curTimestamp;
	inst->assoProperty.lastSeenWholeScanId = 0;
	//inst->assoProperty.ellipseVolume = 0.f;
	memset((uint8_t *)inst->assoProperty.bitIndicatorsOfDetAsso, 0, sizeof(inst->assoProperty.bitIndicatorsOfDetAsso));
	//inst->assoProperty.assoDistribution.uSectorCountMaxIndex = 0U;
	//inst->assoProperty.assoDistribution.uSectorOccupationSum = 0U;
	//memset(inst->assoProperty.assoDistribution.uSectorOccupation, 0, sizeof(inst->assoProperty.assoDistribution.uSectorOccupation));

	// Init object management
	inst->objManagement.heartBeatCount = 0;
	inst->objManagement.allocationTime = 0;
	inst->objManagement.allocationRange = 0;
	inst->objManagement.allocationVelocity = 0;
	inst->objManagement.estNumOfPoints = 0;
	inst->objManagement.detect2activeCount = 0;
	inst->objManagement.detect2freeCount = 0;
	inst->objManagement.active2freeCount = 0;
	inst->objManagement.lifetime = 0;
	inst->objManagement.state = TRACK_STATE_INIT;
	inst->objManagement.isNewTracker = 0;
	inst->objManagement.forcastFrameNum = 0;
//	inst->objManagement.abnormalTrackerCnter = 0;

	// Init object kinematic parameters
	inst->objKinematic.velocityHandling = VELOCITY_INIT;
	inst->objKinematic.rangeRate = 0;
	inst->objKinematic.maxDistance = 0.f;
	inst->objKinematic.initPosition.x0 = 0.f;
	inst->objKinematic.initPosition.y0 = 0.f;
	inst->objKinematic.initPosition.z0 = 0.f;


	exit:
	if(*errCode != TRACKING_EOK) {
		inst = NULL;
	}
    return (void *)inst;
}

