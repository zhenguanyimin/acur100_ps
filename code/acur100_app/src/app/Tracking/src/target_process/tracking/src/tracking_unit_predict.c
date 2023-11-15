
#include <string.h>
#include <math.h>
#include <float.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

/**
*  @b Description
*  @n
*		tracking Module calls this function to run tracking unit prediction step 
*
*  @param[in]  handle
*		This is handle to tracking unit
*
*  \ingroup tracking_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void tracking_unitPredict(void *handle, sTracking_platformInfo* platformInfo)
{
    sTracking_objectUnit *inst;
	float temp1[SSIZE_XY*SSIZE_XY] = { 0.f };
	float temp3[SSIZE_XY*SSIZE_XY] = { 0.f };
	float S_apriori[SSIZE_XY] = { 0 };
	int dimLen = 0;
	float Q[SSIZE_XY*SSIZE_XY];
	float F[SSIZE_XY*SSIZE_XY];
    inst = (sTracking_objectUnit *)handle;
	dimLen = inst->tracking_params->stateVectorDimLength;
	inst->timestamp = g_curTimestamp;
	//inst->objManagement.lifetime++;
	memcpy(F, inst->tracking_params->F, sizeof(inst->tracking_params->F));
	memcpy(Q, inst->tracking_params->Q, sizeof(inst->tracking_params->Q));
	//Q[0] = const_process_noise_stat[0] * 50.f;
	//Q[7] = const_process_noise_stat[1] * 30.f;
	//Q[14] = const_process_noise_stat[2] * 150.f;
	//Q[21] = const_process_noise_stat[3] * 100.f;
	//Q[28] = const_process_noise_stat[4] * 1500.f;
	//Q[35] = const_process_noise_stat[5] * 1000.f;
	/* obj.S_apriori_hat(1:mSize) = obj.F(1:mSize,1:mSize) * obj.S_hat(1:mSize) */
	tracking_matrixMultiply(SSIZE_XY, SSIZE_XY, 1, F, inst->kalman_state.S_hat, S_apriori);
	memcpy(inst->kalman_state.S_hat, S_apriori, sizeof(inst->kalman_state.S_hat));
	tracking_matrixMultiply(SSIZE_XY, SSIZE_XY, SSIZE_XY, F, inst->kalman_state.P_hat, temp1);
	tracking_matrixTransposeMultiply(SSIZE_XY, SSIZE_XY, SSIZE_XY, temp1, F, temp3);
//	/* obj.P_apriori(1:mSize,1:mSize) = obj.F(1:mSize,1:mSize) * obj.P(1:mSize,1:mSize) * obj.F(1:mSize,1:mSize)' + obj.Q(1:mSize,1:mSize) */
	tracking_matrixAdd(SSIZE_XY, SSIZE_XY, temp3, Q, temp1);
	tracking_matrixMakeSymmetrical(SSIZE_XY, temp1, inst->kalman_state.P_hat);

	tracking_cartesian2spherical(inst->tracking_params->stateVectorType, \
		inst->kalman_state.S_hat, inst->kalman_state.H_s.array);
	inst->objManagement.isNewTracker = 0;
#ifdef DEBUG_LOG_ZQ
	//my_printf("--predInfo-- id %d, x %.2f, y%.2f,z %.2f,vx %.2f,vy %.2f,vz %.2f,ax %.2f,ay %.2f,az %.2f ,covY %.2f", \
		inst->uid, inst->kalman_state.S_hat[0], inst->kalman_state.S_hat[1], inst->kalman_state.S_hat[2], \
		inst->kalman_state.S_hat[3], inst->kalman_state.S_hat[4], inst->kalman_state.S_hat[5], \
		inst->kalman_state.S_hat[6], inst->kalman_state.S_hat[7], inst->kalman_state.S_hat[8],inst->kalman_state.P_hat[10]);
	
#endif // DEBUG_LOG

	//inst->objGeometryProperty.hasTrackerOfNearRegion = 0U;
	// update Vabs
	//tracking_calculateTrackerAbsVelocity(inst, platformInfo);
}

void tracking_tasPredict(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTracking_objectUnit *tasTracker = NULL;
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		if (uid == inst->tasTargetId)
		{
			tasTracker = (sTracking_objectUnit *)inst->hTrack[uid];
			if (measInfo->condenceDone == 1U)
			{
				tasTracker->objManagement.lifetime++;
			}
			tracking_unitPredict(inst->hTrack[uid], &inst->platformInfo);
			break;
		}
		tElem = tracking_listGetNext(tElem);
	}
}
void tracking_twsPredict(sTracking_moduleInstance *inst)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTracking_objectUnit *twsTracker = NULL;
	tElem = tracking_listGetFirst(&inst->twsTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		twsTracker = (sTracking_objectUnit *)inst->hTrack[uid];
		if (inst->wholeSpaceScanCycleCnt - inst->lastWholeSpaceScanCycleCnt == 1)  // start of current scan
		{
			twsTracker->objManagement.lifetime++;
		}
		tracking_unitPredict(inst->hTrack[uid], &inst->platformInfo);
		tElem = tracking_listGetNext(tElem);
	}
}
