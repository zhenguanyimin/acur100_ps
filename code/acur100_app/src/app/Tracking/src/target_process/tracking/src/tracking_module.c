
#include <math.h>
#include <float.h>
#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

#ifdef MULTI_AZIMU
#define NUM_DOPPLERS (NUM_EXTERN_DOPPLER+1)
#else
#define NUM_DOPPLERS (1)
#endif

/**
*  @b Description
*  @n
*      This is a MODULE level predict function. The function is called by external step function to perform unit level kalman filter predictions
*
*  @param[in]  inst
*      Pointer to tracking module instance
*
*  \ingroup TRACKING_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void tracking_modulePredict(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	tracking_cal_dt_transMatrix(inst);

	if (g_scanType == TAS_SCAN)
	{
		tracking_tasPredict(inst, measInfo);
	}
	else
	{
		tracking_twsPredict(inst);
	}
}





/**
*  @b Description
*  @n
*      This is a MODULE level associatiation function. The function is called by external step function to associate measurement points with known targets
*
*  @param[in]  inst
*      Pointer to TRACKING module instance
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  num
*      Number of input measurements
*
*  \ingroup TRACKING_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/
void tracking_moduleAssociate(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	if (g_scanType == TAS_SCAN)
	{
		tracking_tasAssociate(inst, measInfo);
	}
	else
	{
		tracking_twsAssociate(inst, measInfo);
	}
}

/**
*  @b Description
*  @n
*      This is a MODULE level update function. The function is called by external step function to perform unit level kalman filter updates
*
*  @param[in]  inst
*      Pointer to tracking module instance
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  var
*      Pointer to an array of input measurment variances. Set to NULL if variances are unknown
*  @param[in]  num
*      Number of input measurements
*
*  \ingroup TRACKING_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/

void tracking_moduleUpdate(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	if (g_scanType == TAS_SCAN)
	{
		tracking_tasUpdate(inst, measInfo);
	}
	else
	{
		tracking_twsUpdate(inst, measInfo);
	}
}
void tracking_targetMerge(sTracking_moduleInstance *inst, sTracking_ListObj *activeList)
{
	int i, j, k;
	float gateX, gateY, gateZ, gateVx, gateVy, gateVz;
	float Xi, Xj, Yi, Yj, Zi, Zj, Vxi, Vxj, Vyi, Vyj, Vzi, Vzj,iRange,jRange,iDopper,jDopper,\
		iAzimuth,jAzimuth,iPitch,jPitch;
	sTracking_ListElem *iElem, *jElem;
	sTracking_objectUnit *iUnit, *jUnit;
	uint16_t iData, jData;
	sTracking_ListElem *tElem = NULL;
	sTracking_ListElem *tElemToRemove = NULL;
	sTracking_enum_state state = TRACK_STATE_INIT;
	uint16_t uid = 0;
	int iAlive, jAlive;
	gateX = 10.0f;
	gateY = 10.0f;
	gateZ = 10.f;
	gateVx = 3.0f;
	gateVy = 3.0f;
	gateVz = 3.0f;
	for (iElem = tracking_listGetFirst(activeList), i = 0; iElem != 0; iElem = tracking_listGetNext(iElem), i++)
	{
		iData = iElem->data;
		iUnit = (sTracking_objectUnit*)inst->hTrack[iData];
		Xi = iUnit->kalman_state.S_hat[0];
		Yi = iUnit->kalman_state.S_hat[1];
		Zi = iUnit->kalman_state.S_hat[2];
		Vxi = iUnit->kalman_state.S_hat[3];
		Vyi = iUnit->kalman_state.S_hat[4];
		Vzi = iUnit->kalman_state.S_hat[5]; 
		iRange = iUnit->kalman_state.H_s.vector.range;
		iDopper = iUnit->kalman_state.H_s.vector.doppler;
		iAzimuth = iUnit->kalman_state.H_s.vector.azimuthRad;
		iPitch = iUnit->kalman_state.H_s.vector.pitchRad;
		iAlive = iUnit->objManagement.lifetime;
		/*if (iUnit->objManagement.state == TRACK_STATE_DETECTION)
			continue;*/
		if (iAlive <= 3) continue;
			
		for (jElem = tracking_listGetFirst(activeList), j = 0; jElem != 0; jElem = tracking_listGetNext(jElem), j++)
		{
			jData = jElem->data;
			jUnit = (sTracking_objectUnit*)inst->hTrack[jData];
			if (j <= i)	continue;				
			/*if (jUnit->objManagement.state == TRACK_STATE_DETECTION)
				continue;*/
			jAlive = jUnit->objManagement.lifetime;
			if (jAlive <= 3) continue;
			Xj = jUnit->kalman_state.S_hat[0];
			Yj = jUnit->kalman_state.S_hat[1];
			Zj = jUnit->kalman_state.S_hat[2];
			Vxj = jUnit->kalman_state.S_hat[3];
			Vyj = jUnit->kalman_state.S_hat[4];
			Vzj = jUnit->kalman_state.S_hat[5];
			jRange = jUnit->kalman_state.H_s.vector.range;
			jDopper = jUnit->kalman_state.H_s.vector.doppler;
			jAzimuth = jUnit->kalman_state.H_s.vector.azimuthRad;
			jPitch = jUnit->kalman_state.H_s.vector.pitchRad;
			
			if (((fabsf(Xi - Xj) < gateX) && (fabsf(Yi - Yj) < gateY) && (fabsf(Zi - Zj) < gateZ)\
				&& (fabsf(Vxi - Vxj) < gateVx) && (fabsf(Vyi - Vyj) < gateVy) && (fabsf(Vzi - Vzj) < gateVz)) || \
				(iUnit->assoProperty.assoCondenceMeasId == jUnit->assoProperty.assoCondenceMeasId&&\
				(fabsf(iRange - jRange) < gateX) && (fabsf(iDopper - jDopper) < gateVx / 2.f) &&\
					(fabsf(iAzimuth - jAzimuth) < 4.f*DEG2RAD) && (fabsf(iPitch - jPitch) < 3.f*DEG2RAD))) 
			{
				if (iAlive >= jAlive)
				{
					jUnit->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
					my_printf("debug %d id %d", 6, jUnit->uid);
#endif // DEBUG_LOG
				}
				else
				{
					iUnit->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
					my_printf("debug %d id %d", 7, iUnit->uid);
#endif // DEBUG_LOG
				}
			}
		}
	}
}

void tracking_moduleMerge(sTracking_moduleInstance *inst)
{
	if (g_scanType == TAS_SCAN)
	{
		tracking_targetMerge(inst, &inst->tasTrackList);
	}
	else
	{
		tracking_targetMerge(inst, &inst->twsTrackList);
	}
}

/**
*  @b Description
*  @n
*      This is a MODULE level tracking manager function. The function is called by external step function to perform unit level of deleting tracker
*
*  @param[in]  inst
*      Pointer to tracking module instance

*  \ingroup TRACKING_ALG_MODULE_FUNCTION
*
*  @retval
*      None
*/
void tracking_moduleManager(sTracking_moduleInstance *inst)
{
	if (g_scanType == TAS_SCAN)
	{
		tracking_tasManager(inst);
	}
	else
	{
		tracking_twsManager(inst);
	}
	tracking_TasTwsTypeManagement(inst);
}

void tracking_moduleAllocate(sTracking_moduleInstance *inst,sMeasOutput *measInfo)
{
	sTracking_ListElem *tElemFree = NULL;
	sTracking_ListElem *tElemActive = NULL;
	sTracking_ListElem *tElemMinScore = NULL;
	sMeasurement_SphPoint* measurement;
	int unitStart_ret = -1;
	sTracking_allocationParams allocThres;
	sTracking_objectUnit *pTracker;
	sTrackingParams *tracking_params = inst->tracking_params;
	int n;
	int i;
	uint16_t uid;
	float maxRange = 0.1f;
	float minMag_thre_array[MAX_NUM_DETECTS] = { 0.f };
	float minSNR_thre_array[MAX_NUM_DETECTS] = { 0.f };
	float maxVelThre_array[MAX_NUM_DETECTS] = { 0.f };
	// 1.get new tracker allocate params
	memcpy(&allocThres, &inst->tracking_params->advParams.allocationParams, sizeof(sTracking_allocationParams));
	for (n = 0; n < measInfo->num; n++)
	{
		measurement = &measInfo->measurement[n];
		if (inst->bestIndex[n] == TRACKING_ID_POINT_NOT_ASSOCIATED) {
			getAllocationParams(measurement->array, &allocThres, \
				&inst->tracking_params->advParams, 1, &inst->platformInfo);
			maxVelThre_array[n] = allocThres.maxVelThre;
			getMagMinParams(measurement->array, &minMag_thre_array[n]);
			getSNRMinParams(measurement->array, &minSNR_thre_array[n]);
		}
	}
	for (i = 0; i < measInfo->num; i++)
	{
		measurement = &measInfo->measurement[i];
		if (inst->bestIndex[i] == TRACKING_ID_POINT_NOT_ASSOCIATED)
		{
			// 2. check whether trackList is full or not,if so, delete the worest one
			tElemFree = tracking_listGetFirst(&inst->freeList);
			if (tElemFree == 0)
			{
				// TODO: 
				maxRange = 0.1f;
				tElemActive = tracking_listGetFirst(&inst->twsTrackList);
				while (tElemActive != 0)
				{
					uid = tElemActive->data;
					pTracker = (sTracking_objectUnit *)inst->hTrack[uid];
					if (pTracker->kalman_state.H_s.vector.range > maxRange && pTracker->objManagement.state != TRACK_STATE_ACTIVE) {
						maxRange = pTracker->kalman_state.H_s.vector.range;
						tElemMinScore = tElemActive;
					}
					tElemActive = tracking_listGetNext(tElemActive);
				}

				if (measurement->vector.range < maxRange)
				{
#ifdef DEBUG_LOG_ZQ
					my_printf("Delete, id[%d]\n", (int)tElemMinScore->data);
#endif
					tracking_listRemoveElement(&inst->twsTrackList, tElemMinScore);
					inst->targetNumCurrent--;
					tracking_listEnqueue(&inst->freeList, tElemMinScore);
					tElemFree = tracking_listGetFirst(&inst->freeList);
				}
			}
			// 3.allocate new tracker
			tElemFree = tracking_listDequeue(&inst->freeList);
			if (tElemFree != 0) {
				unitStart_ret = tracking_unitStart(inst->hTrack[tElemFree->data], &inst->platformInfo, \
					measurement,inst->bestIndex, inst->wholeSpaceScanCycleCnt);
				if (unitStart_ret == 0)
				{
					pTracker = (sTracking_objectUnit *)inst->hTrack[tElemFree->data];
#ifdef DEBUG_LOG_ZQ
					my_printf("new tracker id %d, x %.2f,y %.2f,z %.2f, vx %.2f, vy %.2f,vz %.2f,", pTracker->uid,\
						pTracker->kalman_state.S_hat[0], pTracker->kalman_state.S_hat[1], pTracker->kalman_state.S_hat[2], \
						pTracker->kalman_state.S_hat[3], pTracker->kalman_state.S_hat[4], pTracker->kalman_state.S_hat[5]);
#endif // DEBUG_LOG

					// if tws workmode was setted , new trackers were created in twsTrackList, else created in tasTrackList
					if (tracking_params->workMode == 0)
					{
						tracking_listEnqueue(&inst->twsTrackList, tElemFree);
					}
					else
					{
						tracking_listEnqueue(&inst->twsTrackList, tElemFree);
					}
				}
				else
				{
					tracking_listEnqueue(&inst->freeList, tElemFree);
				}
			}
		}
	}
}

//uint8_t tracking_unit_cluster(sTracking_moduleInstance* inst, \
//	sMeasurement_SphPoint* point, \
//	uint16_t num, uint16_t n, sTracking_measurementSphUnion* mCenter, \
//	sTracking_measurementSphUnion* mCurrent, \
//	uint8_t* vIsClustered, uint16_t* vIndex, int* clu_num, uint8_t n_ex, \
//	float* minMag_thre_array, float* minSNR_thre_array, float* maxVelThre_array)
//{
//	int k = 0;
//	int ex_k = 0;
//	int m = 0;
//	uint8_t isWithinLimits = true;
//	sTracking_measurementSphUnion H_limits;
//	sTracking_measurementSphUnion u_tilda;
//	uint8_t getNeighbor = false;
//
//	tracking_calcMeasurementLimits(mCenter->vector.range, &inst->tracking_params->advParams.allocationParams.limits, &H_limits.vector);
//	if (point[n].mag < minMag_thre_array[n]) {
//		return 0;
//	}
//	if (point[n].snr < minSNR_thre_array[n]) {
//		return 0;
//	}
//	for(k=n+1; k<num; k++) {
//		isWithinLimits = true;
//		if(inst->bestIndex[k] == TRACKING_ID_POINT_NOT_ASSOCIATED) {
//			if (vIsClustered[k*NUM_DOPPLERS] != 0 && vIsClustered[n*NUM_DOPPLERS+n_ex] != 0) {
//				continue;
//			}
//			if (point[k].mag < minMag_thre_array[k]) {
//				continue;
//			}
//			if (point[k].snr < minSNR_thre_array[k]) {
//				continue;
//			}
//			memcpy(&(mCurrent->vector), &(point[k].vector), sizeof(mCurrent->vector));
//			if(fabsf(mCurrent->vector.doppler - mCenter->vector.doppler) < maxVelThre_array[n]) {
//				tracking_vectorSub(MEASUREMENT_VECTOR_SIZE, mCurrent->array, mCenter->array, u_tilda.array); 
//				for (m = 0; m < MSIZE_SPH-1; m++)
//				{
//					if (fabsf(u_tilda.array[m]) > H_limits.array[m])
//					{
//						isWithinLimits = false;
//						break;
//					}
//				}
//				if(point[n].flag_moving != point[k].flag_moving)
//				{
//					if (fabsf(u_tilda.array[MSIZE_SPH-1]) > TRACKING_MIN_STATIC_VELOCITY_RAD)
//					{
//						isWithinLimits = false;
//					}
//				}
//				else
//				{
//					if (fabsf(u_tilda.array[MSIZE_SPH-1]) > H_limits.array[MSIZE_SPH-1])
//					{
//						isWithinLimits = false;
//					}
//				}
//				
//				if(isWithinLimits) {
//					if (vIsClustered[k*NUM_DOPPLERS] == 0 && vIsClustered[n*NUM_DOPPLERS+n_ex]==0)
//					{
//						vIndex[k*NUM_DOPPLERS] = (uint16_t)(*clu_num);
//						vIsClustered[k*NUM_DOPPLERS] = 1;
//						vIndex[n*NUM_DOPPLERS+n_ex] = (uint16_t)(*clu_num);
//						vIsClustered[n*NUM_DOPPLERS+n_ex] = 1;
//						(*clu_num)++;
//					}
//					else if(vIsClustered[k*NUM_DOPPLERS]==0 && vIsClustered[n*NUM_DOPPLERS*n_ex]!=0)
//					{
//						vIndex[k*NUM_DOPPLERS] = vIndex[n*NUM_DOPPLERS+n_ex];
//						vIsClustered[k*NUM_DOPPLERS] = 1;
//					}
//					else if (vIsClustered[k*NUM_DOPPLERS] != 0 && vIsClustered[n*NUM_DOPPLERS+n_ex] == 0) {
//						vIndex[n*NUM_DOPPLERS+n_ex] = vIndex[k*NUM_DOPPLERS];
//						vIsClustered[n*NUM_DOPPLERS+n_ex] = 1;
//					}
//					getNeighbor = true;
//				}
//			}
//
//		}
//	}
//	return getNeighbor;
//}



