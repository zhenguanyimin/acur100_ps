
#include <math.h>
#include <string.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

void tracking_tasAssociate(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTracking_objectUnit *tasTracker = NULL;
	sTrackingParams *tracking_params = inst->tracking_params;
	int i = 0;
	float mdGating = tracking_params->advParams.tasMdGatingParams.gating;
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		if (uid == inst->tasTargetId)
		{
			tracking_unitAssociate(inst->hTrack[uid], &inst->platformInfo, measInfo, \
				mdGating, &inst->bestIndex[0], inst->wholeSpaceScanCycleCnt, inst->lastWholeSpaceScanCycleCnt);
			break;
		}
		tElem = tracking_listGetNext(tElem);
	}
	// TODO: whether abort or create new tracker using unassociated measurements for TAS_SCAN
	for (i = 0; i < measInfo->num; i++)
	{
		if (inst->bestIndex[i] != inst->tasTargetId)
		{
			inst->bestIndex[i] = TRACKING_ID_POINT_BEHIND_THE_WALL;
		}
	}
}
void tracking_twsAssociate(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTracking_objectUnit *twsTracker = NULL; 
	sTracking_objectUnit *tasTracker = NULL;
	sTrackingParams *tracking_params = inst->tracking_params;
	sMeasurement_SphPoint *measurement;
	sMeasurement_SphPoint u_tilda;
	float md = 0.f;
	float mdGating = tracking_params->advParams.twsMdGatingParams.gating;
	int i = 0;
	float dt = 0.f;
	//1.using NN to associate tws targets and measurements
	tElem = tracking_listGetFirst(&inst->twsTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		tracking_unitAssociate(inst->hTrack[uid], &inst->platformInfo, measInfo, \
			mdGating, &inst->bestIndex[0],inst->wholeSpaceScanCycleCnt,inst->lastWholeSpaceScanCycleCnt);
		tElem = tracking_listGetNext(tElem);
	}
	//2.confirm the unassociated measurements whether origined from tas targets or not, if so, abort the measurement
	for (i = 0; i < measInfo->num; i++)
	{
		measurement = &measInfo->measurement[i];
		if (inst->bestIndex[i] == TRACKING_ID_POINT_NOT_ASSOCIATED)
		{
			tElem = tracking_listGetFirst(&inst->tasTrackList);
			while (tElem != 0)
			{
				uid = tElem->data;
				tasTracker = (sTracking_objectUnit *)inst->hTrack[uid];
				// predict tas targets  
				dt = g_curTimestamp - tasTracker->timestamp;
				tracking_cal_FQ(inst, dt);
				tracking_unitPredict(inst->hTrack[uid], &inst->platformInfo);
				tracking_measurementCovCal(tasTracker);
				tracking_vectorSub(MEASUREMENT_VECTOR_SIZE, measurement->array, tasTracker->kalman_state.H_s.array, u_tilda.array);
				tracking_computeMahalanobis(u_tilda.array, tasTracker->assoProperty.gC_inv, &md);
				if (md < mdGating)
				{
					inst->bestIndex[i] = TRACKING_ID_POINT_BEHIND_THE_WALL;
				}
				tElem = tracking_listGetNext(tElem);
			}
		}
	}
}

void tracking_unitAssociate(void* handle, sTracking_platformInfo* platformInfo, sMeasOutput *measInfo,\
	float gating,uint16_t* bestIndex,uint32_t wholeSpaceScanCycleCnt, uint32_t lastWholeSpaceScanCycleCnt)
{
	sTracking_objectUnit *inst = (sTracking_objectUnit *)handle;
	uint8_t num = measInfo->num;
	sMeasurement_SphPoint u_tilda;
	sMeasurement_SphPoint* measurement;
	float md; // mah distance
    int i;
	float minMd = 1000.f;
	int8_t assocId = -1;
	int m = 0, n = 0;
	memset(&u_tilda, 0, sizeof(sMeasurement_SphPoint));
	memset((uint8_t *)(inst->assoProperty.bitIndicatorsOfDetAsso), 0, sizeof(inst->assoProperty.bitIndicatorsOfDetAsso));
	inst->assoProperty.assoCondenceMeasId = -1;
	tracking_measurementCovCal(inst);
	for (i = 0; i < num; i++)
	{
		measurement = &measInfo->measurement[i];
		tracking_vectorSub(MEASUREMENT_VECTOR_SIZE, measurement->array, inst->kalman_state.H_s.array, u_tilda.array);
		tracking_computeMahalanobis(u_tilda.array, inst->assoProperty.gC_inv, &md);
#ifdef DEBUG_LOG_ZQ
		//my_printf("trackId measId [%d %d] md %.2f", inst->uid, i, md);
#endif // DEBUG_log

		if (md < minMd)
		{
			minMd = md;
			assocId = i;
		}
	}
	if (minMd < gating)
	//if (minMd < 20.f)
	{
		inst->assoProperty.assoCondenceMeasId = assocId;
		inst->assoProperty.lastSeenTime = g_curTimestamp;
		inst->assoProperty.lastSeenWholeScanId = wholeSpaceScanCycleCnt;
		inst->assoProperty.assMahDistance = minMd;
		inst->objManagement.forcastFrameNum = 0;
		measurement= &measInfo->measurement[assocId];
		for (n = 0; n < measurement->detectionNum; n++)
		{
			m = measurement->detectionId[n];
			inst->assoProperty.bitIndicatorsOfDetAsso[m >> 3] |= (1 << (m & 0x7));
		}
		bestIndex[assocId] = inst->uid;
#ifdef DEBUG_LOG_ZQ
		my_printf("assoPairs [%d %d] md %.2f", assocId, inst->uid, minMd);
#endif // DEBUGLOG

	}
	else  // unassociated
	{
		inst->assoProperty.assMahDistance = -1.f;
		if (g_scanType == TAS_SCAN)
		{
#ifdef DEBUG_LOG
			my_printf("condenceDone %d", measInfo->condenceDone);
#endif // DEBUG_LOG
			if (measInfo->condenceDone == 1)
			{
				inst->objManagement.forcastFrameNum++;
			}
		}
		else
		{
			if (wholeSpaceScanCycleCnt - lastWholeSpaceScanCycleCnt == 1)		// start of current scan
			{
				inst->objManagement.forcastFrameNum = wholeSpaceScanCycleCnt - inst->assoProperty.lastSeenWholeScanId - 1;
			}
			
		}
#ifdef DEBUG_LOG
		//my_printf("id %d forcastNum %d", inst->uid, inst->objManagement.forcastFrameNum);
#endif // DEBUG_LOG

	}
}



/**
*  @b Description
*  @n
*		tracking Module calls this function to obtain the measurement vector scoring from the tracking unit perspective
*
*  @param[in]  handle
*		This is handle to tracking unit
*  @param[in]  point
*		This is an array of measurement points
*  @param[inout]  bestScore
*		This is a pointer current scoresheet with best scores. Only better scores are updated
*  @param[inout]  bestInd
*		This is a pointer current scoresheet winners. Only better scores winners are updated
*  @param[out]  isUnique
*       This is an array indicating whether point belongs to a single target (1) or not (0)
*  @param[in]  num
*		Number of measurement points
*
*  \ingroup tracking_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/
// Consider the ambiguity azimuth, given the multi-azimuth
void tracking_unitScore_multiAzim(void *handle, sMeasurement_SphPoint *point, float *bestScore, \
                      uint16_t *bestInd, uint8_t *isUnique, uint16_t num, uint8_t *isAssociatedActive, \
					  float *minMag_thre_array, float *mag_thre_array, float *minSNR_thre_array, float *snr_thre_array, \
					  float *rcs_thre_array, float *statVel_thre_array, void** hTrack, sTracking_platformInfo *platformInfo)
{
	sTracking_objectUnit *inst = NULL;
	uint16_t n = 0;
	uint16_t m = 0;
	uint16_t k = 0;
	uint16_t i = 0;
	uint8_t isWithinLimits = 0;
	float mdp = 0.f;
	float md = 0.f;
    sTracking_measurementSphUnion u_tilda;
	float score = 0.f;
	float rvOut = 0.f;
	uint8_t *visited = gScratchPadData->visited;
	uint16_t *neighs = gScratchPadData->neighs;

	float logdet = 0.f;
	uint16_t neighCount=0;
	uint16_t *neighCurrent = NULL;
	uint16_t *neighLast = NULL;
	int16_t DebugFlag = 0;
	//sTracking_allocationParams allocThres;
	sTracking_cartesian_position cart_c;
	sTracking_boundaryBox box;
	float width = 0.f;
	float length = 0.f;
	float max_rcs = 0;
	float max_snr = 0;
	float max_mag = 0.0f;
	float max_angelSin = 0.0f;
	float max_magCoorX = 0.f;
	float max_magCoorY = 0.f;
	float rcs_thre = 10.0f;
	float mag_thre = 0.0f;
    float sinAngle = 0.f;
	float cosAngle = 0.f;
	uint16_t validDet_index[MAX_NUM_DETECTS] = { 0U };
	uint16_t validDet_cnt = 0U;
	float rvMax = DISAMBIGUITY_MAXVEL_FASTSCAN;
	float cart_tilda[6] = { 0.f };
	float doppler_tmp = 0.f;
	float dx = 0.f;
	float dy = 0.f;
	float dz = 0.f;
	float dvrad = 0.f;
	float dx_ex[NUM_EXTERN_DOPPLER] = { 0.f };
	float dy_ex[NUM_EXTERN_DOPPLER] = { 0.f };
	float dz_ex[NUM_EXTERN_DOPPLER] = { 0.f };
	float dvrad_ex[NUM_EXTERN_DOPPLER] = { 0.f };
	int ex_idx = -2; // -1: dx, 0: dx_ex[0], 1: dx_ex[1]
	int ex_idx_array[MAX_NUM_DETECTS] = { 0 };
	float x_ = 0.f;
	float y_ = 0.f;
	float vrad_ = 0.f;
	float angle_ = 0.f;
	sTracking_gateLimits gating_;
	sTracking_gateLimits gating1_;
	sTracking_gateLimits gating2_;
	int member = 0;
	float vel_thre = 0.f;
	sTracking_gatingParams exGating_scope_;
	sTracking_gatingParams gatingParams_;
	int alive0 = 0;
	int alive1 = 0;
	int alive2 = 0;
	sTracking_objectUnit *pTracker = NULL;
	memset((uint8_t *)&u_tilda, 0, sizeof(sTracking_measurementSphUnion));
	inst = (sTracking_objectUnit *)handle;
	memset((uint8_t *)visited, 0, sizeof(uint8_t)*TRACKING_NUM_POINTS_MAX);
	memset((uint8_t *)neighs, 0, sizeof(uint16_t)*TRACKING_NUM_POINTS_MAX);
	//memset((uint8_t *)&allocThres, 0, sizeof(sTracking_allocationParams));
	memset((uint8_t *)&cart_c, 0, sizeof(sTracking_cartesian_position));
	memset((uint8_t *)&box, 0, sizeof(sTracking_boundaryBox));
	neighCurrent = neighLast = neighs;
	memset((uint8_t *)&box, 0, sizeof(sTracking_boundaryBox));
	memset(&gating_, 0, sizeof(sTracking_gateLimits));
	memset(&gating1_, 0, sizeof(sTracking_gateLimits));
	memset(&gating2_, 0, sizeof(sTracking_gateLimits));
	memset(&exGating_scope_, 0, sizeof(sTracking_gatingParams));
	memset(ex_idx_array, -1, sizeof(int)*MAX_NUM_DETECTS);

	if(!getExGatingParams(inst->kalman_state.H_s.array, &exGating_scope_, &inst->tracking_params->advParams, platformInfo)){
		exGating_scope_.limits.depth = 0.2f;
		exGating_scope_.limits.width = 0.4f;
		exGating_scope_.limits.height = 0.5f;
		exGating_scope_.limits.vel = 0.4f;
	}

	getGatingParams(inst->kalman_state.H_s.array, &gatingParams_, &inst->tracking_params->advParams, platformInfo);
	inst->assoProperty.assocGating.depth = inst->assoProperty.assocDynamicGating.depth + 2.f*exGating_scope_.limits.depth;
	inst->assoProperty.assocGating.width = inst->assoProperty.assocDynamicGating.width + 2.f*exGating_scope_.limits.width;
	inst->assoProperty.assocGating.height = inst->assoProperty.assocDynamicGating.height + 2.f*exGating_scope_.limits.height;
	inst->assoProperty.assocGating.vel = inst->assoProperty.assocDynamicGating.vel + 2.f*exGating_scope_.limits.vel;

	gating_.depth = inst->assoProperty.assocGating.depth;
	gating_.width = inst->assoProperty.assocGating.width;
	gating_.height = inst->assoProperty.assocGating.height;
	gating_.vel = inst->assoProperty.assocGating.vel;

	/*if(inst->objClass.eObjClass==CLASS_VEHICLE){
		gating_.width *= 2.f;
	}*/
	if(    fabsf(inst->kalman_state.S_hat[0])<10.f \
		&& fabsf(inst->kalman_state.S_hat[1])<10.f \
		//&& inst->objClass.eObjClass==VEHICLE 
		&& (    fabsf(inst->kalman_state.fTrackerHeading-90.f*DEG2RAD)<80.f*DEG2RAD \
			 || fabsf(inst->kalman_state.fTrackerHeading+90.f*DEG2RAD)<80.f*DEG2RAD \
			 || inst->eDynamicProperty==TRACKING_DYNAMIC_STATIC  ) \
		){
		gating_.width *= 2.0f;
		//if(inst->kalman_state.S_hat[1]<0.f){
		//	gating_.depth *= 1.0f;
		//}
		//else{
		//	gating_.depth *= 0.5f;
		//}
		gating_.vel *= 2.0f;
		//gating_.depth *= 2.0f;
		if(    fabsf(inst->kalman_state.S_hat[1])<3.f \
			&& fabsf(inst->kalman_state.S_hat[0])<4.5f ){
			gating_.depth *= 0.7f;
		}
	}
	else if(fabsf(inst->kalman_state.S_hat[3])<5.f){
	}
	if (inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC && fabsf(inst->kalman_state.S_hat[0]) < 3.f\
		&&fabsf(inst->kalman_state.S_hat[0]) < 3.f && (inst->objManagement.lifetime > 20 || \
		(inst->objManagement.heartBeatCount - inst->objManagement.allocationTime > 20 && fabsf(inst->kalman_state.fVabsY) > 5.f)) )
	{
		gating_.vel *= 1.5f;
		gatingParams_.limits.vel *= 1.5f;
	}
	inst->assoProperty.assocGating.depth = gating_.depth;
	inst->assoProperty.assocGating.width = gating_.width;
	inst->assoProperty.assocGating.vel = gating_.vel;

	if(inst->assoProperty.assocGating.depth>gatingParams_.limits.depth){
		inst->assoProperty.assocGating.depth = gatingParams_.limits.depth;
	}

	if (inst->assoProperty.assocGating.width > gatingParams_.limits.width) {
		inst->assoProperty.assocGating.width = gatingParams_.limits.width;
	}
	if(inst->assoProperty.assocGating.height>gatingParams_.limits.height){
		inst->assoProperty.assocGating.height = gatingParams_.limits.height;
	}
	if(inst->assoProperty.assocGating.vel>gatingParams_.limits.vel){
		inst->assoProperty.assocGating.vel = gatingParams_.limits.vel;
	}

	gating_.depth = inst->assoProperty.assocGating.depth;
	gating_.width = inst->assoProperty.assocGating.width;
	gating_.height = inst->assoProperty.assocGating.height;
	gating_.vel = inst->assoProperty.assocGating.vel;
	gating1_.depth = inst->assoProperty.assocGating.depth*0.5f;
	gating1_.width = inst->assoProperty.assocGating.width*0.5f;
	gating1_.height = inst->assoProperty.assocGating.height*0.5f;
	gating1_.vel = inst->assoProperty.assocGating.vel*0.5f;
	gating2_.depth = inst->assoProperty.assocGating.depth*3.f;
	gating2_.width = inst->assoProperty.assocGating.width*3.f;
	gating2_.height = inst->assoProperty.assocGating.height*3.f;
	gating2_.vel = inst->assoProperty.assocGating.vel*3.f;
#ifdef DEBUG_LOG
	if (inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC)
	{
		//my_printf("id[%d], gating_: depth: %.4f, width: %.4f, height: %.4f, vel: %.4f,SmallObjFlag:%d,state:%d,atc2free:%d\n", \
			inst->uid, gating_.depth, gating_.width, gating_.height, gating_.vel,\
			inst->objGatingCtrl.nearRegionSmallObjFlag,inst->objManagement.state,inst->objManagement.active2freeCount);
	}
#endif

	memset((uint8_t *)(inst->assoProperty.bitIndicatorsOfDetAsso), 0, sizeof(inst->assoProperty.bitIndicatorsOfDetAsso));
	memset((uint8_t*)(&inst->curTarget.unionSph), 0, sizeof(inst->curTarget.unionSph));
	memset((uint8_t*)(&inst->curTarget.unionX), 0, sizeof(inst->curTarget.unionX));
	memset((uint8_t*)(&inst->curTarget.unionY), 0, sizeof(inst->curTarget.unionY));
	
	for (n = 0; n < num; n++) {
		if (bestInd[n] == TRACKING_ID_POINT_BEHIND_THE_WALL)
			continue;
		if (point[n].mag < minMag_thre_array[n]) {
			continue;
		}
		if (point[n].snr < minSNR_thre_array[n]) {
			continue;
		}
		ex_idx_array[n] = -1;
		ex_idx = -1;

		//tracking_vectorSub(MSIZE_SPH, point[n].array, inst->kalman_state.H_s.array, u_tilda.array);
		isWithinLimits = false;
		dx = point[n].x - inst->kalman_state.S_hat[0];
		dy = point[n].y - inst->kalman_state.S_hat[1];
		dvrad = point[n].vector.doppler - inst->kalman_state.H_s.vector.doppler;
		if(1U==tracking_isInsideWithGating(dx, dy, 0.f, 0.f, &gating2_)){
			isWithinLimits = true;
		}
		// pre-select the valid detections for the tracker association
		if(isWithinLimits){
			validDet_index[validDet_cnt] = n;
			validDet_cnt++;
		}
		/* Any point outside the limits is outside the gate */
		isWithinLimits = true;
		isWithinLimits = tracking_unitScore_association( \
			&point[n], inst, &gating_, &gating1_, &gating2_, \
			point[n].vector.azimuthRad, point[n].sinAzim, point[n].cosAzim, \
			point[n].vector.doppler, dx, dy, dvrad, platformInfo,0);
       if(true==isWithinLimits){
			ex_idx = -1;
		}
		if (isWithinLimits == false)
			continue;
		ex_idx_array[n] = ex_idx;
		//if(inst->objManagement.state==TRACK_STATE_ACTIVE || inst->objManagement.state==TRACK_STATE_NEW_ACTIVE){
		//	point[n].disambigVelReliable = 1.f;
		//}
		visited[n] = 1;
		neighs[neighCount] = (uint16_t)(n);
		neighCount++;
		neighLast++;
		if (max_rcs < point[n].rcs) {
			max_rcs = point[n].rcs;
		}
		if (max_snr < point[n].snr) {
			max_snr = point[n].snr;
		}
		if (max_mag < point[n].mag) {
			max_mag = point[n].mag;
			max_magCoorX = point[n].x;
			max_magCoorY = point[n].y;
		}
		if (fabsf(max_angelSin) < fabsf(point[n].vector.azimuthRad)) {
			max_angelSin = point[n].vector.azimuthRad;
		}
	}
	while (neighCurrent != neighLast)
	{
		member = (int)(*neighCurrent++);
		for (k = 0; k < validDet_cnt; k++) {
			n = validDet_index[k];
			if (bestInd[n] == TRACKING_ID_POINT_BEHIND_THE_WALL)
				continue;
			if (visited[n] == 1) {
				continue;
			}
			// Stationary targets can associate with moving object, 
			//  but stationary object cannot associate with moving target
			//if(    inst->eDynamicProperty  ==  TRACKING_DYNAMIC_STATIC \
			//	&& point[n].flag_moving    !=                       0U)
			//if(point[n].flag_moving!=point[member].flag_moving)
			//{
			//	continue;
			//}
			if (point[n].mag < minMag_thre_array[n]) {
				continue;
			}
			if (point[n].snr < minSNR_thre_array[n]) {
				continue;
			}
			ex_idx = -1;
			isWithinLimits = false;
			x_ = point[member].x;
			y_ = point[member].y;
			vrad_ = point[member].vector.doppler;
			dx = x_ - point[n].x;
			dy = y_ - point[n].y;
			dvrad = vrad_ - point[n].vector.doppler;
			
			/* Any point outside the limits is outside the gate */
			isWithinLimits = true;
			isWithinLimits = tracking_unitScore_pointsAssociation( \
				&point[n], inst, &point[member], ex_idx_array[member], &gating_, &gating1_, &gating2_, \
				point[n].vector.azimuthRad, point[n].sinAzim, point[n].cosAzim, \
				point[n].vector.doppler, dx, dy, dvrad);

			if(true==isWithinLimits){
				ex_idx = -1;
			}
			if (isWithinLimits == false)
				continue;
			ex_idx_array[n] = ex_idx;
			//if(inst->objManagement.state==TRACK_STATE_ACTIVE || inst->objManagement.state==TRACK_STATE_NEW_ACTIVE){
			//	point[n].disambigVelReliable = 1.f;
			//}
			visited[n] = 1;
			neighs[neighCount] = (uint16_t)(n);
			neighCount++;
			neighLast++;
			if (max_rcs < point[n].rcs) {
				max_rcs = point[n].rcs;
			}
			if (max_snr < point[n].snr) {
				max_snr = point[n].snr;
			}
			if (max_mag < point[n].mag) {
				max_mag = point[n].mag;
				max_magCoorX = point[n].x;
				max_magCoorY = point[n].y;
			}
			if (fabsf(max_angelSin) < fabsf(point[n].vector.azimuthRad)) {
				max_angelSin = point[n].vector.azimuthRad;
			}
		}
	}
#ifdef DEBUG_LOG
	//my_printf("id[%d], preAssociated cnt: %d\n", inst->uid, neighCount);
#endif
	memset(&inst->assoProperty.u_tilda_sum, 0, sizeof(inst->assoProperty.u_tilda_sum));
	inst->curTarget.uNumOfDetections = 0;
	for(m=0; m<neighCount; m++)
	{
		n = (int)(neighs[m]);
		tracking_vectorSub(MSIZE_SPH, point[n].array, inst->kalman_state.H_s.array, u_tilda.array);
		/* Radial velocity estimation is known */
		// Consider the long object when crossing
		sinAngle = point[n].sinAzim;
		cosAngle = point[n].cosAzim;
		//myMath_sincosd(point[n].vector.azimuthRad*RAD2DEG, &sinAngle, &cosAngle);
		doppler_tmp = inst->kalman_state.S_hat[2]*cosAngle + inst->kalman_state.S_hat[3]*sinAngle;
		rvOut = point[n].vector.doppler;
		//point[n].disambigVelReliable = 1.0f;
		//rvOut = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->kalman_state.H_s.vector.doppler, point[n].vector.doppler);
		u_tilda.vector.azimuthRad = point[n].vector.azimuthRad - inst->kalman_state.H_s.vector.azimuthRad;
		u_tilda.vector.doppler = rvOut - doppler_tmp;
		/* For the gating purposes we compute partial Mahalanobis distance, ignoring doppler */		
		if(inst->kalman_state.uType==0U)
		{
			tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u_tilda.array, \
				cart_tilda);
			cart_tilda[2] = sqrtf(cart_tilda[2]*cart_tilda[2]+cart_tilda[3]*cart_tilda[3]);
			// mdp = v*D*v'; v(x,y,vr), D(3x3) for x,y,vr
			tracking_computeMahalanobisPartial(cart_tilda, inst->assoProperty.gC_inv, &mdp);
		}
		else
		{
			tracking_computeMahalanobisPartial(u_tilda.array, inst->assoProperty.gC_inv, &mdp);
		}
		//printf("---- mdp: %.2f \n", mdp);
		/* Gating Step */
		//if(1 || mdp < inst->G) {
		//if (fabsf(inst->max_rcs - max_rcs) < 10 && fabsf(inst->max_mag-max_mag)<5 && mdp < inst->G) {
		if(1 || mdp < inst->assoProperty.G) 
		{
			if(    inst->objManagement.state==TRACK_STATE_ACTIVE 
				|| inst->objManagement.state==TRACK_STATE_NEW_ACTIVE 
				|| isAssociatedActive[n]==0)
			{
				//if(bestInd[n] < TRACKING_NUM_TRACKS_MAX)
				//    /* This point is not unique. Clear the indication */
				isUnique[n>>3] &= ~(1<<(n & 0x0007));
				inst->assoProperty.bitIndicatorsOfDetAsso[n>>3] |= (1<<(n&0x7));
				inst->curTarget.uNumOfDetections++;
				/* Scoring */
				tracking_computeMahalanobis(u_tilda.array, inst->assoProperty.gC_inv, &md);
				score = logdet + md;
				if(    (	inst->eDynamicProperty  ==  TRACKING_DYNAMIC_STATIC \
						 && point[n].flag_moving    !=                       0U) \
					|| (	inst->eDynamicProperty  !=  TRACKING_DYNAMIC_STATIC \
						 && point[n].flag_moving    ==                       0U)    ){
					score = score * 2.0f;
				}
				if(    (score < bestScore[n] && isAssociatedActive[n]==0 && inst->objManagement.state==TRACK_STATE_DETECTION) \
					|| (score < bestScore[n] && isAssociatedActive[n]==1 && inst->objManagement.state!=TRACK_STATE_DETECTION) \
					|| bestInd[n]==TRACKING_ID_POINT_NOT_ASSOCIATED) {
					/* If we are the best, register our score, and the index */
					bestScore[n] = score;
					bestInd[n] = (uint16_t)inst->uid;
				}
				if(inst->objManagement.state==TRACK_STATE_ACTIVE || inst->objManagement.state==TRACK_STATE_NEW_ACTIVE)
				{
					isAssociatedActive[n] = 1;
				}

			}
		}
	}
}


uint8_t tracking_unitScore_association( \
	sMeasurement_SphPoint* point, void* handle,\
	sTracking_gateLimits* gating_, sTracking_gateLimits* gating1_, sTracking_gateLimits* gating2_, \
	float azimuthRad, float sinAngle, float cosAngle, float doppler, float dx, \
	float dy, float dvrad, sTracking_platformInfo* platformInfo,uint8_t overtakeFlag)
{
	sTracking_objectUnit *inst = NULL;
	uint8_t isWithinLimits = true;
	float doppler_tmp = 100.f;
	float rvOut = 0.f;
	float vel_thre = gating_->vel;
	float egoVel = platformInfo->egoLineVel_lat;
	sTracking_gateLimits gating_tmp;
	sTracking_gateLimits gating_overtake;
	float absVel_thres = 2.f;
	float flag_asso = -1.f;
	int alive = 0;
	
	inst = (sTracking_objectUnit *)handle;
	memcpy(&gating_tmp, gating_, sizeof(gating_tmp));
	memcpy(&gating_overtake, gating_, sizeof(gating_overtake));
	gating_overtake.depth *= 1.5f;
	alive = (int)(inst->objManagement.heartBeatCount-inst->objManagement.allocationTime);
	alive = (alive>1000)? (1000) : (alive);

	if(inst->eDynamicProperty  ==  TRACKING_DYNAMIC_STATIC){
		if(point->flag_moving==0U){
			if(0U==tracking_isInsideWithGating(dx, dy, 0.f, \
				0.f, gating1_)){
				isWithinLimits = false;
				flag_asso = 1.f;
			}
		}
		else{
			isWithinLimits = false;
			flag_asso = 2.f;
		}
	}
	else{		// moving or stopped tracker
		if (fabsf(inst->kalman_state.fVabsX) > 2.f && fabsf(inst->kalman_state.S_hat[1]) > 5.f&&\
			fabsf(platformInfo->egoLineVel_lat)<1.f)
		{
			if (point->flag_moving == 0U)
			{
				isWithinLimits = false;
				flag_asso = 18.f;
			}
		}
		if(point->flag_moving==1U){
			if (overtakeFlag == 0||dx>0)
			{
				if (0U == tracking_isInsideWithGating(dx, dy, 0.f, \
					0.f, &gating_tmp)) {
					isWithinLimits = false;
					flag_asso = 3.f;
				}
			}
			else 
			{
				if (0U == tracking_isInsideWithGating(dx, dy, 0.f, \
					0.f, &gating_overtake)) {
					isWithinLimits = false;
					flag_asso = 3.1f;
				}
			}
			
		}
		else{
			// Enable assocation between moving object and static target, 
			//  when the object is stable and from long distance
		}
	}

	if (isWithinLimits == false)
		return false;
	/* Radial velocity estimation is not yet known, unroll based on velocity measured at allocation time */
	// Consider the long object when crossing
	doppler_tmp = inst->kalman_state.S_hat[2]*cosAngle + inst->kalman_state.S_hat[3]*sinAngle;
	//doppler_tmp = inst->kalman_state.H_s.vector.doppler;
	rvOut = doppler;
	dvrad = rvOut - doppler_tmp;

//	}
//	else
//	{
////#ifdef DEBUG_LOG
////		//my_printf("id[%d], azimuth: %.4f\n", inst->uid, point->vector.azimuthRad*RAD2DEG);
////#endif
//		if(inst->eDynamicProperty  ==  TRACKING_DYNAMIC_STATIC){
//			vel_thre = gating1_->vel*0.5f;
//		}
//		//else if(fabsf(point[n].vector.azimuthRad*RAD2DEG)<3.0f){
//		//	vel_thre = inst->assoProperty.H_limits.vector.doppler*2.f;
//		//}
//		else{
//			vel_thre = gating_->vel;
//		}
//		if (fabsf(dvrad) > vel_thre)
//		{
//			isWithinLimits = false;
//			flag_asso = 7.f;
//		}
//	}
#ifdef DEBUG_LOG
if (inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC)
{
	/*my_printf("222 id[%d], flag_asso: %.1f\n", \
		inst->uid, flag_asso);*/
}
	
#endif
#ifdef DEBUG_LOG
	//my_printf("id[%d], point.r: %.3f, point.vel: %.3f, dvrad: %.4f, vel_thre: %.4f, isWithLimits: %d\n", \
		inst->uid, point->vector.range, point->vector.doppler, dvrad, vel_thre, (int)(isWithinLimits));
#endif
	if(false==isWithinLimits){
		return false;
	}
	return true;
}

uint8_t tracking_unitScore_pointsAssociation( \
	sMeasurement_SphPoint* point1, void* handle, sMeasurement_SphPoint* point0, \
	int ex_idx, sTracking_gateLimits* gating_, sTracking_gateLimits* gating1_, sTracking_gateLimits* gating2_, \
	float azimuthRad, float sinAngle, float cosAngle, float doppler, float dx, float dy, float dvrad)
{
	sTracking_objectUnit *inst = NULL;
	uint8_t isWithinLimits = true;
	float doppler_tmp = 100.f;
	float rvOut = 0.f;
	//float dvrad_temp = 100.f;
	float vel_thre = gating_->vel;
	sTracking_gateLimits gating_tmp;
	
	inst = (sTracking_objectUnit *)handle;
	memcpy(&gating_tmp, gating_, sizeof(gating_tmp));

	/* Any point outside the limits is outside the gate */
	isWithinLimits = true;
	return true;
}



