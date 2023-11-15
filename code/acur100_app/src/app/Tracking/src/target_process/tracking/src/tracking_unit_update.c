#include <string.h>
#include <math.h>
#include <float.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"


//#define TRACKING_POS_DIMENSION (0U)
//#define TRACKING_VEL_DIMENSION (1U)
//#define TRACKING_ACC_DIMENSION (2U)

void tracking_tasUpdate(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTracking_objectUnit *tasTracker = NULL;
	sTrackingParams *tracking_params = inst->tracking_params;
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		if (uid == inst->tasTargetId)
		{
			tracking_unitUpdate(inst->hTrack[uid], &inst->platformInfo, measInfo);
			break;
		}
		tElem = tracking_listGetNext(tElem);
	}
}
void tracking_twsUpdate(sTracking_moduleInstance *inst, sMeasOutput *measInfo)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTracking_objectUnit *twsTracker = NULL;
	sTrackingParams *tracking_params = inst->tracking_params;
	tElem = tracking_listGetFirst(&inst->twsTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		tracking_unitUpdate(inst->hTrack[uid], &inst->platformInfo, measInfo);
		tElem = tracking_listGetNext(tElem);
	}
}

/**
*  @b Description
*  @n
*		tracking Module calls this function to perform an update step for the tracking unit. 
*
*  @param[in]  handle
*		This is handle to tracking unit
*  @param[in]  point
*		This is an array of measurement points
*  @param[in]  var
*      Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
*  @param[in]  pInd
*		This is an array of associated UIDs. After association and allocation steps, each measurment shall have a UID assigned.
*  @param[out] isUnique
*       This is an array indicating whether point belongs to a single target (1) or not (0).
*  @param[in]  num
*		Number of measurement points
*
*  \ingroup tracking_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void tracking_unitUpdate(void *handle, sTracking_platformInfo *platformInfo, \
	sMeasOutput *measInfo)
{
    sTracking_objectUnit *inst = NULL;
	uint16_t n = 0;
	uint16_t m = 0;
	float trajDistance = 0.f, dx = 0.f, dy = 0.f, dz = 0.f;
	float u_tilda[MSIZE_SPH];

	float J[MSIZE_SPH*SSIZE_XY] = { 0.f };
	float PJ[MSIZE_SPH*SSIZE_XY] = { 0.f };
	float JPJ[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float cC_inv[MSIZE_SPH*MSIZE_SPH] = { 0.f }; 
	float K[SSIZE_XY*MSIZE_SPH] = { 0.f };
    /* Rm is Measurement error covariance matrix */
	float Rm[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float Meas_var[MSIZE_SPH] = { 0.f };

	float temp1[SSIZE_XY*SSIZE_XY] = { 0.f };
	float temp2[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float temp3[SSIZE_XY*SSIZE_XY] = { 0.f };
	float det = 0;
	int dimLen = 0;
	int i = 0;
	int j = 0, num_P = 0;

    inst = (sTracking_objectUnit *)handle;
	dimLen = inst->tracking_params->stateVectorDimLength;
	if (inst->assoProperty.assoCondenceMeasId >= 0) {
		//// 1. Set prediction state with saved state
		//// 2. The compesanted state is just used for association
		//tracking_measurementCovCal(inst);
		/* Compute state vector partial derivatives (Jacobian matrix) */
		tracking_computeJacobian(inst->tracking_params->stateVectorType, inst->kalman_state.S_hat, J);
		/* Compute JPJ' = J(:,1:MSIZE_SPH) * obj.P_apriori(1:MSIZE_SPH,1:MSIZE_SPH) * J(:,1:MSIZE_SPH)' */
		tracking_matrixComputePJT(inst->kalman_state.P_hat, J, PJ);
		tracking_matrixMultiply(MSIZE_SPH, SSIZE_XY, MSIZE_SPH, J, PJ, JPJ);
		if (g_scanType == TWS_SCAN)
		{
			tracking_measurement_variance_cal(inst, const_tws_measurement_noise, Meas_var);
		}
		else
		{
			tracking_measurement_variance_cal(inst, const_tas_measurement_noise, Meas_var);
		}
		tracking_matrixSetDiag(MSIZE_SPH, Meas_var, Rm);

	    tracking_matrixAdd(MSIZE_SPH, MSIZE_SPH, JPJ, Rm, temp2);
	    /* Compute inverse of cC */
        tracking_matrixInv(temp2, &det, cC_inv);
	    tracking_matrixMultiply(SSIZE_XY, MSIZE_SPH, MSIZE_SPH, PJ, cC_inv, K);

		/* Compute innovation */
		tracking_matrixSub(MSIZE_SPH, 1, measInfo->measurement[inst->assoProperty.assoCondenceMeasId].array, \
			inst->kalman_state.H_s.array, u_tilda);
		/* State estimation */
		/* obj.S_hat(1:MSIZE_SPH) = obj.S_apriori_hat(1:MSIZE_SPH) + K * u_tilda */
		tracking_matrixMultiply(SSIZE_XY, MSIZE_SPH, 1, K, u_tilda, temp1);
		tracking_matrixAdd(SSIZE_XY,1, inst->kalman_state.S_hat, temp1, inst->kalman_state.S_hat);
	    /* Covariance estimation */
        /* obj.P(1:MSIZE_SPH,1:MSIZE_SPH) = obj.P_apriori(1:MSIZE_SPH,1:MSIZE_SPH) - K * J(:,1:MSIZE_SPH) * obj.P_apriori(1:MSIZE_SPH,1:MSIZE_SPH) */
	    tracking_matrixTransposeMultiply(SSIZE_XY, MSIZE_SPH, SSIZE_XY, K, PJ, temp1);
	    tracking_matrixSub(SSIZE_XY, SSIZE_XY, inst->kalman_state.P_hat, temp1, temp3);
		tracking_matrixMakeSymmetrical(SSIZE_XY, temp3, inst->kalman_state.P_hat);

#ifdef DEBUG_LOG_ZQ
		//my_printf("--update info-- Id %d x %.2f, y %.2f, z %.2f,vx %.2f, vy %.2f, vz %.2f, ax %.2f, ay %.2f, az %.2f,convY %.2f", \
			inst->uid, inst->kalman_state.S_hat[0], inst->kalman_state.S_hat[1], inst->kalman_state.S_hat[2], inst->kalman_state.S_hat[3], \
			inst->kalman_state.S_hat[4], inst->kalman_state.S_hat[5], inst->kalman_state.S_hat[6], inst->kalman_state.S_hat[7], \
			inst->kalman_state.S_hat[8], inst->kalman_state.P_hat[10]);
#endif // DEBUG_LOG

    }
	tracking_cartesian2spherical(inst->tracking_params->stateVectorType, inst->kalman_state.S_hat, inst->kalman_state.H_s.array);
	dx = inst->kalman_state.S_hat[0] - inst->objKinematic.initPosition.x0;
	dy = inst->kalman_state.S_hat[1] - inst->objKinematic.initPosition.y0;
	dz = inst->kalman_state.S_hat[2] - inst->objKinematic.initPosition.z0;
	trajDistance = sqrtf(dx*dx + dy * dy + dz * dz);
	if (trajDistance > inst->objKinematic.maxDistance)
	{
		inst->objKinematic.maxDistance = trajDistance;
	}
	//tracking_unitEvent(inst, inst->assoProperty.assoCondenceMeasId, platformInfo);
	//return(inst->objManagement.state);
}


void tracking_velocityStateHandling(void *handle, sMeasurement_Sph_vector *uVec)
{
	sTracking_objectUnit *inst;
	float instanteneousRangeRate;
	float rrError;
	float rvError;
	float rvIn;
	float denominator = 0.0001f;

	inst = (sTracking_objectUnit *)handle;
    rvIn = uVec->doppler;

	switch(inst->objKinematic.velocityHandling) {

		case VELOCITY_INIT:
            uVec->doppler = inst->objKinematic.rangeRate;
			inst->objKinematic.velocityHandling = VELOCITY_RATE_FILTER;
			break;

		case VELOCITY_RATE_FILTER:
			/* In this state we are using filtered Rate Range to unroll radial velocity, stabilizing Range rate */
            instanteneousRangeRate = (uVec->range - inst->objManagement.allocationRange)/((inst->objManagement.heartBeatCount-inst->objManagement.allocationTime)*inst->tracking_params->dt);

			inst->objKinematic.rangeRate = inst->tracking_params->advParams.unrollingParams.alpha * inst->objKinematic.rangeRate + (1-inst->tracking_params->advParams.unrollingParams.alpha) * instanteneousRangeRate;
#ifdef ENABLE_DISAMBIGUITYVELOCITY
            uVec->doppler = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->objKinematic.rangeRate, rvIn);
#else
			uVec->doppler = rvIn;
#endif

			denominator = inst->objKinematic.rangeRate;
			myMath_checkZero(&denominator);
			rrError = (instanteneousRangeRate - inst->objKinematic.rangeRate)/denominator;
		
			if(fabsf(rrError) < inst->tracking_params->advParams.unrollingParams.confidence) {
				inst->objKinematic.velocityHandling = VELOCITY_TRACKING;
			}
			else {	
			}
			break;

		case VELOCITY_TRACKING:
			/* In this state we are using filtered Rate Range to unroll radial velocity and monitoring Hs error */
			denominator = (inst->objManagement.heartBeatCount-inst->objManagement.allocationTime)*inst->tracking_params->dt;
			myMath_checkZero(&denominator);
            instanteneousRangeRate = (uVec->range - inst->objManagement.allocationRange)/denominator;

			inst->objKinematic.rangeRate = inst->tracking_params->advParams.unrollingParams.alpha * inst->objKinematic.rangeRate + (1-inst->tracking_params->advParams.unrollingParams.alpha) * instanteneousRangeRate;
#ifdef ENABLE_DISAMBIGUITYVELOCITY
            uVec->doppler = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->objKinematic.rangeRate, rvIn);
#else
			uVec->doppler = rvIn;
#endif

			denominator = uVec->doppler;
			myMath_checkZero(&denominator);
            rvError = (inst->kalman_state.H_s.vector.doppler - uVec->doppler)/denominator;
			if(fabsf(rvError) < 0.1f) {
				inst->objKinematic.velocityHandling = VELOCITY_LOCKED;
			}
			else {
			}
			break;

		case VELOCITY_LOCKED:
#ifdef ENABLE_DISAMBIGUITYVELOCITY
            uVec->doppler = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->kalman_state.H_s.vector.doppler, uVec->doppler);
#else
			uVec->doppler = rvIn;
#endif
			break;
	}
}

// update eDynamicProperty
void tracking_unit_dynamicUpdate(void* handle, sTracking_platformInfo* platformInfo)
{
	float static_vel_thres = 0.f;
	float static_velx_thres = 0.f;
	sTracking_objectUnit *inst = NULL;
	float condition_flag = -1.f;
	float zone_track_y = 0.0f;//judge target fall in which zone

	inst = (sTracking_objectUnit*)(handle);
	

	getStaticThres(inst->kalman_state.H_s.array, &static_vel_thres, &static_velx_thres, \
		platformInfo->egoLineVel_long, platformInfo->egoLineVel_lat, platformInfo->egoOmega);
#ifdef DEBUG_LOG
	if (inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC) {

		//my_printf("id[%d], DynamicProperty: %d, static_thres: %.3f | %.3f\n", \
			inst->uid, \
			(int)(inst->eDynamicProperty), \
			static_vel_thres, \
			static_velx_thres);
	}
#endif
	// update eDynamicProperty

#ifdef DEBUG_LOG
	if (inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC) {
		//my_printf("inst[%d],life:%d, eDynamicProperty: %d,fabsVx:%f, fabsVy:%f, assoM:%d, predCnt:%d, condition:%.2f \n", \
			inst->uid, inst->objManagement.heartBeatCount - inst->objManagement.allocationTime, \
			(int)(inst->eDynamicProperty),inst->kalman_state.fVabsX,inst->kalman_state.fVabsY, \
			inst->curTarget.uFlagMoving*inst->curTarget.uNumOfDetections,inst->objManagement.active2freeCount,condition_flag);
	}
	
#endif


//	if(    fabsf(inst->kalman_state.fVabsX)	<	static_velx_thres \
//		&& fabsf(inst->kalman_state.fVabsY)	<	static_velx_thres \
//		&& inst->eDynamicProperty			!=		  TRACKING_DYNAMIC_STATIC \
//		)
//	{
//		if(    inst->objGeometryProperty.historyTrajAbsLength > 5.f \
//			&& (    inst->objGeometryProperty.assocMovingCounter > 10 \
//				 || inst->curTarget.uFlagMoving == 1U \
//				 || inst->objGeometryProperty.historyTrajAbsLength > 10.f) \
//			){
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STOP;
//		}
//		else{
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STATIC;
//		}
//		inst->kalman_state.S_hat[4] = platformInfo->egoLineAcc_long; //0.f;
//		inst->kalman_state.S_hat[5] = platformInfo->egoLineAcc_lat; //0.f;
//		inst->kalman_state.S_x_hat[2] = inst->kalman_state.S_hat[4];
//		inst->kalman_state.S_y_hat[2] = inst->kalman_state.S_hat[5];

//	#ifdef DEBUG_LOG
//		//my_printf("inst[%d], 11 \n", inst->uid);
//	#endif
//	}
//	//else if(   inst->objGeometryProperty.assocStatCounter > 2*inst->objGeometryProperty.assocMovingCounter \
//	//		&& inst->objGeometryProperty.assocStatCounter > 30 \
//	//		&& inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC \
//	//		&& 0U==isInZeroDegreeRegion(inst->kalman_state.H_s.array) ){
//	//	// For the points or tracker of angle=0
//	//	inst->eDynamicProperty = TRACKING_DYNAMIC_STATIC;
//	//	inst->kalman_state.S_hat[4] = platformInfo->egoLineAcc_long; //0.f;
//	//	inst->kalman_state.S_hat[5] = platformInfo->egoLineAcc_lat; //0.f;
//	//	fVx_tmp = -1.f*platformInfo->egoLineVel_long + fYawRate*(inst->kalman_state.S_y_hat[0]+platformInfo->radarDist_lat);
//	//	fVy_tmp = -1.f*platformInfo->egoLineVel_lat - fYawRate*(inst->kalman_state.S_x_hat[0]+platformInfo->radarDist_long);
//	//	//lowPassFilter, x = k*input + (1-k)*x
//	//	myMath_lowPassFilter(&inst->kalman_state.S_hat[2], fVx_tmp, 0.7f);
//	//	myMath_lowPassFilter(&inst->kalman_state.S_hat[3], fVy_tmp, 0.7f);
//	//	inst->kalman_state.S_hat[4] = platformInfo->egoLineAcc_long; //0.f;
//	//	inst->kalman_state.S_hat[5] = platformInfo->egoLineAcc_lat; //0.f;
//	//	inst->kalman_state.S_x_hat[1] = inst->kalman_state.S_hat[2];
//	//	inst->kalman_state.S_y_hat[1] = inst->kalman_state.S_hat[3];
//	//	inst->kalman_state.S_x_hat[2] = inst->kalman_state.S_hat[4];
//	//	inst->kalman_state.S_y_hat[2] = inst->kalman_state.S_hat[5];
//	//}
//	else if(   inst->objGeometryProperty.assocStatCounter > inst->objGeometryProperty.assocMovingCounter \
//			&& inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC \
//			&& fabsf(inst->kalman_state.fVabsX)	<	2.f*static_velx_thres \
//			&& fabsf(inst->kalman_state.fVabsY)	<	2.f*static_velx_thres \
//			&& 0U==isInVeryNearRegion(inst->kalman_state.H_s.array) )
//	{
//		// For the points or tracker of angle=0
//		if(inst->objGeometryProperty.historyTrajAbsLength > 10.f){
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STOP;
//		}
//		else{
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STATIC;
//		}
//		//fVx_tmp = -1.f*platformInfo->egoLineVel_long + fYawRate*(inst->kalman_state.S_y_hat[0]+platformInfo->radarDist_lat);
//		//fVy_tmp = -1.f*platformInfo->egoLineVel_lat - fYawRate*(inst->kalman_state.S_x_hat[0]+platformInfo->radarDist_long);
//		////lowPassFilter, x = k*input + (1-k)*x
//		//myMath_lowPassFilter(&inst->kalman_state.S_hat[2], fVx_tmp, 0.7f);
//		//myMath_lowPassFilter(&inst->kalman_state.S_hat[3], fVy_tmp, 0.7f);
//		inst->kalman_state.S_hat[4] = platformInfo->egoLineAcc_long; //0.f;
//		inst->kalman_state.S_hat[5] = platformInfo->egoLineAcc_lat; //0.f;
//		//inst->kalman_state.S_x_hat[1] = inst->kalman_state.S_hat[2];
//		//inst->kalman_state.S_y_hat[1] = inst->kalman_state.S_hat[3];
//		inst->kalman_state.S_x_hat[2] = inst->kalman_state.S_hat[4];
//		inst->kalman_state.S_y_hat[2] = inst->kalman_state.S_hat[5];
//	#ifdef DEBUG_LOG
//		//my_printf("inst[%d], 22 \n", inst->uid);
//	#endif
//	}
//	else if(    (    (    fabsf(inst->kalman_state.fVabsX)	>	2.f*static_velx_thres \
//					  || fabsf(inst->kalman_state.fVabsY)	>	2.f*static_velx_thres) \
//				  && (inst->eDynamicProperty					==			 TRACKING_DYNAMIC_STATIC) \
//				  && (    inst->objGeometryProperty.historyTrajAbsLength > 10.f \
//					   && inst->objGeometryProperty.assocMovingCounter > 20 ) ) \
//			 || (    (    fabsf(inst->kalman_state.fVabsX)	>	static_velx_thres \
//					   || fabsf(inst->kalman_state.fVabsY)	>	static_velx_thres ) \
//				  && (inst->eDynamicProperty					==			 TRACKING_DYNAMIC_STATIC) \
//				  && (inst->objGeometryProperty.historyTrajAbsLength > 2.f) \
//				  //&& inst->objGeometryProperty.assocStatCounter < inst->objGeometryProperty.assocMovingCounter 
//				  && inst->curTarget.uFlagMoving == 1U \
//				  && inst->objGeometryProperty.assocMovingCounter >10 \
//				  && (fabsf(platformInfo->egoLineVel_lat)<0.1f) \
//				  && (fabsf(platformInfo->egoLineVel_long)<0.1f) ) )
//	{
//		inst->eDynamicProperty = TRACKING_DYNAMIC_UNKNOWN;
//	#ifdef DEBUG_LOG
//		//my_printf("inst[%d], 33 \n", inst->uid);
//	#endif
//	}
//	else if(    (    fabsf(inst->kalman_state.fVabsX)	>	1.5f*static_velx_thres \
//				  || fabsf(inst->kalman_state.fVabsY)	>	1.5f*static_velx_thres) \
//			 && (inst->eDynamicProperty					==			 TRACKING_DYNAMIC_STATIC) \
//			 && inst->curTarget.uFlagMoving == 1U \
//			 && inst->objGeometryProperty.assocMovingCounter >20 )\
//	{
//		if(inst->objGeometryProperty.historyTrajAbsLength > 10.f){
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STOP;
//		}
//		else{
//			inst->eDynamicProperty = TRACKING_DYNAMIC_UNKNOWN;
//		}
//	#ifdef DEBUG_LOG
//		//my_printf("inst[%d], 44 \n", inst->uid);
//	#endif
//	}
//	else if(    (inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC) \
//			 && inst->objGeometryProperty.historyTrajAbsLength < 10.f \
//			 && inst->objManagement.state == TRACK_STATE_DETECTION \
//			 && inst->objGeometryProperty.assocStatCounter >10 )\
//	{
//		if(inst->objGeometryProperty.historyTrajAbsLength > 5.f \
//			&& inst->objClass.objFeatures.absVelRad_max>(5.f/3.6f) \
//		  ){
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STOP;
//		}
//		else{
//			inst->eDynamicProperty = TRACKING_DYNAMIC_STATIC;
//		}
//	#ifdef DEBUG_LOG
//		//my_printf("inst[%d], 55 \n", inst->uid);
//	#endif
//	}
//	//else if(inst->objClass.objFeatures.absVelRad_max>static_vel_thres){
//	//	if(    inst->objGeometryProperty.historyTrajAbsLength > 5.f \
//	//		|| fabsf(inst->kalman_state.fVabsX)	>	static_velx_thres \
//	//		|| fabsf(inst->kalman_state.fVabsY)	>	static_velx_thres ){
//	//		inst->eDynamicProperty = TRACKING_DYNAMIC_UNKNOWN;
//	//	}
//	//	else{
//	//		inst->eDynamicProperty = TRACKING_DYNAMIC_STOP;
//	//	}
//	//}
//	else{
//	#ifdef DEBUG_LOG
//		//my_printf("inst[%d], else \n", inst->uid);
//	#endif
//	}

}

