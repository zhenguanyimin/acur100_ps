/**
*   @file  tracking_unit_currentTarget.c
*
*   @brief
*      Unit level update function for the tracking Algorithm
*
*  \par
*  NOTE:
*      (C) Copyright 2017 Texas Instruments, Inc.
*/

#include <string.h>
#include <math.h>
#include <float.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

#define TRACKING_SPREAD_ALPHA     	                (0.01f) /* 1 percent */
#define TRACKING_MIN_DISPERSION_ALPHA	                (0.1f)  /* 10 percent */
#define TRACKING_MIN_POINTS_TO_UPDATE_DISPERSION      (3)     /* 3 points */

#define USING_CONSTMEANOISE 1

/**
*  @b Description
*  @n
*		tracking Module calls this function to calculate current target for the tracking unit. 
*/
void tracking_unitCurrentTargetWithWeightScale(void *handle, sMeasurement_SphPoint *point, sMeasurement_Sph_vector *var,
	uint16_t *pInd, uint8_t *isUnique, uint16_t num, sTracking_platformInfo *platformInfo)
{
	sTracking_objectUnit *inst = NULL;
	uint16_t n = 0;
	uint16_t m = 0;
	uint16_t k = 0;
	uint32_t cnt = 0; // benyao 1206

	uint32_t myPointNum = 0;
	uint32_t myUniquePointNum = 0;
	float alpha = 0.f;
	float rvPilot = 0.f;
	float vel = 0.f;
	float u_max[MSIZE_SPH];
	float u_min[MSIZE_SPH];
#ifdef HUHANG

	float cart_max3[MSIZE_SPH];//x,y,doppler
	float cart_min3[MSIZE_SPH];//x,y,doppler
#endif // DEBUG
	float cart_max[SSIZE_XY];
	float cart_min[SSIZE_XY];

	float cart_center[SSIZE_XY];

	/* D is Dispersion matrix */
	float D[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	/* Rm is Measurement error covariance matrix */
	//float *Rm = gScratchPadData->Rm;
	/* Rc is Measurement error covariance matrix for the centroid used for Kalman update */
	float Rc[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float length = 0.5f;
	float width = 0.5f;
	float height = 0.5f;
	float fSNR = 0.f;
	float fRCS = -1000.f;
	float fMag = 0.f;
	float fAngel = 0.f;
	float fMagCoorX = 0.f;
	float fMagCoorY = 0.f;
	float fPDH0 = 0.1f;

	float mag_array[MAX_NUM_DETECTS] = { 0.f };
	float fMag_min = 1000.f;
	float fMag_mean = 0.f;
	float fMag_spread = 0.f;
	float fMag_middle = 0.f;
	float fMag_stdVar = 0.f;

	float fYawRate = (fabsf(platformInfo->egoOmega)<0.5f*DEG2RAD)? 0.0f : platformInfo->egoOmega;
	float fSensorVx = platformInfo->egoLineVel_long - fYawRate*platformInfo->radarDist_lat;
	float fSensorVy = platformInfo->egoLineVel_lat + fYawRate*platformInfo->radarDist_long;

	sTracking_measurementSphUnion 	u;
	sTracking_measurementSphUnion 	u_sum;
	sTracking_measurementSphUnion 	u_centroid;
#ifdef HUHANG
	float u_cart3[MSIZE_SPH] = { 0.f };//x,y,doppler
	float u_cart_centroidi[MSIZE_SPH] = { 0.f };//x,y,doppler
	float u_cart_centroid[MSIZE_SPH]= { 0.f };//x,y,doppler
#endif // HUHANG
	float u_cart[SSIZE_XY] = { 0.f };
	
	float centroidSnr = 0.f;
	float spread = 0.f;
	float sigma = 0.f;
	float rvIn = 0.f;
	uint8_t targetCanStop = true;

	sTracking_measurementSphUnion 	uvar;
	sTracking_measurementSphUnion     uvar_sum;
	sTracking_measurementSphUnion 	uvar_mean;
	uint8_t flag_nearOrFarScan = 0U;
	uint8_t flag_moving = 0U;
	uint16_t flag_moving_cnt = 0U;
	sTracking_measurementSphUnion 	rm_diag;
	float fVradAbs = 100.f;
	uint16_t idxOfDetsAssoc[MAX_NUM_DETECTS] = { 0 };
	uint16_t idxOfDetsAssoc_num = 0;
	uint8_t isInDyncGating[MAX_NUM_DETECTS] = { 0 };
	float weightOfDetsAssoc[MAX_NUM_DETECTS] = { 0.f };
	float weightOfDetsAssoc_range[MAX_NUM_DETECTS] = { 0.f };
	float weightOfDetsAssoc_azimuthRad[MAX_NUM_DETECTS] = { 0.f };
	float weightOfDetsAssoc_doppler[MAX_NUM_DETECTS] = { 0.f };
	float weightOfDetsAssoc_snr[MAX_NUM_DETECTS] = { 0.f };
	float weightOfDetsAssoc_rcs[MAX_NUM_DETECTS] = { 0.f };
	float weightOfDetsAssoc_mag[MAX_NUM_DETECTS] = { 0.f };
	float doppler_rvOut[MAX_NUM_DETECTS] = { 0.f };
	float doppler_tmp_vec[MAX_NUM_DETECTS] = { 0.f };
	float sumWeightOfDetsAssoc = 0.f;
	float minLong = 100.f;
	float minLat = 100.f;
	float weightScale_minLong = 0.f;
	float weightScale_minLat = 0.f;
	float weightScale_moving = 0.f;
	//float sum_range = 0.f;
	//float sum_doppler = 0.f;
	//float sum_azimuthRad = 0.f;
	//float sum_snr = 0.f;
	//float sum_rcs = 0.f;
	//float sum_mag = 0.f;
	float rvMax = 0.f;
	float rvOut = 0.f;
	float doppler_tmp = 0.f;
	float sinAngle = 0.f;
	float cosAngle = 0.f;
	sMeasurement_SphPoint u_tilda;
	uint8_t isWithinLimits = true;
	float curTarget_cart[SSIZE_XY] = { 0.f };
	sTracking_gateLimits assocDynamicGating_tmp; 
	float scale_tmp = 0.1f;
	int flagValid = 0;
	int flagDynGatingValid = 0;
	int i = 0;
	int dimLen = 0;
	float dx = 100.f;
	float dy = 100.f;
	

	float AssoW = 0.0f, AssoD = 0.0f, AssoVel = 0.0f;
	float Width = 0.0f, Depth = 0.0f, Vel = 0.0f;


	float static_vel_thres = TRACKING_MIN_STATIC_VELOCITY_RAD;
	float static_velx_thres = TRACKING_MIN_STATIC_VELOCITY;
	uint8_t movingAssoFlag = 0U;
	uint8_t assoOverCnt = 0;
	uint8_t nearCount1to2 = 0;
	memset((uint8_t *)&u_centroid, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&u, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&uvar_mean, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&uvar, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&rm_diag, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&u_tilda, 0, sizeof(sMeasurement_SphPoint));
	memset((uint8_t *)&u_sum, 0, sizeof(u_sum));
	memset((uint8_t *)&assocDynamicGating_tmp, 0, sizeof(assocDynamicGating_tmp));
	inst = (sTracking_objectUnit *)handle;
	myPointNum = 0;
	myUniquePointNum = 0;
	//memset((uint8_t *)D, 0, sizeof(float)*MSIZE_SPH*MSIZE_SPH);
	//memset((uint8_t *)Rm, 0, sizeof(float)*MSIZE_SPH*MSIZE_SPH);
	//memset((uint8_t *)Rc, 0, sizeof(float)*MSIZE_SPH*MSIZE_SPH);

	if(inst->objManagement.isNewTracker==1){
		return;

	}

	tracking_vectorInit(MSIZE_SPH, 0.f, u_sum.array);
	tracking_vectorInit(MSIZE_SPH, 0.f, uvar_sum.array);
	tracking_vectorInit(MSIZE_SPH, -FLT_MAX, u_max);
	tracking_vectorInit(MSIZE_SPH, FLT_MAX, u_min);
#ifdef HUHANG
	tracking_vectorInit(MSIZE_SPH, -FLT_MAX, cart_max3);
	tracking_vectorInit(MSIZE_SPH, FLT_MAX, cart_min3);
	tracking_vectorInit(MSIZE_SPH, 0.f, u_cart_centroidi);
	tracking_vectorInit(MSIZE_SPH, 0.f, u_cart_centroid);

#endif // HUHANG
	tracking_vectorInit(SSIZE_XY, -FLT_MAX, cart_max);
	tracking_vectorInit(SSIZE_XY, FLT_MAX, cart_min);

	tracking_vectorInit(SSIZE_XY, 0.f, cart_center);
	tracking_vectorInit(MSIZE_SPH, 0.f, u_centroid.array);
	tracking_vectorInit(MSIZE_SPH, 0.f, rm_diag.array);
	if (inst->eDynamicProperty == TRACKING_DYNAMIC_MOVING && inst->curTarget.uNumOfDetections > 1)
	{
		for (n = 0; n < num; n++)
		{
			if ((inst->assoProperty.bitIndicatorsOfDetAsso[n >> 3] & (0x1 << (n & 0x7))) )
			{
				if (point[n].flag_moving == 1U && point[n].disambigVelReliable>=0.9f) 
				{
					movingAssoFlag = 1U;	//TODO: delet the bitIndicatorsOfDetAsso
					break;
				}
			}
		}
	}
	
	// Calculate sum
	for(n=0; n<num; n++)
	{
		if ((!(inst->assoProperty.bitIndicatorsOfDetAsso[n >> 3] & (0x1 << (n & 0x7))) \
			|| (inst->curTarget.uNumOfDetections == 0)))
		{
			continue;
		}
		else if (movingAssoFlag == 1U && point[n].flag_moving == 0U)
		{
			if (fabsf(point[n].vector.azimuthRad) > AZIMUTH_BOUNDARY)
				assoOverCnt++;
			continue;
		}
		else
		{
			if (fabsf(point[n].vector.azimuthRad) > AZIMUTH_BOUNDARY)
				assoOverCnt++;
		}
		// 

		tracking_vectorSub(MSIZE_SPH, point[n].array, inst->kalman_state.H_s.array, u_tilda.array);
		if(point[n].flag_moving==1U && point[n].disambigVelReliable!=1.0f)
		{
			/* Radial velocity estimation is not yet known, unroll based on velocity measured at allocation time */
			if (0 && inst->objKinematic.velocityHandling < VELOCITY_LOCKED) {
#ifdef ENABLE_DISAMBIGUITYVELOCITY
				rvOut = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->objManagement.allocationVelocity, point[n].vector.doppler);
#else
				rvOut = point[n].vector.doppler;
#endif
				doppler_rvOut[n] = rvOut;
				u_tilda.vector.doppler = rvOut - inst->objManagement.allocationVelocity;
				doppler_tmp_vec[n] = inst->objManagement.allocationVelocity;
			}
			else {
				/* Radial velocity estimation is known */
				// Consider the long object when crossing
				myMath_sincosd(point[n].vector.azimuthRad*RAD2DEG, &sinAngle, &cosAngle);
				doppler_tmp = inst->kalman_state.S_hat[2]*cosAngle + inst->kalman_state.S_hat[3]*sinAngle;
#ifdef ENABLE_DISAMBIGUITYVELOCITY
				rvOut = tracking_unrollRadialVelocity(rvMax, doppler_tmp, point[n].vector.doppler);
#else
				rvOut = point[n].vector.doppler;
#endif
				//point[n].vector.doppler = rvOut;
				//point[n].disambigVelReliable = 1.0f;
				//rvOut = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->kalman_state.H_s.vector.doppler, point[n].vector.doppler);
				u_tilda.vector.doppler = rvOut - doppler_tmp;
				doppler_rvOut[n] = rvOut;
				doppler_tmp_vec[n] = doppler_tmp;
			}
		}
		else
		{
			doppler_rvOut[n] = point[n].vector.doppler;
			doppler_tmp_vec[n] = inst->kalman_state.H_s.vector.doppler;
		}
#ifdef HUHANG
		if ((fabsf(platformInfo->egoLineVel_lat) > 8.3f)\
			&& (fabsf(inst->kalman_state.S_hat[1]) < 15.f)\
			&&(fabsf(inst->kalman_state.S_hat[0]) <4.65 ))
		{
			if ((point[n].x>1.5)&&(point[n].x < minLong))
			{
				minLong = point[n].x;
			}
			if (fabsf(point[n].y) < minLat)
			{
				minLat = point[n].y;
			}
		}
		else
		{
			if (fabsf(inst->kalman_state.S_hat[1]) < 5.f \
				&& point[n].x < minLong ){
				minLong = point[n].x;
			}
			if (fabsf(point[n].y) < minLat)
			{
				minLat = point[n].y;
			}
		}
#else
		if (fabsf(inst->kalman_state.S_hat[1]) < 5.f \
			&& point[n].x < minLong) {
			minLong = point[n].x;
		}
		
#endif // HUHANG

		dx = point[n].x - inst->kalman_state.S_hat[0];
		dy = point[n].y - inst->kalman_state.S_hat[1];
		//for (k = 0; k < MSIZE_SPH; k++)
		//{
		//	if (fabsf(u_tilda.array[k]) > inst->assoProperty.assocDynamic_H_limits.array[k])
		//	{
		//		isWithinLimits = false;
		//		break;
		//	}
		//}
		isWithinLimits = true;
#ifdef HUHANG
		if (fabsf(platformInfo->egoLineVel_lat) > 8.3f)
		{
			if ((fabsf(dx) > inst->assoProperty.assocDynamicGating.depth)\
				||(fabsf(dy) > inst->assoProperty.assocDynamicGating.width))
			{
				isWithinLimits = false;
			}
		}
		else
		{
			for (k = 0; k < MSIZE_SPH; k++)
			{
				if (fabsf(u_tilda.array[k]) > inst->assoProperty.assocDynamic_H_limits.array[k])
				{
					isWithinLimits = false;
					break;
				}
			}
		}
#else
		for (k = 0; k < MSIZE_SPH; k++)
		{
			if (fabsf(u_tilda.array[k]) > inst->assoProperty.assocDynamic_H_limits.array[k])
			{
				isWithinLimits = false;
				break;
			}
		}
#endif // HUHANG

		
	
//#ifdef DEBUG_LOG
//		//my_printf("inst[%d] u_tilda[%d]: %.3f, %.3f, %.3f\n", inst->uid, n, \
//			u_tilda.array[0], u_tilda.array[1], u_tilda.array[2]);
//#endif
		idxOfDetsAssoc[idxOfDetsAssoc_num] = n;
		idxOfDetsAssoc_num++;
	}
#ifdef DEBUG_LOG
	//my_printf("inst[%d] association point num: %d\n", inst->uid, idxOfDetsAssoc_num);
//	//my_printf("inst[%d] gating_limit: %.3f, %.3f, %.3f\n", inst->uid,\
//		inst->assoProperty.H_limits.array[0], \
//		inst->assoProperty.H_limits.array[1], \
//		inst->assoProperty.H_limits.array[2]);
//	//my_printf("inst[%d] dynamic gating_limit: %.3f, %.3f, %.3f\n", inst->uid,\
//		inst->assoProperty.assocDynamic_H_limits.array[0], \
//		inst->assoProperty.assocDynamic_H_limits.array[1], \
//		inst->assoProperty.assocDynamic_H_limits.array[2]);
//	//my_printf("inst[%d] u_tilda_sum: %.3f, %.3f, %.3f\n", inst->uid,\
//		inst->assoProperty.u_tilda_sum.array[0], \
//		inst->assoProperty.u_tilda_sum.array[1], \
//		inst->assoProperty.u_tilda_sum.array[2]);
//	//my_printf("inst[%d] H_s: %.3f, %.3f, %.3f\n", inst->uid,\
//		inst->kalman_state.H_s.array[0], \
//		inst->kalman_state.H_s.array[1], \
//		inst->kalman_state.H_s.array[2]);
#endif
	// Calculate weight scale
	//for(n=0; n<num; n++)
	nearCount1to2 = 0;
	for(k = 0; k < idxOfDetsAssoc_num; k++) 
	{
		n = idxOfDetsAssoc[k];
		if(point[n].flag_moving!=0U){
			flag_moving_cnt++;
		}
		if (idxOfDetsAssoc_num>=8)
		{
			if ((point[n].x >= 1.5) && (point[n].x <= 3))
			{
				nearCount1to2++;
			}
		}
		//if(    (!(inst->assoProperty.bitIndicatorsOfDetAsso[n>>3] & (0x1 << (n & 0x7))) \
		//	|| (inst->curTarget.uNumOfDetections							==       0)) )
		//{
		//	continue;
		//}
		myMath_checkZero(&inst->assoProperty.u_tilda_sum.vector.range);
		myMath_checkZero(&inst->assoProperty.u_tilda_sum.vector.doppler);
		myMath_checkZero(&inst->assoProperty.u_tilda_sum.vector.azimuthRad);
		myMath_checkZero(&inst->assoProperty.u_tilda_sum.mag);
		myMath_checkZero(&inst->assoProperty.u_tilda_sum.snr);
		myMath_checkZero(&inst->assoProperty.u_tilda_sum.rcs);

//#ifdef DEBUG_LOG
//		//my_printf("inst[%d] currentTarget point[%d]: %.3f, %.3f, %.3f\n", \
//			inst->uid, n, point[n].vector.range, point[n].vector.azimuthRad*RAD2DEG, \
//			point[n].vector.doppler);
//#endif
		tracking_vectorSub(MSIZE_SPH, point[n].array, inst->kalman_state.H_s.array, u_tilda.array);
		if(point[n].flag_moving==1U && point[n].disambigVelReliable!=1.0f)
		{
			u_tilda.vector.doppler = doppler_rvOut[n] - doppler_tmp_vec[n];
		}
//#ifdef DEBUG_LOG
//		//my_printf("inst[%d] u_tilda[%d]: %.3f, %.3f, %.3f\n", inst->uid, n, \
//			u_tilda.array[0], u_tilda.array[1], u_tilda.array[2]);
//#endif
		/* Any point outside the limits is outside the gate */
		isWithinLimits = true;
		//for (m = 0; m < MSIZE_SPH; m++)
		//{
		//	if (fabsf(u_tilda.array[m]) > inst->assoProperty.assocDynamic_H_limits.array[m])
		//	{
		//		isWithinLimits = false;
		//		break;
		//	}
		//}
		if(0==isInDyncGating[k]){
			isWithinLimits = false;
		}
#ifdef DEBUG_LOG
		//my_printf("ID:%d, tgtId:%d ,clsId:%d", inst->uid, n,point[n].clusterId);
#endif // DEBUG_LOG
		if(!isWithinLimits)
		{
			//if(    (point[n].flag_moving==1U && inst->eDynamicProperty==TRACKING_DYNAMIC_STATIC) \
			//	|| (point[n].flag_moving==0U && inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC) )
			//if(point[n].flag_moving==0U && inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC)
			//{
			//	u_tilda.vector.range = 10.f * u_tilda.vector.range;
			//	u_tilda.vector.doppler = 10.f * u_tilda.vector.doppler;
			//	u_tilda.vector.azimuthRad = 10.f * u_tilda.vector.azimuthRad;
			//}
			scale_tmp = 0.2f;
			//if(point[n].flag_moving==0U && inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC)
			if(    (point[n].flag_moving==1U && inst->eDynamicProperty==TRACKING_DYNAMIC_STATIC) \
				|| (point[n].flag_moving==0U && inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC) )
			{
				scale_tmp = 0.1f;
			}
			else
			{
				flagValid = 1;
			}
			weightOfDetsAssoc_range[k] = scale_tmp*(inst->assoProperty.u_tilda_sum.vector.range-fabsf(u_tilda.vector.range))/inst->assoProperty.u_tilda_sum.vector.range;
			weightOfDetsAssoc_azimuthRad[k] = scale_tmp*(inst->assoProperty.u_tilda_sum.vector.azimuthRad-fabsf(u_tilda.vector.azimuthRad))/inst->assoProperty.u_tilda_sum.vector.azimuthRad;
			weightOfDetsAssoc_doppler[k] = scale_tmp*(inst->assoProperty.u_tilda_sum.vector.doppler-fabsf(u_tilda.vector.doppler))/inst->assoProperty.u_tilda_sum.vector.doppler;

			//idxOfDetsAssoc[idxOfDetsAssoc_num] = n;
			//idxOfDetsAssoc_num++;
		}
		else
		{
			scale_tmp = 0.8f;
			//if(point[n].flag_moving==0U && inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC)
			if(    (point[n].flag_moving==1U && inst->eDynamicProperty==TRACKING_DYNAMIC_STATIC) \
				|| (point[n].flag_moving==0U && inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC) )
			{
				scale_tmp = 0.5f;
			}
			else
			{
				flagValid = 1;
			}
			flagDynGatingValid = 1;
			if(1 || pInd[n] == inst->uid) {
				weightOfDetsAssoc_range[k] = scale_tmp;
				weightOfDetsAssoc_azimuthRad[k] = scale_tmp;
				weightOfDetsAssoc_doppler[k] = scale_tmp;
				weightOfDetsAssoc_mag[k] = scale_tmp;
				weightOfDetsAssoc_snr[k] = scale_tmp;
				weightOfDetsAssoc_rcs[k] = scale_tmp;

				//idxOfDetsAssoc[idxOfDetsAssoc_num] = n;
				//idxOfDetsAssoc_num++;
			}
		}
//#ifdef DEBUG_LOG
//		//my_printf("00 inst[%d] point[%d] weightOfDetsAssoc_range: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc_range[k]);
//		//my_printf("00 inst[%d] point[%d] weightOfDetsAssoc_doppler: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc_doppler[k]);
//		//my_printf("00 inst[%d] point[%d] weightOfDetsAssoc_azimuthRad: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc_azimuthRad[k]);
//#endif
	}
	// normalization
	if(idxOfDetsAssoc_num<=1)
	{
		weightOfDetsAssoc_range[0] = 1.f;
		weightOfDetsAssoc_azimuthRad[0] = 1.f;
		weightOfDetsAssoc_doppler[0] = 1.f;
		weightOfDetsAssoc_mag[0] = 1.f;
		weightOfDetsAssoc_snr[0] = 1.f;
		weightOfDetsAssoc_rcs[0] = 1.f;
		//if(idxOfDetsAssoc_num==1 && flagValid==0)
		//{
		//	idxOfDetsAssoc_num = 0;
		//	myPointNum = 0;
		//}
	}
	else
	{
		myMath_nomalization(weightOfDetsAssoc_range, idxOfDetsAssoc_num);
		myMath_nomalization(weightOfDetsAssoc_azimuthRad, idxOfDetsAssoc_num);
		myMath_nomalization(weightOfDetsAssoc_doppler, idxOfDetsAssoc_num);
		myMath_nomalization(weightOfDetsAssoc_mag, idxOfDetsAssoc_num);
		myMath_nomalization(weightOfDetsAssoc_snr, idxOfDetsAssoc_num);
		myMath_nomalization(weightOfDetsAssoc_rcs, idxOfDetsAssoc_num);
	}

	// 1. Set prediction state with saved state
	// 2. The compesanted state is just used for association
	dimLen = inst->tracking_params->stateVectorDimLength;
	/* Convert from cartesian to spherical */
    tracking_cartesian2spherical(inst->tracking_params->stateVectorType, inst->kalman_state.S_hat, inst->kalman_state.H_s.array);

	//printf("%llu: uid[%d]: prediction inst->S_apriori_hat: (%.2f, %.2f)\n", \
	//	inst->heartBeatCount, inst->uid, inst->S_apriori_hat[0], inst->S_apriori_hat[1]);
	/* Compute means of associated measurement points */
	/* Accumulate measurements */
	sumWeightOfDetsAssoc = 0.0f;
	for(k = 0; k < idxOfDetsAssoc_num; k++) 
	{
		n = idxOfDetsAssoc[k];
		weightScale_minLong = 0.0f; 
		weightScale_moving = 0.0f;
		weightScale_minLat=0.0f;
#ifdef HUHANG
		if (fabsf(platformInfo->egoLineVel_lat) > 8.3f)
		{
			if ((idxOfDetsAssoc_num >= 8)&&(nearCount1to2>=4))
			{
				if (point[n].x - minLong < 0.5f) {
					//weightScale_minLong = 1.0f;
					weightScale_minLong = 5.0f;
				}
				//if ((fabsf(point[n].x) >1.5f)&&(fabsf(point[n].x) <= 3.0f))
				//{
				//	weightScale_minLong = 3.0f;
				//}
				else
				{
					weightScale_minLong = 0.5f;
				}
			}
			else
			{
				if ((fabsf(inst->kalman_state.S_hat[0]) > 0.5f) && (fabsf(inst->kalman_state.S_hat[0]) < 4.65f))
				{
					if (point[n].x - minLong < 0.5f) {
						//weightScale_minLong = 1.0f;
						weightScale_minLong = 3.0f;
					}
					else if (point[n].x - minLong < 1.f) {
						weightScale_minLong = 1.0f;
					}
					else {
						weightScale_minLong = 0.0f;
					}
					/*if (fabsf(point[n].x) < 0.5f)
					{
						weightScale_minLong = 0.5f;
					}
					else if (fabsf(point[n].x) < 1.5f)
					{
						weightScale_minLong = 1.0f;
					}
					else if (fabsf(point[n].x) < 3.65f)
					{
						weightScale_minLong = 2.0f;
					}
					else if (fabsf(point[n].x) < 4.65f)
					{
						weightScale_minLong = 1.0f;
					}
					else
					{
						weightScale_minLong = 0.5f;
					}*/
				}
				else
				{
					weightScale_minLong = 1.0f;
				}
			}
			if ((fabsf(inst->kalman_state.S_hat[0]) > 0.5f) && (fabsf(inst->kalman_state.S_hat[0]) < 4.65f)\
				&&((idxOfDetsAssoc_num>=3)||(inst->objGeometryProperty.flag_truck==1))\
				/*&&(fabsf(inst->kalman_state.S_hat[1]) > 5.0f)*/)
			{
				if (point[n].y - minLat < 1.0f) {
					//weightScale_minLong = 1.0f;
					weightScale_minLat = 3.0f;
				}
				else if (point[n].y - minLat < 3.f) {
					weightScale_minLat = 1.0f;
				}
				else
				{
					weightScale_minLat = 0.0f;
				}
			}
		}
		else
		{
			if (point[n].x - minLong < 0.5f) {
				//weightScale_minLong = 1.0f;
				weightScale_minLong = 5.0f;
			}
			else if (point[n].x - minLong < 1.f) {
				weightScale_minLong = 0.5f;
			}
			else if (point[n].x - minLong < 2.f) {
				weightScale_minLong = 0.1f;
			}
			else if (point[n].x - minLong < 3.f) {
				weightScale_minLong = 0.01f;	
			}
			else {
				weightScale_minLong = 0.0f;
			}
		}
#else
		if (point[n].x - minLong < 0.5f) {
			//weightScale_minLong = 1.0f;
			weightScale_minLong = 5.0f;
	}
		else if (point[n].x - minLong < 1.f) {
			weightScale_minLong = 0.5f;
		}
		else if (point[n].x - minLong < 2.f) {
			weightScale_minLong = 0.1f;
		}
		else if (point[n].x - minLong < 3.f) {
			weightScale_minLong = 0.01f;	
		}
		else {
			weightScale_minLong = 0.0f;
		}
#endif // HUHANG
#ifdef HUHANG
		if (point[n].flag_moving == 1 \
			&& flag_moving_cnt > 0 \
			&& inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC)
#else
		if (point[n].flag_moving == 1 \
			&& flag_moving_cnt > 0 \
			&& inst->eDynamicProperty != TRACKING_DYNAMIC_STATIC  )
#endif // !HUHANG
		{
			weightScale_moving = 1.0f;
			//weightScale_moving = 0.0f;
		}
		else{
			weightScale_moving = 0.0f;
		}
		weightOfDetsAssoc[k] = 0.3f * weightOfDetsAssoc_range[k] \
			+ 0.3f * weightOfDetsAssoc_doppler[k] \
			+ 0.2f * weightOfDetsAssoc_azimuthRad[k] \
			+ 0.2 * weightOfDetsAssoc_mag[k] \
			+ weightScale_minLong \
			+ weightScale_moving\
			+ weightScale_minLat;
		//weightOfDetsAssoc[k] *= weightScale_minLong;
		sumWeightOfDetsAssoc += weightOfDetsAssoc[k];
		//weightOfDetsAssoc[k] = 0.3f*weightOfDetsAssoc_range[k] \
		//	+ 0.2f*weightOfDetsAssoc_doppler[k] \
		//	+ 0.3f*weightOfDetsAssoc_azimuthRad[k] \
		//	+ 0.08f*weightOfDetsAssoc_rcs[k] \
		//	+ 0.08f*weightOfDetsAssoc_snr[k] \
		//	+ 0.04*weightOfDetsAssoc_mag[k];
		//if(    fabsf(inst->kalman_state.S_hat[0])<10.f \
		//	&& fabsf(inst->kalman_state.S_hat[1])<5.f ){
		//	weightOfDetsAssoc[k] = 0.1f*weightOfDetsAssoc_range[k] \
		//		+ 0.1f*weightOfDetsAssoc_doppler[k] \
		//		+ 0.1f*weightOfDetsAssoc_azimuthRad[k] \
		//		+ 0.7*weightOfDetsAssoc_mag[k];
		//}
		//else{
		//	weightOfDetsAssoc[k] = 0.3f*weightOfDetsAssoc_range[k] \
		//		+ 0.3f*weightOfDetsAssoc_doppler[k] \
		//		+ 0.2f*weightOfDetsAssoc_azimuthRad[k] \
		//		+ 0.2*weightOfDetsAssoc_mag[k];
		//}
//		weightOfDetsAssoc[k] = 0.4f*weightOfDetsAssoc_range[k] \
//			+ 0.3f*weightOfDetsAssoc_doppler[k] \
//			+ 0.3f*weightOfDetsAssoc_azimuthRad[k];
		
//#ifdef DEBUG_LOG
//		//my_printf("inst[%d] point[%d] weightOfDetsAssoc: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc[k]);
//		//my_printf("inst[%d] point[%d] weightOfDetsAssoc_range: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc_range[k]);
//		//my_printf("inst[%d] point[%d] weightOfDetsAssoc_doppler: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc_doppler[k]);
//		//my_printf("inst[%d] point[%d] weightOfDetsAssoc_azimuthRad: %.3f\n", inst->uid, n, \
//			weightOfDetsAssoc_azimuthRad[k]);
//#endif
		memcpy(u.array, point[n].array, sizeof(u.array));
		u.vector.doppler = doppler_rvOut[n];
		/* Compute maximums and minimums for each measurement */
		//if(isUnique[n>>3] & (0x1 << (n & 0x0007))) {
		if(1) 
		{
			for(m = 0; m < MSIZE_SPH; m++)
			{
				if(u.array[m] > u_max[m])
					u_max[m] = u.array[m];
				if(u.array[m] < u_min[m])
					u_min[m] = u.array[m];
			}
			myUniquePointNum++;

			tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u.array, u_cart);
			for (m = 0; m < SSIZE_XY; m++) {
				if (u_cart[m] > cart_max[m])
					cart_max[m] = u_cart[m];
				if (u_cart[m] < cart_min[m])
					cart_min[m] = u_cart[m];
			}

#ifdef HUHANG
			if (fabsf(platformInfo->egoLineVel_lat) > 8.3f)
			{
				u_cart3[0] = u_cart[0];
				u_cart3[1] = u_cart[1];
				u_cart3[2] = u.vector.doppler;
				memcpy(u_cart_centroidi, u_cart3, 3 * sizeof(u_cart3[0]));

				u_cart3[0] -= inst->kalman_state.S_hat[0];
				u_cart3[1] -= inst->kalman_state.S_hat[1];
				u_cart3[2] -= inst->kalman_state.H_s.vector.doppler;

				for (m = 0; m < MSIZE_SPH; m++)
				{
					if (u_cart3[m] > cart_max3[m])
						cart_max3[m] = u_cart3[m];
					if (u_cart3[m] < cart_min3[m])
						cart_min3[m] = u_cart3[m];
				}
			}
#endif
		}
		if(var != NULL) {
			uvar.vector = var[n];
			tracking_vectorAdd(MSIZE_SPH, uvar.array, uvar_sum.array, uvar_sum.array); 
		}

		//if(myPointNum == 0)
		//	rvPilot = u.vector.doppler;
		//else
		//	u.vector.doppler = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, rvPilot, u.vector.doppler); /* Unroll using rvPilot */

		if (point[n].snr != 0) {
			centroidSnr += point[n].snr;
			//tracking_vectorScalarMulAcc(MSIZE_SPH, u.array,  point[n].snr, u_sum.array);
		}
		else {
			centroidSnr += 1.0;
			//tracking_vectorScalarMulAcc(MSIZE_SPH, u.array,  1.0, u_sum.array);
		}
		tracking_vectorScalarMulAcc(MSIZE_SPH, u.array, weightOfDetsAssoc[k], \
			u_centroid.array);
#ifdef HUHANG
		if (fabsf(platformInfo->egoLineVel_lat) > 8.3f)
		{
			tracking_vectorScalarMulAcc(MSIZE_SPH, u_cart_centroidi, weightOfDetsAssoc[k], \
				u_cart_centroid);
		}
#endif // HUHANG

		//fSNR += point[n].snr*weightOfDetsAssoc[k];
		fSNR = (point[n].snr>fSNR)? point[n].snr : fSNR;
		//fMag = (point[n].mag>fMag)? point[n].mag : fMag;
		if (fMag <= point[n].mag)
		{
			fMag = point[n].mag;
			fMagCoorX = point[n].x;
			fMagCoorY = point[n].y;
		}
		fAngel = (fabsf(point[n].vector.azimuthRad) > fabsf(fAngel)) ? point[n].vector.azimuthRad : fAngel;
		fMag_min = (point[n].mag < fMag_min) ? point[n].mag : fMag_min;
		fMag_mean += point[n].mag;
		mag_array[k] = point[n].mag;
		//fMag += point[n].mag*weightOfDetsAssoc[k];
		//fRCS += point[n].rcs*weightOfDetsAssoc[k];
		fRCS = (point[n].rcs>fRCS)? point[n].rcs : fRCS;
		myPointNum++;
//#ifdef DEBUG_LOG
//		//my_printf("inst[%d], u_centroid: %.3f, %.3f, %.3f\n", inst->uid, \
//			u_centroid.vector.range, u_centroid.vector.azimuthRad*RAD2DEG, \
//			u_centroid.vector.doppler);
//		//my_printf("inst[%d], u: %.3f, %.3f, %.3f\n", inst->uid, \
//			u.vector.range, u.vector.azimuthRad*RAD2DEG, \
//			u.vector.doppler);
//#endif
	}
	if (myPointNum)
	{
		if (fabsf(sumWeightOfDetsAssoc) >= 0.5f)
		{
			tracking_vectorScalarMul(MSIZE_SPH, u_centroid.array, \
				1.0f / sumWeightOfDetsAssoc, u_centroid.array);

#ifdef HUHANG
			tracking_vectorScalarMul(MSIZE_SPH, u_cart_centroid, \
				1.0f / sumWeightOfDetsAssoc, u_cart_centroid);
#endif // HUHANG

		}

		fMag_mean = fMag_mean / ((float)(idxOfDetsAssoc_num));
		fMag_spread = fMag - fMag_min;
		for (k = 0; k < idxOfDetsAssoc_num; k++)
		{
			fMag_stdVar += (mag_array[k] - fMag_mean) * (mag_array[k] - fMag_mean);
		}
		fMag_stdVar = sqrtf(fMag_stdVar / ((float)(idxOfDetsAssoc_num)));
		myMath_insertSort_float(mag_array, idxOfDetsAssoc_num);
		fMag_middle = mag_array[(int)(((float)(idxOfDetsAssoc_num)) / 2.f)];
	}
//#ifdef DEBUG_LOG
//	//my_printf("inst[%d], num: %d, mag_array[0]: %.3f, end: %.3f\n", inst->uid, \
//		idxOfDetsAssoc_num, mag_array[0], mag_array[idxOfDetsAssoc_num-1]);
//#endif
	
	//if(flagDynGatingValid==0 && idxOfDetsAssoc_num>0)
	//{
	//	scale_tmp = 0.8f;
	//	u_centroid.vector.range = scale_tmp*u_centroid.vector.range + (1.f-scale_tmp)*inst->kalman_state.H_s.vector.range;
	//	u_centroid.vector.azimuthRad = scale_tmp*u_centroid.vector.azimuthRad + (1.f-scale_tmp)*inst->kalman_state.H_s.vector.azimuthRad;
	//	u_centroid.vector.doppler = scale_tmp*u_centroid.vector.doppler + (1.f-scale_tmp)*inst->kalman_state.H_s.vector.doppler;
	//	fSNR = scale_tmp*fSNR + (1.f-scale_tmp)*inst->curTarget.fSNR;
	//	fMag = scale_tmp*fMag + (1.f-scale_tmp)*inst->curTarget.fMag;
	//	fRCS = scale_tmp*fRCS + (1.f-scale_tmp)*inst->curTarget.fRCS;
	//}

	AssoW = 0.0f;
	AssoD = 0.0f;
	AssoVel = 0.0f;

	if(myPointNum)
	{
		// Update dynamic gating
#ifdef HUHANG
		if (fabsf(platformInfo->egoLineVel_lat) > 8.3f)
		{
			Width = (fabsf(cart_max3[1]) > fabsf(cart_min3[1])) ? fabsf(cart_max3[1]) : fabsf(cart_min3[1]);
			Depth = (fabsf(cart_max3[0]) > fabsf(cart_min3[0])) ? fabsf(cart_max3[0]) : fabsf(cart_min3[0]);
			Vel = (fabsf(cart_max3[2]) > fabsf(cart_min3[2])) ? fabsf(cart_max3[2]) : fabsf(cart_min3[2]);
			if (fabsf(inst->kalman_state.S_hat[1]) < 8)
			{
				if (Width < 1)
				{
					AssoW = 2.0f;
				}
				else if (Width < 2.0f)
				{
					AssoW = 1.0f;
				}
				else if (Width < 3.0f)
				{
					AssoW = 0.75f;
				}
				else
				{
					AssoW = 0.5f;
				}
				
				if (Depth < 0.5f)
				{
					AssoD = 1.0f;
				}
				else if (Width < 1.0f)
				{
					AssoD = 0.75f;
				}
				else if (Width < 1.5f)
				{
					AssoD = 0.5f;
				}
				else
				{
					AssoD = 0.25f;
				}
				AssoVel = 2.0f;
			}
			else if (fabsf(inst->kalman_state.S_hat[1]) < 16)
			{
				if (Width < 1)
				{
					AssoW = 2.0f;
				}
				else if (Width < 2.0f)
				{
					AssoW = 1.25f;
				}
				else if (Width < 2.5f)
				{
					AssoW = 0.75f;
				}
				else
				{
					AssoW = 0.5f;
				}

				if (Depth < 0.75f)
				{
					AssoD = 1.0f;
				}
				else if (Width < 1.25f)
				{
					AssoD = 0.75f;
				}
				else if (Width < 1.5f)
				{
					AssoD = 0.5f;
				}
				else
				{
					AssoD = 0.25f;
				}
				AssoVel = 1.0f;
			}
			else
			{
				if (Width < 1)
				{
					AssoW = 1.5f;
				}
				else if (Width < 1.5f)
				{
					AssoW = 1.0f;
				}
				else 
				{
					AssoW = 0.5f;
				}

				if (Depth < 0.5f)
				{
					AssoD = 1.0f;
				}
				else if (Width < 1.0f)
				{
					AssoD = 0.75f;
				}
				else 
				{
					AssoD = 0.5f;
				}
			}
			Width += AssoW;
			Depth += AssoD;
			Vel += AssoVel;

			if (inst->objGeometryProperty.hasStbTrkNotAssoAllPointOv30Spd == 5)
			{
				Width += 5;
			}
			else if (inst->objGeometryProperty.hasStbTrkNotAssoAllPointOv30Spd == 3)
			{
				Width += 3;
			}
			else if (inst->objGeometryProperty.hasStbTrkNotAssoAllPointOv30Spd == 1)
			{
				Width += 1;
			}
			else
			{
				Width += 0;
			}

			if (inst->objGeometryProperty.flag_truck == 1)
			{
				Width += 2;
			}
		}
		else
		{
			tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u_centroid.array, curTarget_cart);
			Width = fabsf(curTarget_cart[1] - inst->kalman_state.S_hat[1]);
			Depth = fabsf(curTarget_cart[0] - inst->kalman_state.S_hat[0]);
			Vel = fabsf(u_centroid.vector.doppler - inst->kalman_state.H_s.vector.doppler);

			AssoW = 2.0f;
			AssoD = 2.0f;
			AssoVel = 2.0f;
			Width *= AssoW;
			Depth *= AssoD;
			Vel *= AssoVel;
		}
		assocDynamicGating_tmp.depth = Depth;
		assocDynamicGating_tmp.depth = (assocDynamicGating_tmp.depth < inst->tracking_params->advParams.gatingParams.limits.depth * 0.7f) ? (assocDynamicGating_tmp.depth) : (inst->tracking_params->advParams.gatingParams.limits.depth * 0.7f);
		assocDynamicGating_tmp.width = Width;
		assocDynamicGating_tmp.width = (assocDynamicGating_tmp.width < inst->tracking_params->advParams.gatingParams.limits.width * 0.7f) ? (assocDynamicGating_tmp.width) : (inst->tracking_params->advParams.gatingParams.limits.width * 0.7f);
		assocDynamicGating_tmp.vel = Vel;
		assocDynamicGating_tmp.vel = (assocDynamicGating_tmp.vel < inst->tracking_params->advParams.gatingParams.limits.vel * 0.7f) ? (assocDynamicGating_tmp.vel) : (inst->tracking_params->advParams.gatingParams.limits.vel * 0.7f);
		assocDynamicGating_tmp.height = inst->assoProperty.assocDynamicGating.height;

#else
		tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u_centroid.array, curTarget_cart);
		Width = fabsf(curTarget_cart[1] - inst->kalman_state.S_hat[1]);
		Depth = fabsf(curTarget_cart[0] - inst->kalman_state.S_hat[0]);
		Vel = fabsf(u_centroid.vector.doppler - inst->kalman_state.H_s.vector.doppler);
		AssoW = 2.0f;
		AssoD = 2.0f;
		AssoVel = 2.0f;
		assocDynamicGating_tmp.depth = Depth * AssoD;
		assocDynamicGating_tmp.depth = (assocDynamicGating_tmp.depth < inst->tracking_params->advParams.gatingParams.limits.depth * 0.7f) ? (assocDynamicGating_tmp.depth) : (inst->tracking_params->advParams.gatingParams.limits.depth * 0.7f);
		assocDynamicGating_tmp.width = Width * AssoW;
		assocDynamicGating_tmp.width = (assocDynamicGating_tmp.width < inst->tracking_params->advParams.gatingParams.limits.width * 0.7f) ? (assocDynamicGating_tmp.width) : (inst->tracking_params->advParams.gatingParams.limits.width * 0.7f);
		assocDynamicGating_tmp.vel = Vel * AssoVel;
		assocDynamicGating_tmp.vel = (assocDynamicGating_tmp.vel < inst->tracking_params->advParams.gatingParams.limits.vel * 0.7f) ? (assocDynamicGating_tmp.vel) : (inst->tracking_params->advParams.gatingParams.limits.vel * 0.7f);
		assocDynamicGating_tmp.height = inst->assoProperty.assocDynamicGating.height;

#endif // HUHANG

		
		//if (fabsf(platformInfo->egoLineVel_lat) < 8.3f)
		//{
		//	AssoW = 2.0f;
		//	AssoD = 2.0f;
		//	AssoVel = 2.0f;
		//}
		//else
		//{
		//	if (fabsf(inst->kalman_state.S_hat[1]) < 8)
		//	{
		//		AssoW = 6.0f;
		//		AssoD = 4.0f;
		//	}
		//	else if (fabsf(inst->kalman_state.S_hat[1]) < 16)
		//	{
		//		AssoW = 6.0f;
		//		AssoD = 5.0f;
		//	}
		//	else
		//	{
		//		AssoW = 3.0f;
		//		AssoD = 3.0f;
		//	}
		//	if (Width < 1.0f)
		//	{
		//		Width = 1.0f;
		//	}
		//	if (Depth < 1.0f)
		//	{
		//		Depth = 1.0f;
		//	}
		//	AssoVel = 1.0f;
		//}
		//assocDynamicGating_tmp.depth = Depth * AssoD;
		//assocDynamicGating_tmp.depth = (assocDynamicGating_tmp.depth < inst->tracking_params->advParams.gatingParams.limits.depth * 0.7f) ? (assocDynamicGating_tmp.depth) : (inst->tracking_params->advParams.gatingParams.limits.depth * 0.7f);
		//assocDynamicGating_tmp.width = Width * AssoW;
		//assocDynamicGating_tmp.width = (assocDynamicGating_tmp.width < inst->tracking_params->advParams.gatingParams.limits.width * 0.7f) ? (assocDynamicGating_tmp.width) : (inst->tracking_params->advParams.gatingParams.limits.width * 0.7f);
		//assocDynamicGating_tmp.vel = Vel * AssoVel;
		//assocDynamicGating_tmp.vel = (assocDynamicGating_tmp.vel < inst->tracking_params->advParams.gatingParams.limits.vel * 0.7f) ? (assocDynamicGating_tmp.vel) : (inst->tracking_params->advParams.gatingParams.limits.vel * 0.7f);
		//assocDynamicGating_tmp.height = inst->assoProperty.assocDynamicGating.height;
	
		myMath_lowPassFilter(&inst->assoProperty.assocDynamicGating.depth, assocDynamicGating_tmp.depth, 0.3f);
		myMath_lowPassFilter(&inst->assoProperty.assocDynamicGating.width, assocDynamicGating_tmp.width, 0.3f);
		myMath_lowPassFilter(&inst->assoProperty.assocDynamicGating.vel, assocDynamicGating_tmp.vel, 0.3f);
		// limit the min dynamic gating
		if (inst->assoProperty.assocDynamicGating.depth < 1.0f) {
			inst->assoProperty.assocDynamicGating.depth = 1.0f;
		}
		if (inst->assoProperty.assocDynamicGating.width < 1.0f) {
			inst->assoProperty.assocDynamicGating.width = 1.0f;
		}
		if (inst->assoProperty.assocDynamicGating.vel < 1.0f) {
			inst->assoProperty.assocDynamicGating.vel = 1.0f;
		}
	}
	if (myPointNum)
	{
		if (flag_moving_cnt >= (int)(((float)(myPointNum)) * 0.5f + 0.6f) \
			|| flag_moving_cnt >= 2) {
			flag_moving = 1U;
		}
		else {
			flag_moving = 0U;
		}
		//std::cout << "track id: " << int(inst->uid) << ", the finally associated points: " << myPointNum << std::endl;
		// Calculate Bounding box
		length = cart_max[0] - cart_min[0];
		// Calculate the size of bounding box
		if (length < 0.1f) {
			length = 0.1f;
		}
		//if (length < inst->objGeometryProperty.length) {
		//	inst->objGeometryProperty.length = length*0.1f + inst->objGeometryProperty.length*0.9f;
		//}
		//else {
		//	inst->objGeometryProperty.length = length;
		//}
		width = cart_max[1] - cart_min[1];
		if (width < 0.1f) {
			width = 0.1f;
		}
		//if (width < inst->objGeometryProperty.width) {
		//	inst->objGeometryProperty.width = width*0.1f + inst->objGeometryProperty.width*0.9f;
		//}
		//else {
		//	inst->objGeometryProperty.width = width;
		//}
#ifdef TRACKING_3D
		height = cart_max[2] - cart_min[2];
		/*if (height < 0.5f) {
			height = 0.5f;
		}
		if (height < inst->height) {
			inst->height = height * 0.1f + inst->height * 0.9f;
		}
		else {
			inst->height = height;
		}*/
#endif
	}

	if(myPointNum) {
		/* Update estimated number of points */
		if(myPointNum > inst->objManagement.estNumOfPoints)
			inst->objManagement.estNumOfPoints = (float)(myPointNum);
		else
			inst->objManagement.estNumOfPoints = 0.99f*inst->objManagement.estNumOfPoints + 0.01f*((float)(myPointNum));

		/* lower bound for the estimated number of points is allocation threshold */ 
		if(inst->objManagement.estNumOfPoints < inst->tracking_params->advParams.allocationParams.pointsThre)
			inst->objManagement.estNumOfPoints = (float)(inst->tracking_params->advParams.allocationParams.pointsThre);

		/* Compute measurement centroid as SNR-weighted mean of associated points */
		//tracking_vectorScalarMul(MSIZE_SPH, u_sum.array, 1.0f/centroidSnr, u_centroid.array);
		//tracking_vectorScalarMul(MSIZE_SPH, u_sum.array, 1.0f, u_centroid.array);
		//fSNR = centroidSnr / myPointNum;

		/* Unroll centroid radial velocity based on target state 
		Compute the mean vecolity from the first frame to current frame */
		rvIn = u_centroid.vector.doppler;
		//tracking_velocityStateHandling(inst, &u_centroid.vector);
		inst->objKinematic.velocityHandling = VELOCITY_LOCKED;

		/* Compute mean measurment variance, if availbale */
		if(var != NULL) {
			tracking_vectorScalarMul(MSIZE_SPH, uvar_sum.array, 1.0f/(float)myPointNum, uvar_mean.array);
		}

		/* Update measurement spread if we have 2 or more unique points */
		if(myUniquePointNum > 1) {
			for(m = 0; m < MSIZE_SPH; m++) {            
				spread = u_max[m] - u_min[m];
				/* Unbiased spread estimation */
				spread = spread*(myUniquePointNum+1)/(myUniquePointNum-1);
				/* if spread can't be smaller than measurment error */
				if(spread < spreadMin[m])
					spread = spreadMin[m];
				/* if spread can't be larger than configured limits */
				if(spread > 2*inst->assoProperty.H_limits.array[m])
					spread = 2*inst->assoProperty.H_limits.array[m];
				if(spread > inst->curTarget.estSpread.array[m])
					inst->curTarget.estSpread.array[m] = spread;
				else
					inst->curTarget.estSpread.array[m] = (1.0f-TRACKING_SPREAD_ALPHA)*inst->curTarget.estSpread.array[m] + TRACKING_SPREAD_ALPHA*spread;
			}
		}
	}
	else {
		
	}
	if (myPointNum)
	{
		/* Compute Rm, Measurement Noise covariance matrix */
		if (var == 0) {
			for (m = 0; m < MSIZE_SPH; m++) {
				/* spread is covered by 2 sigmas */
				sigma = inst->curTarget.estSpread.array[m] / 2;
				uvar_mean.array[m] = sigma * sigma;
			}
		}
		//tracking_matrixSetDiag(MEASUREMENT_VECTOR_SIZE, uvar_mean.array, Rm);
		memcpy(inst->curTarget.Rm, uvar_mean.array, sizeof(float) * MSIZE_SPH);
	}
	/* Update Group Dispersion gD matrix */
	if(myPointNum > TRACKING_MIN_POINTS_TO_UPDATE_DISPERSION) {
		/* D is the new dispersion matrix, MSIZExMSIZE */
		for(n = 0; n < num; n++) {
			if(inst->assoProperty.bitIndicatorsOfDetAsso[n>>3] & (0x1 << (n & 0x7))){
				/* Accumulate covariance from all associated points */
				tracking_matrixCovAcc(MEASUREMENT_VECTOR_SIZE, D, point[n].array, u_centroid.array);
			}
		}
		/* Normalize it */
		tracking_matrixCovNormalize(MEASUREMENT_VECTOR_SIZE, D, myPointNum);

		/* Update persistant group dispersion based on instantaneous D */
		/* The factor alpha goes from maximum (1.f) at the first allocation down to minimum of 0.1f once the target is observed for the long time */ 
		if (inst->objManagement.estNumOfPoints > 0){
			alpha = ((float)myPointNum) / inst->objManagement.estNumOfPoints;
		}
		else {
			alpha = 0.0;
		}

		/*  inst->gD = (1-alpha)*inst->gD + alpha*D */
		tracking_matrixSetDiag(MEASUREMENT_VECTOR_SIZE, inst->curTarget.gD, Rc);
		tracking_matrixCovFilter(MEASUREMENT_VECTOR_SIZE, Rc, D, alpha);
		tracking_matrixGetDiag(MEASUREMENT_VECTOR_SIZE, Rc, inst->curTarget.gD);
	}
	if(myPointNum)
	{
		/* Compute centroid measurement noise covariance matrix Rc used for Kalman updates */
		/* First term represents the error in measuring the centroid and decreased with the number of measurments */
		/* Second term represents the centroid unsertanty due to the fact that not all the memebers observed */
		/* See page 327, 11A.2 of S.Blackman */
		/* Rc = Rm/num + alpha*unit.gD*eye(mSize_SPH); */

		/* alpha is weighting factor that is the function of the number of observed poionts versus total number of reflections in a group */
		/* alpha = (unit.maxAssociatedPoints-num)/((unit.maxAssociatedPoints-1)*num); */
		/* See page 327, 11A.3 of S.Blackman */

		if (inst->objManagement.estNumOfPoints > 1) {
			alpha = ((float)(inst->objManagement.estNumOfPoints - myPointNum)) / ((inst->objManagement.estNumOfPoints - 1)*myPointNum);
		}
		else {
			alpha = 0.0;
		}

		//  Rc.e11 = Rm.e11/myPointNum + alpha*inst->gD[0]; 
		//	Rc.e22 = Rm.e22/myPointNum + alpha*inst->gD[4]; 
		//	Rc.e33 = Rm.e33/myPointNum + alpha*inst->gD[8]; 

		if ( (!USING_CONSTMEANOISE) && myPointNum > TRACKING_MIN_POINTS_TO_UPDATE_DISPERSION) {
			//tracking_matrixGetDiag(MEASUREMENT_VECTOR_SIZE, inst->curTarget.gD, rm_diag.array);
			//memcpy(inst->curTarget.gD, rm_diag.array, sizeof(float)*MSIZE_SPH);
			//for (n = 0; n < MEASUREMENT_VECTOR_SIZE; n++)
			//	rm_diag.array[n] = uvar_mean.array[n] / myPointNum + rm_diag.array[n] * alpha;
			////tracking_matrixSetDiag(MEASUREMENT_VECTOR_SIZE, rm_diag.array, Rc);
			memcpy(inst->kalman_state.Rc, Rc, sizeof(float)*MSIZE_SPH);
			memcpy(inst->curTarget.Rm, Rc, sizeof(float)*MSIZE_SPH);
			//printf("id: %d, rm_diag array: %.3f, %.3f, %.3f \n", inst->uid, \
			//	rm_diag.array[0], rm_diag.array[1], rm_diag.array[2]);
		}
		else {
			memcpy(inst->kalman_state.Rc, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
			memcpy(inst->curTarget.Rm, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
			//switch (inst->stateVectorType) {
			//	case TRACKING_STATE_VECTORS_2DA:
			//	{
			//		float R0[3] = { 1.0f };
			//		memcpy(R0, const_tws_measurement_noise, sizeof(R0));
			//		for (n = 0; n < MEASUREMENT_VECTOR_SIZE; n++){
			//			Rc[n*MEASUREMENT_VECTOR_SIZE + n] = R0[n];
			//		}
			//		break;
			//	}
			//	case TRACKING_STATE_VECTORS_3DA:
			//	{
			//		float R0[4] = { 1.0f };
			//		memcpy(R0, const_tws_measurement_noise, sizeof(R0));
			//		for (n = 0; n < MEASUREMENT_VECTOR_SIZE; n++){
			//			Rc[n*MEASUREMENT_VECTOR_SIZE + n] = R0[n];
			//		}
			//		break;
			//	}
			//}
		}
	}
	else
	{
		memcpy(inst->kalman_state.Rc, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
		memcpy(inst->curTarget.Rm, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
	}
	if(USING_CONSTMEANOISE || myUniquePointNum <= 1) 
	{
		memcpy(inst->kalman_state.Rc, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
		memcpy(inst->curTarget.Rm, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
	}

#ifdef HUHANG
	if ((myPointNum)&& (fabsf(platformInfo->egoLineVel_lat) > 8.3f))
	{
		u_centroid.vector.range = sqrtf(u_cart_centroid[0] * u_cart_centroid[0] + u_cart_centroid[1] * u_cart_centroid[1]);
		u_centroid.vector.azimuthRad = myMath_atan2f(u_cart_centroid[1], u_cart_centroid[0]);
		u_centroid.vector.doppler = u_cart_centroid[2];
	}
#endif // HUHANG

	if(myPointNum) {
		tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u_centroid.array, cart_center);
		//memcpy(&inst->currentTarget_sph, &u_centroid, sizeof(TRACKING_measurementPoint));

#ifdef DEBUG_LOG
		if(myPointNum)
		{
			//my_printf("inst[%d], curTarget: %.3f, %.3f, %.3f\n", inst->uid, \
				u_centroid.vector.range, u_centroid.vector.azimuthRad*RAD2DEG, \
				u_centroid.vector.doppler);
			//my_printf("inst[%d], H_s: %.3f, %.3f, %.3f\n", inst->uid, \
				inst->kalman_state.H_s.vector.range, inst->kalman_state.H_s.vector.azimuthRad*RAD2DEG, \
				inst->kalman_state.H_s.vector.doppler);
		}
#endif


		memcpy(inst->curTarget.unionSph.array, u_centroid.array, sizeof(inst->curTarget.unionSph.array));
		inst->curTarget.unionSphVar.vector.range = 0.25f;
		inst->curTarget.unionSphVar.vector.doppler = 0.25f;
		inst->curTarget.unionSphVar.vector.azimuthRad = 4.0f*DEG2RAD*DEG2RAD;
		for(n=0; n<MSIZE_X; n++){
			inst->curTarget.unionX.array[n] = cart_center[n*inst->tracking_params->stateVectorDimLength];
			inst->curTarget.unionY.array[n] = cart_center[n*inst->tracking_params->stateVectorDimLength+1];
		}
		inst->curTarget.unionXVar.vector.p = 0.25f;
		inst->curTarget.unionXVar.vector.v = 0.25f;
		inst->curTarget.unionYVar.vector.p = 0.25f;
		inst->curTarget.unionYVar.vector.v = 0.25f;
		tracking_vectorSub(MSIZE_SPH, inst->curTarget.unionSph.array, inst->kalman_state.H_s.array, u_tilda.array);
		tracking_computeMahalanobis(u_tilda.array, inst->assoProperty.gC_inv, &inst->curTarget.mdScore);
#ifdef DEBUG_LOG
		//my_printf("inst[%d] mdScore: %.5f\n", inst->uid, inst->curTarget.mdScore);
#endif
		inst->curTarget.fPDH0 = fPDH0;
		inst->curTarget.fSNR = fSNR;
		inst->curTarget.fRCS = fRCS;
		inst->curTarget.fMag = fMag;
		inst->curTarget.fAngel = fAngel;
		//inst->objGeometryProperty.max_peakVal = fMag;
		inst->curTarget.fMagCoorX = fMagCoorX;
		inst->curTarget.fMagCoorY = fMagCoorY;
		inst->curTarget.fIdealStatVrad = tracking_getIdealStationaryVelocity( \
			inst->curTarget.unionX.vector.p, \
			inst->curTarget.unionY.vector.p, \
			inst->curTarget.unionSph.vector.range, \
			platformInfo);
		inst->curTarget.uNumOfDetections = myPointNum;
		inst->curTarget.uFlagMoving = flag_moving;
		inst->curTarget.uFlagNearOrFarScan = flag_nearOrFarScan;
		fVradAbs = inst->curTarget.unionSph.vector.doppler - inst->curTarget.fIdealStatVrad;
		getStaticThres(inst->curTarget.unionSph.array, &static_vel_thres, &static_velx_thres, \
			platformInfo->egoLineVel_long, platformInfo->egoLineVel_lat, platformInfo->egoOmega);
		if(fabsf(fVradAbs)<static_vel_thres && flag_moving==0U)
		{
			//inst->eDynamicProperty == TRACKING_DYNAMIC_STATIC;
			inst->curTarget.uFlagMoving = 0U;
		}
		else
		{
			inst->curTarget.uFlagMoving = 1U;
		}
	}
	else {
		inst->curTarget.uNumOfDetections = 0;
		if(inst->eDynamicProperty == TRACKING_DYNAMIC_STATIC)
		{
			inst->curTarget.uFlagMoving = 0;
		}
		else
		{
			inst->curTarget.uFlagMoving = 1;
		}
	}
}


void tracking_unitCurrentTarget(void *handle, sMeasurement_SphPoint *point, sMeasurement_Sph_vector *var,
	uint16_t *pInd, uint8_t *isUnique, uint16_t num, sTracking_platformInfo *platformInfo)
{
	sTracking_objectUnit *inst = NULL;
	uint16_t n = 0;
	uint16_t m = 0;

	uint32_t myPointNum = 0;
	uint32_t myUniquePointNum = 0;
	float alpha = 0.f;
	float rvPilot = 0.f;
	float vel = 0.f;
	float u_max[MSIZE_SPH];
	float u_min[MSIZE_SPH];
	float cart_max[SSIZE_XY];
	float cart_min[SSIZE_XY];
	float cart_center[SSIZE_XY];

	/* D is Dispersion matrix */
	float D[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	/* Rm is Measurement error covariance matrix */
	//float *Rm = gScratchPadData->Rm;
	/* Rc is Measurement error covariance matrix for the centroid used for Kalman update */
	float Rc[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float length = 0.5f;
	float width = 0.5f;
	float height = 0.5f;
	float fSNR = 0.f;
	float fRCS = -1000.f;
	float fMag = 0.f;
	float fPDH0 = 0.1f;

	float fYawRate = (fabsf(platformInfo->egoOmega)<0.5f*DEG2RAD)? 0.0f : platformInfo->egoOmega;
	float fSensorVx = platformInfo->egoLineVel_long - fYawRate*platformInfo->radarDist_lat;
	float fSensorVy = platformInfo->egoLineVel_lat + fYawRate*platformInfo->radarDist_long;

	sTracking_measurementSphUnion 	u;
	sTracking_measurementSphUnion 	u_sum;
	sTracking_measurementSphUnion 	u_centroid;
	sTracking_measurementSphUnion 	u_tilda;
	float u_cart[SSIZE_XY] = { 0.f };
	float centroidSnr = 0.f;
	float spread = 0.f;
	float sigma = 0.f;
	float rvIn = 0.f;
	uint8_t targetCanStop = true;
	float rvOut = 0.f;
	float sinAngle = 0.f;
	float cosAngle = 0.f;
	float doppler_tmp = 0.f;
	float rvMax = 0.f;

	sTracking_measurementSphUnion 	uvar;
	sTracking_measurementSphUnion     uvar_sum;
	sTracking_measurementSphUnion 	uvar_mean;
	uint8_t flag_nearOrFarScan = 0U;
	uint8_t flag_moving = 0U;
	uint16_t flag_moving_cnt = 0U;
	sTracking_measurementSphUnion 	rm_diag;
	float fVradAbs = 100.f;
	sTracking_gateLimits assocDynamicGating_tmp; 
	int dimLen = 0;
	int i = 0;
	float static_vel_thres = TRACKING_MIN_STATIC_VELOCITY_RAD;
	float static_velx_thres = TRACKING_MIN_STATIC_VELOCITY;
	sBox nearBox;
	uint8_t genCurT = 0U;

	memset((uint8_t *)&u_centroid, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&u_tilda, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&u, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&uvar_mean, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&uvar, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&rm_diag, 0, sizeof(sTracking_measurementSphUnion));
	memset((uint8_t *)&assocDynamicGating_tmp, 0, sizeof(assocDynamicGating_tmp));
	inst = (sTracking_objectUnit *)handle;
	myPointNum = 0;
	myUniquePointNum = 0;

	// 1. Set prediction state with saved state
	// 2. The compesanted state is just used for association
	dimLen = inst->tracking_params->stateVectorDimLength;
	/* Convert from cartesian to spherical */
    tracking_cartesian2spherical(inst->tracking_params->stateVectorType, inst->kalman_state.S_hat, inst->kalman_state.H_s.array);

	if(inst->objManagement.isNewTracker==1){
		return;
	}

	tracking_vectorInit(MSIZE_SPH, 0.f, u_sum.array);
	tracking_vectorInit(MSIZE_SPH, 0.f, uvar_sum.array);
	tracking_vectorInit(MSIZE_SPH, -FLT_MAX, u_max);
	tracking_vectorInit(MSIZE_SPH, FLT_MAX, u_min);
	tracking_vectorInit(SSIZE_XY, -FLT_MAX, cart_max);
	tracking_vectorInit(SSIZE_XY, FLT_MAX, cart_min);
	tracking_vectorInit(SSIZE_XY, 0.f, cart_center);
	tracking_vectorInit(MSIZE_SPH, 0.f, u_centroid.array);
	tracking_vectorInit(MSIZE_SPH, 0.f, rm_diag.array);

	//printf("%llu: uid[%d]: prediction inst->S_apriori_hat: (%.2f, %.2f)\n", \
	//	inst->heartBeatCount, inst->uid, inst->S_apriori_hat[0], inst->S_apriori_hat[1]);
	/* Compute means of associated measurement points */
	/* Accumulate measurements */
	for(n = 0; n < num; n++) {
		if(pInd[n] == inst->uid) {
			memcpy(u.array, point[n].array, sizeof(u.array));

			/* Compute maximums and minimums for each measurement */
			//if(isUnique[n>>3] & (0x1 << (n & 0x0007))) {
			if(1) {
				for(m = 0; m < MSIZE_SPH; m++) {
					if(u.array[m] > u_max[m])
						u_max[m] = u.array[m];
					if(u.array[m] < u_min[m])
						u_min[m] = u.array[m];
				}
				myUniquePointNum++;
				tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u.array, u_cart);
				for(m = 0; m < SSIZE_XY; m++) {
					if(u_cart[m] > cart_max[m])
						cart_max[m] = u_cart[m];
					if(u_cart[m] < cart_min[m])
						cart_min[m] = u_cart[m];
				}
			}
			if(var != NULL) {
				uvar.vector = var[n];
				tracking_vectorAdd(MSIZE_SPH, uvar.array, uvar_sum.array, uvar_sum.array); 
			}

//			if(myPointNum == 0)
//				rvPilot = u.vector.doppler;
//			else
//#ifdef ENABLE_DISAMBIGUITYVELOCITY
//				u.vector.doppler = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, rvPilot, u.vector.doppler); /* Unroll using rvPilot */
//#else
//				u.vector.doppler = rvPilot;
//#endif

			if (point[n].snr != 0) {
				centroidSnr += point[n].snr;
				//tracking_vectorScalarMulAcc(MSIZE_SPH, u.array,  point[n].snr, u_sum.array);
			}
			else {
				centroidSnr += 1.0;
				//tracking_vectorScalarMulAcc(MSIZE_SPH, u.array,  1.0, u_sum.array);
			}
			fSNR = (point[n].snr>fSNR)? point[n].snr : fSNR;
			fMag = (point[n].mag>fMag)? point[n].mag : fMag;
			fRCS = (point[n].rcs>fRCS)? point[n].rcs : fRCS;
			if (0 && inst->objKinematic.velocityHandling < VELOCITY_LOCKED) {
#ifdef ENABLE_DISAMBIGUITYVELOCITY
				rvOut = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->objManagement.allocationVelocity, u.vector.doppler);
#else
				rvOut = u.vector.doppler;
#endif
			}
			else {
				// Consider the long object when crossing
				myMath_sincosd(u.vector.azimuthRad*RAD2DEG, &sinAngle, &cosAngle);
				doppler_tmp = inst->kalman_state.S_hat[2]*cosAngle + inst->kalman_state.S_hat[3]*sinAngle;
#ifdef ENABLE_DISAMBIGUITYVELOCITY
				if(point[n].flag_fastOrSlowScan == 0U)
				{
					rvMax = DISAMBIGUITY_MAXVEL_FASTSCAN;
				}
				else if(point[n].flag_fastOrSlowScan == 1U)
				{
					rvMax = DISAMBIGUITY_MAXVEL_SLOWSCAN;
				}
				/* Radial velocity estimation is known */
				rvOut = tracking_unrollRadialVelocity(rvMax, doppler_tmp, u.vector.doppler);
#else
				rvOut = u.vector.doppler;
#endif
				u.vector.doppler = rvOut;
				//point[n].vector.doppler = rvOut;
				point[n].disambigVelReliable = 1.0f;
				//rvOut = tracking_unrollRadialVelocity(inst->tracking_params->maxRadialVelocity, inst->kalman_state.H_s.vector.doppler, point[n].vector.doppler);
			}
			tracking_vectorScalarMulAcc(MSIZE_SPH, u.array,  1.0, u_sum.array);
			myPointNum++;
			if(point[n].flag_moving!=0U){
				flag_moving_cnt++;
			}
#ifdef DEBUG_LOG
			//my_printf("inst[%d] currentTarget point[%d]\n", inst->uid, n);
#endif
		}
	}
	if(    flag_moving_cnt>=(int)(((float)(myPointNum))*0.5f+0.6f) \
		|| flag_moving_cnt>=2){
		flag_moving = 1U;
	}
	else{
		flag_moving = 0U;
	}
	// Calculate Bounding box
	length = cart_max[0] - cart_min[0];
	// Calculate the size of bounding box
	if (length < 0.5f) {
		length = 0.5f;
	}
	width = cart_max[1] - cart_min[1];
	if (width < 0.5f) {
		width = 0.5f;
	}
#ifdef TRACKING_3D
	height = cart_max[2] - cart_min[2];
	if (height < 0.5f) {
		height = 0.5f;
	}
	/*if (height < inst->height) {
		inst->height = height*0.1f + inst->height*0.9f;
	}
	else {
		inst->height = height;
	}*/
#endif

	if(myPointNum) {
		/* Update estimated number of points */
		if(myPointNum > inst->objManagement.estNumOfPoints)
			inst->objManagement.estNumOfPoints = (float)(myPointNum);
		else
			inst->objManagement.estNumOfPoints = 0.99f*inst->objManagement.estNumOfPoints + 0.01f*((float)(myPointNum));

		/* lower bound for the estimated number of points is allocation threshold */ 
		if(inst->objManagement.estNumOfPoints < inst->tracking_params->advParams.allocationParams.pointsThre)
			inst->objManagement.estNumOfPoints = (float)(inst->tracking_params->advParams.allocationParams.pointsThre);

		/* Compute measurement centroid as SNR-weighted mean of associated points */
		//tracking_vectorScalarMul(MSIZE_SPH, u_sum.array, 1.0f/centroidSnr, u_centroid.array);
		tracking_vectorScalarMul(MSIZE_SPH, u_sum.array, 1.0f/myPointNum, u_centroid.array);
		//fSNR = centroidSnr / myPointNum;

		/* Unroll centroid radial velocity based on target state 
		Compute the mean vecolity from the first frame to current frame */
		rvIn = u_centroid.vector.doppler;
		//tracking_velocityStateHandling(inst, &u_centroid.vector);
		inst->objKinematic.velocityHandling = VELOCITY_LOCKED;

		/* Compute mean measurment variance, if availbale */
		if(var != NULL) {
			tracking_vectorScalarMul(MSIZE_SPH, uvar_sum.array, 1.0f/(float)myPointNum, uvar_mean.array);
		}

		/* Update measurement spread if we have 2 or more unique points */
		if(myUniquePointNum > 1) {
			for(m = 0; m < MSIZE_SPH; m++) {            
				spread = u_max[m] - u_min[m];
				/* Unbiased spread estimation */
				spread = spread*(myUniquePointNum+1)/(myUniquePointNum-1);
				/* if spread can't be smaller than measurment error */
				if(spread < spreadMin[m])
					spread = spreadMin[m];
				/* if spread can't be larger than configured limits */
				if(spread > 2*inst->assoProperty.H_limits.array[m])
					spread = 2*inst->assoProperty.H_limits.array[m];
				if(spread > inst->curTarget.estSpread.array[m])
					inst->curTarget.estSpread.array[m] = spread;
				else
					inst->curTarget.estSpread.array[m] = (1.0f-TRACKING_SPREAD_ALPHA)*inst->curTarget.estSpread.array[m] + TRACKING_SPREAD_ALPHA*spread;
			}
		}
	}
	else {
	
	}

	/* Compute Rm, Measurement Noise covariance matrix */
	if(var == 0) {    
		for(m = 0; m < MSIZE_SPH; m++) {   
			/* spread is covered by 2 sigmas */
			sigma = inst->curTarget.estSpread.array[m]/2;
			uvar_mean.array[m] = sigma*sigma;
		}
	}
	//tracking_matrixSetDiag(MEASUREMENT_VECTOR_SIZE, uvar_mean.array, Rm);
	memcpy(inst->curTarget.Rm, uvar_mean.array, sizeof(float)*MSIZE_SPH);

	/* Update Group Dispersion gD matrix */
	if(myPointNum > TRACKING_MIN_POINTS_TO_UPDATE_DISPERSION) {
		/* D is the new dispersion matrix, MSIZExMSIZE */
		for(n = 0; n < num; n++) {
			if(pInd[n] == inst->uid)
				/* Accumulate covariance from all associated points */
				tracking_matrixCovAcc(MEASUREMENT_VECTOR_SIZE, D, point[n].array, u_centroid.array);
		}
		/* Normalize it */
		tracking_matrixCovNormalize(MEASUREMENT_VECTOR_SIZE, D, myPointNum);

		/* Update persistant group dispersion based on instantaneous D */
		/* The factor alpha goes from maximum (1.f) at the first allocation down to minimum of 0.1f once the target is observed for the long time */ 
		if (inst->objManagement.estNumOfPoints > 0){
			alpha = ((float)myPointNum) / inst->objManagement.estNumOfPoints;
		}
		else {
			alpha = 0.0;
		}

		/*  inst->gD = (1-alpha)*inst->gD + alpha*D */
		tracking_matrixSetDiag(MEASUREMENT_VECTOR_SIZE, inst->curTarget.gD, Rc);
		tracking_matrixCovFilter(MEASUREMENT_VECTOR_SIZE, Rc, D, alpha);
		tracking_matrixGetDiag(MEASUREMENT_VECTOR_SIZE, Rc, inst->curTarget.gD);
	}
	if(myPointNum)
	{
		/* Compute centroid measurement noise covariance matrix Rc used for Kalman updates */
		/* First term represents the error in measuring the centroid and decreased with the number of measurments */
		/* Second term represents the centroid unsertanty due to the fact that not all the memebers observed */
		/* See page 327, 11A.2 of S.Blackman */
		/* Rc = Rm/num + alpha*unit.gD*eye(mSize_SPH); */

		/* alpha is weighting factor that is the function of the number of observed poionts versus total number of reflections in a group */
		/* alpha = (unit.maxAssociatedPoints-num)/((unit.maxAssociatedPoints-1)*num); */
		/* See page 327, 11A.3 of S.Blackman */

		if (inst->objManagement.estNumOfPoints > 1) {
			alpha = ((float)(inst->objManagement.estNumOfPoints - myPointNum)) / ((inst->objManagement.estNumOfPoints - 1)*myPointNum);
		}
		else {
			alpha = 0.0;
		}

		//  Rc.e11 = Rm.e11/myPointNum + alpha*inst->gD[0]; 
		//	Rc.e22 = Rm.e22/myPointNum + alpha*inst->gD[4]; 
		//	Rc.e33 = Rm.e33/myPointNum + alpha*inst->gD[8]; 

		if ((!USING_CONSTMEANOISE) && myPointNum > TRACKING_MIN_POINTS_TO_UPDATE_DISPERSION) {
			//tracking_matrixGetDiag(MEASUREMENT_VECTOR_SIZE, inst->curTarget.gD, rm_diag.array);
			memcpy(inst->curTarget.gD, rm_diag.array, sizeof(float)*MSIZE_SPH);
			for (n = 0; n < MEASUREMENT_VECTOR_SIZE; n++)
				rm_diag.array[n] = uvar_mean.array[n] / myPointNum + rm_diag.array[n] * alpha;
			//tracking_matrixSetDiag(MEASUREMENT_VECTOR_SIZE, rm_diag.array, Rc);
			memcpy(inst->kalman_state.Rc, rm_diag.array, sizeof(float)*MSIZE_SPH);
			//printf("id: %d, rm_diag array: %.3f, %.3f, %.3f \n", inst->uid, \
			//	rm_diag.array[0], rm_diag.array[1], rm_diag.array[2]);
		}
		else {
			memcpy(inst->kalman_state.Rc, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
			//switch (inst->stateVectorType) {
			//	case TRACKING_STATE_VECTORS_2DA:
			//	{
			//		float R0[3] = { 1.0f };
			//		memcpy(R0, const_tws_measurement_noise, sizeof(R0));
			//		for (n = 0; n < MEASUREMENT_VECTOR_SIZE; n++){
			//			Rc[n*MEASUREMENT_VECTOR_SIZE + n] = R0[n];
			//		}
			//		break;
			//	}
			//	case TRACKING_STATE_VECTORS_3DA:
			//	{
			//		float R0[4] = { 1.0f };
			//		memcpy(R0, const_tws_measurement_noise, sizeof(R0));
			//		for (n = 0; n < MEASUREMENT_VECTOR_SIZE; n++){
			//			Rc[n*MEASUREMENT_VECTOR_SIZE + n] = R0[n];
			//		}
			//		break;
			//	}
			//}
		}
	}
	if(USING_CONSTMEANOISE || myUniquePointNum <= 1) 
	{
		memcpy(inst->curTarget.Rm, const_tws_measurement_noise, sizeof(float)*MSIZE_SPH);
	}

	nearBox.x1 = 0.f;
	nearBox.x2 = 10.f;
	nearBox.y1 = -5.f;
	nearBox.y2 = 5.f;
	tracking_spherical2cartesian(inst->tracking_params->stateVectorType, u_centroid.array, cart_center);
	//if(    myPointNum<2 \
	//	&& inst->objManagement.state==TRACK_STATE_DETECTION \
	//	&& inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC \
	//	&& myMath_isPointInsideBox(cart_center, &nearBox)) {
	//	genCurT = 0U;
	//}
	//else if(myPointNum>0){
	if(myPointNum>0){
		genCurT = 1U;
	}
	else{
		genCurT = 0U;
	}
	if(genCurT) {
		// Update dynamic gating
		assocDynamicGating_tmp.depth = fabsf(cart_center[0]-inst->kalman_state.S_hat[0])*2.f;
		assocDynamicGating_tmp.depth = (assocDynamicGating_tmp.depth<inst->tracking_params->advParams.gatingParams.limits.depth*0.7f)? (assocDynamicGating_tmp.depth):(inst->tracking_params->advParams.gatingParams.limits.depth*0.7f);
		assocDynamicGating_tmp.width = fabsf(cart_center[1]-inst->kalman_state.S_hat[1])*2.f;
		assocDynamicGating_tmp.width = (assocDynamicGating_tmp.width<inst->tracking_params->advParams.gatingParams.limits.width*0.7f)? (assocDynamicGating_tmp.width):(inst->tracking_params->advParams.gatingParams.limits.width*0.7f);
		assocDynamicGating_tmp.vel = fabsf(u_centroid.vector.doppler-inst->kalman_state.H_s.vector.doppler)*2.f;
		assocDynamicGating_tmp.vel = (assocDynamicGating_tmp.vel<inst->tracking_params->advParams.gatingParams.limits.vel*0.7f)? (assocDynamicGating_tmp.vel):(inst->tracking_params->advParams.gatingParams.limits.vel*0.7f);
		assocDynamicGating_tmp.height = inst->assoProperty.assocDynamicGating.height;
		myMath_lowPassFilter(&inst->assoProperty.assocDynamicGating.depth, assocDynamicGating_tmp.depth, 0.3f);
		myMath_lowPassFilter(&inst->assoProperty.assocDynamicGating.width, assocDynamicGating_tmp.width, 0.3f);
		myMath_lowPassFilter(&inst->assoProperty.assocDynamicGating.vel, assocDynamicGating_tmp.vel, 0.3f);
		// Calculate the size of bounding box
		if (inst->assoProperty.assocDynamicGating.depth < 1.0f) {
			inst->assoProperty.assocDynamicGating.depth = 1.0f;
		}
		if (inst->assoProperty.assocDynamicGating.width < 1.0f) {
			inst->assoProperty.assocDynamicGating.width = 1.0f;
		}
		//memcpy(&inst->currentTarget_sph, &u_centroid, sizeof(TRACKING_measurementPoint));
		memcpy(inst->curTarget.unionSph.array, u_centroid.array, sizeof(inst->curTarget.unionSph.array));
		inst->curTarget.unionSphVar.vector.range = 0.25f;
		inst->curTarget.unionSphVar.vector.doppler = 0.25f;
		inst->curTarget.unionSphVar.vector.azimuthRad = 4.0f*DEG2RAD*DEG2RAD;
		for(n=0; n<MSIZE_X; n++){
			inst->curTarget.unionX.array[n] = cart_center[n*inst->tracking_params->stateVectorDimLength];
			inst->curTarget.unionY.array[n] = cart_center[n*inst->tracking_params->stateVectorDimLength+1];
		}
//#ifdef TA_PROJECT
//		myMath_sincosd(inst->curTarget.unionSph.vector.azimuthRad*RAD2DEG, &sinAngle, &cosAngle);
//		if(fabsf(sinAngle)>0.01f)
//		{
//			inst->curTarget.unionX.array[TRACKING_VEL_DIMENSION] = -1.f*platformInfo->egoLineVel_long + \
//				fYawRate*(inst->curTarget.unionY.vector.p+platformInfo->radarDist_lat);
//			inst->curTarget.unionY.array[TRACKING_VEL_DIMENSION] = \
//				(inst->curTarget.unionSph.vector.doppler - inst->curTarget.unionX.array[TRACKING_VEL_DIMENSION]*cosAngle)/ sinAngle;
//		}
//#else
//		if(fabsf(cosAngle)>0.01f)
//		{
//			inst->curTarget.unionY.array[TRACKING_VEL_DIMENSION] = -1.f*platformInfo->egoLineVel_lat + \
//				fYawRate*(inst->curTarget.unionX.vector.p+platformInfo->radarDist_long);
//			inst->curTarget.unionX.array[TRACKING_VEL_DIMENSION] = \
//				(inst->curTarget.unionSph.vector.doppler - inst->curTarget.unionY.array[TRACKING_VEL_DIMENSION]*sinAngle)/ cosAngle;
//		}
//#endif
		inst->curTarget.unionXVar.vector.p = 0.25f;
		inst->curTarget.unionXVar.vector.v = 0.25f;
		inst->curTarget.unionYVar.vector.p = 0.25f;
		inst->curTarget.unionYVar.vector.v = 0.25f;
		tracking_vectorSub(MSIZE_SPH, inst->curTarget.unionSph.array, inst->kalman_state.H_s.array, u_tilda.array);
		tracking_computeMahalanobis(u_tilda.array, inst->assoProperty.gC_inv, &inst->curTarget.mdScore);
#ifdef DEBUG_LOG
		//my_printf("inst[%d] mdScore: %.5f\n", inst->uid, inst->curTarget.mdScore);
#endif
		inst->curTarget.fPDH0 = fPDH0;
		inst->curTarget.fSNR = fSNR;
		inst->curTarget.fRCS = fRCS;
		inst->curTarget.fMag = fMag;
		inst->curTarget.fIdealStatVrad = tracking_getIdealStationaryVelocity( \
			inst->curTarget.unionX.vector.p, \
			inst->curTarget.unionY.vector.p, \
			inst->curTarget.unionSph.vector.range, \
			platformInfo);
		inst->curTarget.uNumOfDetections = myPointNum;
		//inst->curTarget.uFlagMoving = flag_moving;
		inst->curTarget.uFlagNearOrFarScan = flag_nearOrFarScan;
		fVradAbs = inst->curTarget.unionSph.vector.doppler - inst->curTarget.fIdealStatVrad;
		getStaticThres(inst->curTarget.unionSph.array, &static_vel_thres, &static_velx_thres, \
			platformInfo->egoLineVel_long, platformInfo->egoLineVel_lat, platformInfo->egoOmega);
		if(fabsf(fVradAbs)<static_vel_thres && flag_moving==0U)
		{
			//inst->eDynamicProperty == TRACKING_DYNAMIC_STATIC;
			inst->curTarget.uFlagMoving = 0U;
		}
		else
		{
			inst->curTarget.uFlagMoving = 1U;
		}
	}
	else {
		inst->curTarget.uNumOfDetections = 0U;
		if(inst->eDynamicProperty == TRACKING_DYNAMIC_STATIC)
		{
			inst->curTarget.uFlagMoving = 0U;
		}
		else
		{
			inst->curTarget.uFlagMoving = 1U;
		}
	}
}

void tracking_unitCalcCurrentTargetVelocityAndVar(void *handle, sTracking_platformInfo *platformInfo)
{
	sTracking_objectUnit *inst;
	float dt = 0;
	uint8_t trackerMoving_flag = 0U;

	inst = (sTracking_objectUnit *)handle;

#ifdef DEBUG_LOG
	//my_printf("inst[%d] has numOfDets: %d (conf: %.3f) in Calculating curTargetVel\n", \
		inst->uid, inst->curTarget.uNumOfDetections, inst->curTarget.fCurTargetConf);
#endif

	if(inst->curTarget.uNumOfDetections==0){
		return;
	}

	trackerMoving_flag = (inst->eDynamicProperty==TRACKING_DYNAMIC_STATIC)? 0U : 1U;
	if(inst->curTarget.uFlagMoving == 0U)
	{
		tracking_calculateCurrentTargetRelativeVelocity(&inst->curTarget.unionX.array[TRACKING_VEL_DIMENSION], \
			&inst->curTarget.unionY.array[TRACKING_VEL_DIMENSION], 0.f, &inst->curTarget.unionSph, \
			&inst->curTarget.unionX, &inst->curTarget.unionY, inst->curTarget.fIdealStatVrad, \
			inst->kalman_state.S_hat[2], inst->kalman_state.S_hat[3], &inst->curTarget.fCurTargetConf, trackerMoving_flag, \
			0U, platformInfo);
	}
	else
	{
		//tracking_calculateTrackerHeading(inst, platformInfo);
		tracking_calculateCurrentTargetRelativeVelocity(&inst->curTarget.unionX.array[TRACKING_VEL_DIMENSION], \
			&inst->curTarget.unionY.array[TRACKING_VEL_DIMENSION], inst->kalman_state.fTrackerHeading, &inst->curTarget.unionSph, \
			&inst->curTarget.unionX, &inst->curTarget.unionY, inst->curTarget.fIdealStatVrad, \
			inst->kalman_state.S_hat[2], inst->kalman_state.S_hat[3], &inst->curTarget.fCurTargetConf, trackerMoving_flag, \
			1U, platformInfo);
	}
	tracking_calculateCurrentTargetRelativeVelocityVar(inst, platformInfo);
#ifdef DEBUG_LOG
		//my_printf("id[%d]: curTarget relVx: %.3f, relVy: %.3f, heding: %.3f\n", \
			inst->uid, \
			inst->curTarget.unionX.array[TRACKING_VEL_DIMENSION], \
			inst->curTarget.unionY.array[TRACKING_VEL_DIMENSION], \
			inst->kalman_state.fTrackerHeading*RAD2DEG \
			);
#endif
}

void tracking_calculateCurrentTargetRelativeVelocity(float *relVx, float *relVy, float fObjHeading, \
	sTracking_measurementSphUnion *unionSph, sTracking_measurementCartUnion *unionX, \
	sTracking_measurementCartUnion *unionY, float fVradIdealStationary, \
	float tracker_relVx, float tracker_relVy, float* fConf, uint8_t trackerMoving_flag, \
	uint8_t movingFlag, sTracking_platformInfo *platformInfo)
{
	float fYawRate = (fabsf(platformInfo->egoOmega)<0.5f*DEG2RAD)? 0.0f : platformInfo->egoOmega;
	float fSensorVx = platformInfo->egoLineVel_long - fYawRate*platformInfo->radarDist_lat;
	float fSensorVy = platformInfo->egoLineVel_lat + fYawRate*platformInfo->radarDist_long;
	float fCurrentRange = unionSph->vector.range;
	float fCurrentTargetVradAbs = 0.f;
	float fCosObjHeading = 0.f;
	float fSinObjHeading = 0.f;
	float fCurrentTargetVAbs = 0.f;
	float tmp = 0.f;
	float fCurrentTargetRelSpeedX = 0.f;
	float fCurrentTargetRelSpeedY = 0.f;

	if(movingFlag==1U)
	{
		myMath_sincosd(fObjHeading*RAD2DEG, &fSinObjHeading, &fCosObjHeading);
		fCurrentTargetVradAbs = unionSph->vector.doppler - fVradIdealStationary;
		tmp = fCosObjHeading * unionX->vector.p + fSinObjHeading * unionY->vector.p;
		myMath_checkZero(&tmp);
#ifdef DEBUG_LOG
		//my_printf("tmp: %.3f, Heading: %.3f, x: %.3f, y: %.3f\n", \
			tmp, fObjHeading*RAD2DEG, unionX->vector.p, unionY->vector.p);
#endif
		if(fabsf(tmp)<0.5){
			*relVx = tracker_relVx;
			*relVy = tracker_relVy;
			*fConf = 0.1f;
		}
		else{
			fCurrentTargetVAbs = (fCurrentTargetVradAbs * unionSph->vector.range) / tmp;
			fCurrentTargetRelSpeedX = fCurrentTargetVAbs * fCosObjHeading - fSensorVx + fYawRate * unionY->vector.p;
			fCurrentTargetRelSpeedY = fCurrentTargetVAbs * fSinObjHeading - fSensorVy - fYawRate * unionX->vector.p;
			*relVx = fCurrentTargetRelSpeedX;
			*relVy = fCurrentTargetRelSpeedY;
		}
	}
	else if(0U==movingFlag && 1U==trackerMoving_flag){
		*relVx = tracker_relVx;
		*relVy = tracker_relVy;
		*fConf = 0.1f;
	}
	else
	{
		fCurrentTargetRelSpeedX = -1.0f * fSensorVx + fYawRate * unionY->vector.p;
		fCurrentTargetRelSpeedY = -1.0f * fSensorVy - fYawRate * unionX->vector.p;
		*relVx = fCurrentTargetRelSpeedX;
		*relVy = fCurrentTargetRelSpeedY;
	}
}

// Based on associated detections
void tracking_calculateCurrentTargetRelativeVelocityVar(sTracking_objectUnit *inst, sTracking_platformInfo *platformInfo)
{
	float fCosCurTargetAzim = 0.f;
	float fSinCurTargetAzim = 0.f;
	float fCurTargetVxVar = 0.f;
	float fCurTargetVyVar = 0.f;

	myMath_sincosd(inst->curTarget.unionSph.vector.azimuthRad*RAD2DEG, &fSinCurTargetAzim, &fCosCurTargetAzim);
	myMath_checkZero(&fCosCurTargetAzim);
	fCurTargetVxVar = (inst->curTarget.unionSphVar.vector.doppler + inst->curTarget.unionSphVar.vector.azimuthRad) / fCosCurTargetAzim;
	fCurTargetVyVar = (inst->curTarget.unionSphVar.vector.doppler + inst->curTarget.unionSphVar.vector.azimuthRad) / fSinCurTargetAzim;
	//	// For debug
	//	fCurTargetVxVar = 0.2f;
	//	fCurTargetVyVar = 0.2f;

	myMath_minMax(&fCurTargetVxVar, 0.1f, 15.f);
	myMath_minMax(&fCurTargetVyVar, 0.1f, 15.f);
	inst->curTarget.unionXVar.vector.v = fCurTargetVxVar;
	inst->curTarget.unionYVar.vector.v = fCurTargetVyVar;
}

