
#include <math.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

/**
*  @b Description
*  @n
*		This function is used to convert a vector from sherical to cartesian
*
*  @param[in]  format
*		Format used
*  @param[in]  sph
*		Pointer to measurements (spheriacl form) vector
*  @param[in]  cart
*		Pointer to state vector (cartesian form)
*
*  \ingroup tracking_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void tracking_spherical2cartesian(sTracking_enum_state_vector_type format, float *sph, float *cart)
{

	float range; 
	float angle; 
	float azim; 
	float pitchRad; 
	float doppler;
    float sinAngle, cosAngle;
    float sinAzim, cosAzim;
    float sinElev, cosElev;

	switch (format) {
        default:
		case TRACKING_STATE_VECTORS_2DA:
			cart[4] = 0; cart[5] = 0;
			/* fall through */
		case TRACKING_STATE_VECTORS_2DV:
            range = sph[0]; angle = sph[1]; doppler = sph[2];
            myMath_sincosd(angle*RAD2DEG,&sinAngle, &cosAngle);
			cart[0] = range*cosAngle;
			cart[1] = range*sinAngle;
			cart[2] = doppler*cosAngle;
			cart[3] = doppler*sinAngle;
			break;

		case TRACKING_STATE_VECTORS_3DA:
			cart[6] = cart[7] = cart[8] = 0;
			/* fall through */
		case TRACKING_STATE_VECTORS_3DV:
            range = sph[0]; azim = sph[1]; pitchRad = sph[2]; doppler = sph[3];
            myMath_sincosd(azim*RAD2DEG,&sinAzim, &cosAzim);
            myMath_sincosd(pitchRad*RAD2DEG,&sinElev, &cosElev);
			cart[0] = range*cosElev*cosAzim;
			cart[1] = range*cosElev*sinAzim;
			cart[2] = range*sinElev;
			
            cart[3] = doppler*cosElev*cosAzim;
			cart[4] = doppler*cosElev*sinAzim;
			cart[5] = doppler*sinElev;
            break;
	}
}
/**
*  @b Description
*  @n
*		This function is used to convert a vector from cartesian to sherical to cartesian
*
*  @param[in]  format
*		Format used
*  @param[in]  cart
*		Pointer to state vector (cartesian form)
*  @param[in] sph 
*		Pointer to measurements (spheriacl form) vector
*
*  \ingroup TRACKING_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/

void tracking_cartesian2spherical(sTracking_enum_state_vector_type format, float *cart, float *sph)
{
	float posx, posy, posz; 
	float velx, vely, velz;
	float range;

	switch (format) {
        default:
		case TRACKING_STATE_VECTORS_2DV:	
			/* cart = [posx posy velx vely] */
			/* sph = [range azimuth doppler] */
		case TRACKING_STATE_VECTORS_2DA:
			/* cart = [posx posy velx vely accx accy] */
			/* sph = [range azimuth doppler] */
			posx = cart[0]; posy = cart[1]; 
			velx = cart[2]; vely = cart[3];

			/* calc range */
			sph[0] = sqrtf(posx*posx + posy*posy); 
			range = sph[0];
			myMath_checkZero(&range);
			/* calc azimuth */
			sph[1] = myMath_atan2f(posy, posx);
			/* calc doppler */
			sph[2] = (posx*velx+posy*vely)/range;	
			break;

		case TRACKING_STATE_VECTORS_3DV:	
			/* cart = [posx posy posz velx vely velz accx accy accz] */
			/* sph = [range azimuth elevation doppler] */
		case TRACKING_STATE_VECTORS_3DA:	
			/* cart = [posx posy posz velx vely velz] */
			/* sph = [range azimuth elevation doppler] */
            posx = cart[0]; posy = cart[1]; posz = cart[2];
            velx = cart[3]; vely = cart[4]; velz = cart[5];
			/* calc range */
			sph[0] = sqrtf(posx*posx + posy*posy + posz*posz); 
			/* calc azimuth */
			sph[1] = myMath_atan2f(posy, posx);
            /* calc elevation */
			range = sqrtf(posx*posx+posy*posy);
			myMath_checkZero(&range);
            sph[2] = atanf(posz/range);
			/* calc doppler */
			range = sph[0];
			myMath_checkZero(&range);
			sph[3] = (posx*velx+posy*vely+posz*velz)/range;	
			break;
	}
}
void tracking_measurementCovCal(sTracking_objectUnit* inst)
{
	float J[MSIZE_SPH*SSIZE_XY] = { 0.f };
	float PJ[SSIZE_XY*MSIZE_SPH] = { 0.f };
	float JPJ[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float S_inv[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float Rm[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float S[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float Meas_var[MSIZE_SPH] = { 0.f };
	float det = 0.f;
	tracking_computeJacobian(inst->tracking_params->stateVectorType, inst->kalman_state.S_hat, J);
	/* Compute JPJ' = J(:,1:mSize) * obj.P_apriori(1:mSize,1:mSize) * J(:,1:mSize)' */
	tracking_matrixComputePJT(inst->kalman_state.P_hat, J, PJ);
	tracking_matrixMultiply(MSIZE_SPH, SSIZE_XY, MSIZE_SPH, J, PJ, JPJ);
	// TODO: const_tws_measurement_noise
	if (g_scanType == TWS_SCAN)
	{
		tracking_measurement_variance_cal(inst, const_tws_measurement_noise, Meas_var);
	}
	else
	{
		tracking_measurement_variance_cal(inst, const_tas_measurement_noise, Meas_var);
	}

	tracking_matrixSetDiag(MSIZE_SPH, Meas_var, Rm);
	tracking_matrixAdd(MSIZE_SPH, MSIZE_SPH, JPJ, Rm, S);
	/* Compute inverse of S */
	tracking_matrixInv(S, &det, S_inv);
	memcpy(inst->assoProperty.gC_inv, S_inv, sizeof(S_inv));
	inst->assoProperty.gC_det = det;
}

/**
*  @b Description
*  @n
*		This function computes partial derivatives of the cartesian vector
*
*  @param[in]  format
*		Format used
*  @param[in]  cart
*		Pointer to state vector (cartesian form)
*  @param[in] jac 
*		Pointer to jacobian parial derivatives) vector
*
*  \ingroup TRACKING_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/

void tracking_computeJacobian(sTracking_enum_state_vector_type format, float *cart, float *jac)
{    
    uint16_t ssize;
    float posx, posy, posz;
    float velx, vely, velz;
	float range, range2, range3;
    float rangeXY2, rangeXY;

	switch (format) {
		case TRACKING_STATE_VECTORS_2DV:
			/* cart = [posx posy velx vely] */
            ssize = 4;
            break;
        case TRACKING_STATE_VECTORS_2DA:
			/* cart = [posx posy velx vely accx accy] */
            ssize = 6;
            break;

		case TRACKING_STATE_VECTORS_3DV:
			/* cart = [posx posy posz velx vely velz] */
            ssize = 6;
            break;
        case TRACKING_STATE_VECTORS_3DA:
			/* cart = [posx posy posz velx vely velz accx accy accz] */
            ssize = 9;
            break;
		default:
			ssize = 6;

    }


	switch (format) {
        default:
		case TRACKING_STATE_VECTORS_2DV:	    		
            /* jacobian is 3x4 */
            // falls through
        case TRACKING_STATE_VECTORS_2DA:
			/* jacobian is 3x6 */

            posx = cart[0]; posy = cart[1];
            velx = cart[2]; vely = cart[3];

            range2 = (posx*posx + posy*posy);
	        range = sqrtf(range2);
	        range3 = range*range2;
			myMath_checkZero(&range);
			myMath_checkZero(&range2);
			myMath_checkZero(&range3);

            /* dR */
		    jac[0] = posx/range; /* dx */
		    jac[1] = posy/range; /* dy */

            /* dPhi */
		    jac[ssize+0] = -posy/range2; /* dx */
		    jac[ssize+1] = posx/range2; /* dy */

            /* dR' */
		    jac[2*ssize+0] = (posy*(velx*posy - vely*posx))/range3; /* dx */
			jac[2*ssize+1] = (posx*(vely*posx - velx*posy))/range3; /* dy */
			jac[2*ssize+2] = posx/range; /* dx' */
			jac[2*ssize+3] = posy/range; /* dy' */
			break;


        case TRACKING_STATE_VECTORS_3DV:
			/* jacobian is 4x6 */
            // falls through
		case TRACKING_STATE_VECTORS_3DA:
			/* jacobian is 4x9 */
            posx = cart[0]; posy = cart[1]; posz = cart[2];
            velx = cart[3]; vely = cart[4]; velz = cart[5];

	        range2 = posx*posx + posy*posy + posz*posz;
	        range = sqrtf(range2);

            rangeXY2 = posx*posx + posy*posy;
	        rangeXY = sqrtf(rangeXY2);
			myMath_checkZero(&range);
			myMath_checkZero(&range2);
			myMath_checkZero(&rangeXY);
			myMath_checkZero(&rangeXY2);

			/* dR */
		    jac[0] = posx/range;                /* dx */
			jac[1] = posy/range;                /* dy */
			jac[2] = posz/range;                /* dz */

            /* dAzim */
			jac[ssize+0] = -posy/rangeXY2;     /* dx */
			jac[ssize+1] = posx/rangeXY2;    /* dy */
			jac[ssize+2] = 0;                 /* dz */

            /* dElev */
			jac[2 * ssize + 0] = -posx * posz / (range2*rangeXY);  /* dx */
			jac[2 * ssize + 1] = -posy * posz / (range2*rangeXY);  /* dy */
			jac[2 * ssize + 2] = rangeXY / range2;               /* dz */

			/* dR' */
		    jac[3*ssize+0] = (posy*(velx*posy - vely*posx)+posz*(velx*posz-velz*posx))/(range2*range); /* dx */
			jac[3*ssize+1] = (posx*(vely*posx - velx*posy)+posz*(vely*posz-velz*posy))/(range2*range); /* dy */
			jac[3*ssize+2] = (posx*(velz*posx - velx*posz)+posy*(velz*posy-vely*posz))/(range2*range); /* dz */
			jac[3*ssize+3] = posx/range; /* dx' */
			jac[3*ssize+4] = posy/range; /* dy' */
			jac[3*ssize+5] = posz/range; /* dz' */
			break;
	}
}

/**
*  @b Description
*  @n
*		This function is used to unroll radial velocity from +/- rvMax form based on expectation velocity value
*
*  @param[in]  rvMax
*		Unambigious radial velocity
*  @param[in]  rvExp
*		Expected radial velocity value
*  @param[in] rvIn 
*		Measured radial velocity value
*
*  \ingroup TRACKING_ALG_UTILITY_FUNCTION
*
*  @retval
*      rvOut Unrolled radial velocity value
*/

float tracking_unrollRadialVelocity(float rvMax, float rvExp, float rvIn)
{
	float distance;
	uint16_t factor;
	float rvOut;
	myMath_checkZero(&rvMax);

	distance = rvExp - rvIn;
	if(distance >= 0) {
		/* going right */
		factor = (int)((distance+rvMax)/(2*rvMax));
		rvOut = rvIn + 2*rvMax*factor;
	} 
	else {
		/* going left */
		factor = (int)((rvMax-distance)/(2*rvMax));
		rvOut = rvIn - 2*rvMax*factor;
	}
	return rvOut;
}

// Ideal radial velocity for stationary at current target
float tracking_getIdealStationaryVelocity(float fX, float fY, float fRange, sTracking_platformInfo *platformInfo)
{
	float fYawRate = (fabsf(platformInfo->egoOmega)<0.5f*DEG2RAD)? 0.0f : platformInfo->egoOmega;
	float fSensorVx = platformInfo->egoLineVel_long - fYawRate*(fY + platformInfo->radarDist_lat);
	float fSensorVy = platformInfo->egoLineVel_lat + fYawRate*(fX + platformInfo->radarDist_long);
	float fVradIdealStat = 0.f;
	
	myMath_checkZero(&fRange);
	fVradIdealStat = -1.0f * ((fSensorVx*fX + fSensorVy*fY)/fRange);
	
	return fVradIdealStat;
}

void tracking_calculateTrackerAbsVelocity(sTracking_objectUnit *inst, sTracking_platformInfo *platformInfo)
{
	float fYawRate = 0.f;
	float dt = 0.f;
	float fVabsX = 0.f;
	float fVabsY = 0.f;
	dt = inst->tracking_params->dt;
	fYawRate = (fabsf(platformInfo->egoOmega)<1.f*DEG2RAD)? 0.0f : platformInfo->egoOmega;
	fVabsX = inst->kalman_state.S_hat[2] + platformInfo->egoLineVel_long \
			- fYawRate*(inst->kalman_state.S_hat[1]+platformInfo->radarDist_lat);
	fVabsY = inst->kalman_state.S_hat[3] + platformInfo->egoLineVel_lat \
		+ fYawRate*(inst->kalman_state.S_hat[0]+platformInfo->radarDist_long);
	inst->kalman_state.fVabsX = fVabsX;
	inst->kalman_state.fVabsY = fVabsY;

}

void tracking_calculateTrackerHeading(sTracking_objectUnit *inst, sTracking_platformInfo *platformInfo)
{
	//float fVabs_X = (inst->kalman_state.S_hat[2] + platformInfo->egoLineVel_long) - (platformInfo->egoOmega*(inst->kalman_state.S_hat[1]+platformInfo->radarDist_lat));
	//float fVabs_Y = inst->kalman_state.S_hat[3] + platformInfo->egoLineVel_lat + (platformInfo->egoOmega*(inst->kalman_state.S_hat[0]+platformInfo->radarDist_long));
	float fVabs_X = inst->kalman_state.fVabsX; 
	float fVabs_Y = inst->kalman_state.fVabsY; 
	float fHeadingTrackerNew = 0.f;
	float fAbsGt_speed = sqrtf(fVabs_X*fVabs_X + fVabs_Y*fVabs_Y);
	float fCurrentVradAbs = 0.f;
	float fAngleDiff = 0.f;
	float fFilterSpeed = 0.f;
	float fVradDiff_threshold = 0.f;
	//if(inst->eDynamicProperty==TRACKING_DYNAMIC_STATIC)
	//{
	//	inst->kalman_state.fTrackerHeading = 0.f;
	//}
	//else
	//{
	//	inst->kalman_state.fTrackerHeading = myMath_atan2f(fVabs_Y, fVabs_X);
	//}
	inst->kalman_state.fTrackerHeading = myMath_atan2f(fVabs_Y, fVabs_X);
	return;

	fCurrentVradAbs =inst->curTarget.unionSph.vector.doppler - inst->curTarget.fIdealStatVrad; // TODO: currentTarget如果是绝对静止的情况下的径向速度
	//TODO: consider yaw_rate for fVradDiff_threshold, omega[0,0.5] -> [0.5,2], egoVelLong[0, 10] -> [0,1]

	if(inst->eDynamicProperty == TRACKING_DYNAMIC_LEAVING) // leaving, absolutly
	{
		//fVradDiff_threshold = 0.f; //TODO: 内插(inst->S_hat[4]>-0.5 -> 0; <-3 -> 2; [-0.5. -3]内插), 目标地面加速度替换S[2]
		myMath_lineInterpolation(&fVradDiff_threshold, 0.f, 2.f, inst->kalman_state.S_hat[4], -3.f, -0.5f); // S[2]???
	}
	else if(inst->eDynamicProperty == TRACKING_DYNAMIC_ONCOMING) // onComing, absolutly
	{
		fVradDiff_threshold = 0.f; //TODO: 内插(inst->S_hat[4]<0.5 -> 0; >3 -> 2; [-0.5. -3]内插), 目标地面加速度替换S[2]
		myMath_lineInterpolation(&fVradDiff_threshold, 0.f, 2.f, inst->kalman_state.S_hat[4], 0.5f, 3.f); // S[2]???
	}
	else
	{
	}
	fVradDiff_threshold = (fVradDiff_threshold<4.5f)? fVradDiff_threshold : 4.5f;
	if(inst->objManagement.state >= TRACK_STATE_NEW_ACTIVE)
	{
		if(    (inst->eDynamicProperty		!= TRACKING_DYNAMIC_STOP) \
			&& (fabsf(fCurrentVradAbs)  >  fVradDiff_threshold) \
			//&& (inst->curTarget.quality<0.99) ) // TODO: quality is PDH0???
			&& (inst->curTarget.fPDH0<0.99) ) // TODO: quality is PDH0???
		{
			fHeadingTrackerNew = myMath_atan2f(fVabs_Y, fVabs_X);
			//fFilterSpeed = 0.02f; // TODO: (目标真实地面速度<0.5 -> 0.02; >5 -> 1; [0.5. 5]线性内插)
			myMath_lineInterpolation(&fFilterSpeed, 0.02f, 1.f, fAbsGt_speed, 0.5f, 5.f); // 
			fAngleDiff = inst->kalman_state.fTrackerHeading - fHeadingTrackerNew; // TODO: fTrackerHeading feature of tracker
			// TODO: rolling到[-PI,PI]
			myMath_unrolling(&fAngleDiff, -1.0f*M_PI, M_PI);
			inst->kalman_state.fTrackerHeading += fFilterSpeed * fAngleDiff;
		}
		else if(fabsf(fCurrentVradAbs)  < 1.0f)
		{
			inst->kalman_state.fTrackerHeading -= platformInfo->egoOmega * inst->tracking_params->dt * 0.5f;
		}
		else
		{
		}
		// TODO: rolling到[-PI,PI]
		myMath_unrolling(&inst->kalman_state.fTrackerHeading, -1.0f*M_PI, M_PI);
	}
	else
	{
			inst->kalman_state.fTrackerHeading = myMath_atan2f(fVabs_Y, fVabs_X);
	}
}

uint8_t tracking_isInsideWithGating(const float dx, const float dy, const float dz, const float dvrad, \
	sTracking_gateLimits *gate_limits)
{
	if(    (fabsf(dx)<=gate_limits->depth/2.0f)
		&& (fabsf(dy)<=gate_limits->width/2.0f)
		&& (fabsf(dz)<=gate_limits->height/2.0f)
		&& (fabsf(dvrad)<=gate_limits->vel/2.0f) ){
		return 1U;
	}
	else{
		return 0U;
	}
}

uint16_t tracking_getImportanceScore(float range, int flagMoving)
{
	uint16_t score = 0U;
	if(range>100.f){
		score = 1;
	}
	else{
		score = (uint16_t)(100.f-range);
	}
	if(flagMoving==((int)(TRACKING_DYNAMIC_STATIC))){
		score = (uint16_t)(((float)(score))*0.7f);
	}
	return  score;
}
void tracking_measurement_variance_cal(sTracking_objectUnit* pTrack, float* const_measurement_noise, float* Meas_var)
{
	float azimuthRad = pTrack->kalman_state.H_s.vector.azimuthRad;
	float range = pTrack->kalman_state.H_s.vector.range;
	float azimuthBoundary = 20.f*DEG2RAD;
	memcpy(Meas_var, const_measurement_noise, sizeof(float)*MSIZE_SPH);

}

