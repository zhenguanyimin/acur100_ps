
#include <math.h>
#include <float.h>
#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

#ifdef TRACKING_3D

#define PI 3.14159265358979323846f

//const float pInit[] = { 25.f,25.f, 25.f, 16.f, 16.f, 25.f, 25.f, 25.f,9.f };
const float pInit[] = { 4.f,4.f, 16.f, 4.f, 4.f, 9.f, 9.f, 9.f,9.f };
const float spreadInit[] = {1.0f, 2*PI/180.f, 10*PI/180.f, 1.0f};
const float spreadMin[] = {0.25f, PI/180.f, 2*PI/180.f, 0.25f};
/**
*  @b Description
*  @n
*		This function is used to convert a vector from sherical to cartesian
*
*  @param[in]  v
*		Pointer to measurements (spheriacl form) vector
*  @param[out]  c
*		Pointer to state vector (cartesian form)
*
*  \ingroup tracking_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void tracking_sph2cart(sMeasurement_Sph_vector *v, sTracking_cartesian_position *c)
{
    float sinAzim, cosAzim;
    float sinElev, cosElev;

    myMath_sincosd(v->azimuthRad*RAD2DEG,&sinAzim, &cosAzim);
    myMath_sincosd(v->pitchRad*RAD2DEG,&sinElev, &cosElev);

    c->posX = v->range*cosElev*sinAzim;
    c->posY = v->range*cosElev*cosAzim;
    c->posZ = v->range*sinElev;
}
/**
*  @b Description
*  @n
*		This function computes measurment error limits based on current target position and configured target dimensions
*
*  @param[in]  range
*		target range
*  @param[in]  gate_limits
*		target dimentions limits
*  @param[out]  limits
*		Pointer to measurment error limits
*
*  \ingroup tracking_ALG_UTILITY_FUNCTION
*
*  @retval
*      None
*/
void tracking_calcMeasurementLimits(float range, sTracking_gateLimits *gate_limits, sMeasurement_Sph_vector *limits)
{

	if(gate_limits->depth <= FLT_MIN)
		limits->range = FLT_MAX;
	else
		limits->range = gate_limits->depth/2;

	if (gate_limits->width <= FLT_MIN)
		limits->azimuthRad = FLT_MAX;
	else
		//limits->azimuth = atanf((gate_limits->width / 2) / range);
		limits->azimuthRad = myMath_atan2f(gate_limits->width / 2.f, range);
		//limits->azimuth = gate_limits->width / 2; // atanf((gate_limits->width / 2) / range);

	if(gate_limits->height <= FLT_MIN)
		limits->pitchRad = FLT_MAX;
	else
		//limits->pitchRad = atanf((gate_limits->height/2)/range);
		limits->pitchRad = myMath_atan2f(gate_limits->height / 2.f, range);

	if(gate_limits->vel <= FLT_MIN)
		limits->doppler = FLT_MAX;
	else
		limits->doppler = gate_limits->vel/2;
}

void tracking_calcMeasurementVar(sMeasurement_Sph_vector *v, sTracking_varParams *varP, sMeasurement_Sph_vector *var)
{
    float azimErr;
    float elevErr;
    
    var->range = varP->depthStd*varP->depthStd;
    //azimErr = 2*atanf(0.5f*varP->widthStd/v->range);
    azimErr = 2*myMath_atan2f(0.5f*varP->widthStd, v->range);
    //elevErr = 2*atanf(0.5f*varP->heightStd/v->range);
    elevErr = 2*myMath_atan2f(0.5f*varP->heightStd, v->range);
    var->azimuthRad = azimErr*azimErr;
    var->pitchRad = elevErr*elevErr;
    var->doppler = varP->dopplerStd*varP->dopplerStd;
}

/**
*  @b Description
*  @n
*		This function computes target dimension estimations based on estimated measurement spread and centroid range
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  mSpread
*		Vector S
*  @param[in]  R
*		scalar Range
*  @param[out]  tDim
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_calcDim(float *mSpread, float R, float *tDim)
{
    tDim[0] = mSpread[0];
    tDim[1] = 2*R*tanf(mSpread[1]/2);
    tDim[2] = 2*R*tanf(mSpread[2]/2);
    tDim[3] = mSpread[3];
}

/**
*  @b Description
*  @n
*		This function is used to calculate a distance between two points defined in measurement coordinates
*
*  @param[in]  p1
*		Pointer to measurements (spheriacl form) vector
*  @param[in]  p2
*		Pointer to measurements (spheriacl form) vector
*
*  \ingroup tracking_ALG_UTILITY_FUNCTION
*
*  @retval
*      distance, m
*/
float tracking_calcDistance(sMeasurement_Sph_vector *p1, sMeasurement_Sph_vector *p2)
{
    //  d = u1(1)^2 + u2(1)^2 - 2*u1(1)*u2(1)*(cos(u1(3))*cos(u2(3))*cos(u1(2)-u2(2))+sin(u1(3))*sin(u2(3)));
    float sinAzim, cosAzim;
    float sinElev1, cosElev1;
    float sinElev2, cosElev2;

    myMath_sincosd((p1->azimuthRad-p2->azimuthRad)*RAD2DEG,&sinAzim, &cosAzim);
    myMath_sincosd(p1->pitchRad*RAD2DEG,&sinElev1, &cosElev1);
    myMath_sincosd(p2->pitchRad*RAD2DEG,&sinElev2, &cosElev2);

    return p1->range*p1->range + p2->range*p2->range - 2*p1->range*p2->range*(cosElev1*cosElev2*cosAzim+sinElev1*sinElev2);
}
void tracking_matrixUperTri2Matrix(float *P,float *P_)
{
	uint8_t num_P = 0;
	int i,j;
	for (i = 0; i < SSIZE_XY; i++)
	{
		for (j = 0; j < SSIZE_XY; j++)
		{
			if (j >= i)
			{
				P_[i*SSIZE_XY + j] = P[num_P];
				num_P++;
			}
			else
			{
				P_[i*SSIZE_XY + j] = P_[j*SSIZE_XY + i];
			}
		}
	}
}
void tracking_matrixMatrix2UperTri(float *P, float *P_)
{
	uint8_t num_P = 0;
	int i, j;
	num_P = 0;
	for (i = 0; i < SSIZE_XY; i++)
	{
		for (j = i; j < SSIZE_XY; j++)
		{
			P_[num_P] = P[i*SSIZE_XY + j];
			num_P++;
		}
	}
}
/**
*  @b Description
*  @n
*		This function computes Mahanalobis distance between vector v and distribution D.
*		Vector is of length 4. Vector represents the delta between distribution centroid and measurment vector.
*		Distribution is 4x4 matrix. Distribution represents the inverse of error covariance matrix.
*		Vector is real, single precision floating point array.
*		Matrix is real, single precision floating point array
*		Matrix is in row-major order
*
*  @param[in]  v
*		Vector v
*  @param[in]  D
*		Matrix D
*  @param[out]  md
*		md is computed 4 dimensional mahanalobis distance
*		md = v*D*v';
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_computeMahalanobis(float *v, float *D, float *md)
{
	*md = 
		v[0]*(v[0]*D[0]+v[1]*D[4]+v[2]*D[8]+v[3]*D[12])+
		v[1]*(v[0]*D[1]+v[1]*D[5]+v[2]*D[9]+v[3]*D[13])+
		v[2]*(v[0]*D[2]+v[1]*D[6]+v[2]*D[10]+v[3]*D[14])+
		v[3]*(v[0]*D[3]+v[1]*D[7]+v[2]*D[11]+v[3]*D[15]);
}
/**
*  @b Description
*  @n
*		This function computes partial Mahanalobis distance between vector v and distribution D.
*		Vector is of length 4. Vector represents the delta between distribution centroid and measurment vector.
*		The last dimension of vector is ignored (v[3] = 0).
*		Distribution is 4x4 matrix. Distribution represents the inverse of error covariance matrix.
*		Vector is real, single precision floating point array.
*		Matrix is real, single precision floating point array
*		Matrix is in row-major order
*
*  @param[in]  v
*		Vector v
*  @param[in]  D
*		Matrix D
*  @param[out]  mdp
*		mdp is computed 3 dimensional mahanalobis distance (ignoring forth dimension, i.e v[3] = 0)
*		v[3]=0; mdp = v*D*v';
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_computeMahalanobisPartial(float *v, float *D, float *mdp)
{
	*mdp = 
		v[0]*(v[0]*D[0]+v[1]*D[4]+v[2]*D[8])+
		v[1]*(v[0]*D[1]+v[1]*D[5]+v[2]*D[9])+
		v[2]*(v[0]*D[2]+v[1]*D[6]+v[2]*D[10]);
}
/**
*  @b Description
*  @n
*		This function is used to multiply two matrices.
*		First matrix P is of size 9x9, the second one is of the size 4x9.
*		The second matrix is being transposed first.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  P
*		Matrix P
*  @param[in]  J
*		Matrix J
*  @param[out]  PJ
*		Matrix PJ = P(9,9) X J(4,9)T
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void tracking_matrixComputePJT(float *P, float *J, float *PJ)
{
	/* We compute PJ' first because it is re-used in Kalman Gain calculations */
	uint16_t i,j, k, n;
	/* PJ[9x4] = P[9x9] x J[4x9]' */
	for (i = 0U, k = 0U; i < 36U; i+= 4U, k+=9U)
	{
		for (j = 0U, n = 0U; j < 4U; j++, n+=9U)
		{
            PJ[i + j] = (P[k + 0U] * J[n + 0U]) +
						(P[k + 1U] * J[n + 1U]) +
                        (P[k + 2U] * J[n + 2U]) +
                        (P[k + 3U] * J[n + 3U]) +
                        (P[k + 4U] * J[n + 4U]) +
                        (P[k + 5U] * J[n + 5U]) +
                        (P[k + 6U] * J[n + 6U]) +
                        (P[k + 7U] * J[n + 7U]) +
                        (P[k + 8U] * J[n + 8U]);	
		}
	}
}
/**
*  @b Description
*  @n
*		This function computes the inverse of 4x4 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[out]  det_out
*		det_out = determinant;
*  @param[out]  B
*		B = inverse(A);
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void tracking_matrixInv(const float *A, float *det_out, float *B)
{
    float inv[16];
    float det;
    float invdet;
    int i;

    inv[0] = A[5]  * A[10] * A[15] - 
             A[5]  * A[11] * A[14] - 
             A[9]  * A[6]  * A[15] + 
             A[9]  * A[7]  * A[14] +
             A[13] * A[6]  * A[11] - 
             A[13] * A[7]  * A[10];

    inv[4] = -A[4]  * A[10] * A[15] + 
              A[4]  * A[11] * A[14] + 
              A[8]  * A[6]  * A[15] - 
              A[8]  * A[7]  * A[14] - 
              A[12] * A[6]  * A[11] + 
              A[12] * A[7]  * A[10];

    inv[8] = A[4]  * A[9] * A[15] - 
             A[4]  * A[11] * A[13] - 
             A[8]  * A[5] * A[15] + 
             A[8]  * A[7] * A[13] + 
             A[12] * A[5] * A[11] - 
             A[12] * A[7] * A[9];

    inv[12] = -A[4]  * A[9] * A[14] + 
               A[4]  * A[10] * A[13] +
               A[8]  * A[5] * A[14] - 
               A[8]  * A[6] * A[13] - 
               A[12] * A[5] * A[10] + 
               A[12] * A[6] * A[9];

    inv[1] = -A[1]  * A[10] * A[15] + 
              A[1]  * A[11] * A[14] + 
              A[9]  * A[2] * A[15] - 
              A[9]  * A[3] * A[14] - 
              A[13] * A[2] * A[11] + 
              A[13] * A[3] * A[10];

    inv[5] = A[0]  * A[10] * A[15] - 
             A[0]  * A[11] * A[14] - 
             A[8]  * A[2] * A[15] + 
             A[8]  * A[3] * A[14] + 
             A[12] * A[2] * A[11] - 
             A[12] * A[3] * A[10];

    inv[9] = -A[0]  * A[9] * A[15] + 
              A[0]  * A[11] * A[13] + 
              A[8]  * A[1] * A[15] - 
              A[8]  * A[3] * A[13] - 
              A[12] * A[1] * A[11] + 
              A[12] * A[3] * A[9];

    inv[13] = A[0]  * A[9] * A[14] - 
              A[0]  * A[10] * A[13] - 
              A[8]  * A[1] * A[14] + 
              A[8]  * A[2] * A[13] + 
              A[12] * A[1] * A[10] - 
              A[12] * A[2] * A[9];

    inv[2] = A[1]  * A[6] * A[15] - 
             A[1]  * A[7] * A[14] - 
             A[5]  * A[2] * A[15] + 
             A[5]  * A[3] * A[14] + 
             A[13] * A[2] * A[7] - 
             A[13] * A[3] * A[6];

    inv[6] = -A[0]  * A[6] * A[15] + 
              A[0]  * A[7] * A[14] + 
              A[4]  * A[2] * A[15] - 
              A[4]  * A[3] * A[14] - 
              A[12] * A[2] * A[7] + 
              A[12] * A[3] * A[6];

    inv[10] = A[0]  * A[5] * A[15] - 
              A[0]  * A[7] * A[13] - 
              A[4]  * A[1] * A[15] + 
              A[4]  * A[3] * A[13] + 
              A[12] * A[1] * A[7] - 
              A[12] * A[3] * A[5];

    inv[14] = -A[0]  * A[5] * A[14] + 
               A[0]  * A[6] * A[13] + 
               A[4]  * A[1] * A[14] - 
               A[4]  * A[2] * A[13] - 
               A[12] * A[1] * A[6] + 
               A[12] * A[2] * A[5];

    inv[3] = -A[1] * A[6] * A[11] + 
              A[1] * A[7] * A[10] + 
              A[5] * A[2] * A[11] - 
              A[5] * A[3] * A[10] - 
              A[9] * A[2] * A[7] + 
              A[9] * A[3] * A[6];

    inv[7] = A[0] * A[6] * A[11] - 
             A[0] * A[7] * A[10] - 
             A[4] * A[2] * A[11] + 
             A[4] * A[3] * A[10] + 
             A[8] * A[2] * A[7] - 
             A[8] * A[3] * A[6];

    inv[11] = -A[0] * A[5] * A[11] + 
               A[0] * A[7] * A[9] + 
               A[4] * A[1] * A[11] - 
               A[4] * A[3] * A[9] - 
               A[8] * A[1] * A[7] + 
               A[8] * A[3] * A[5];

    inv[15] = A[0] * A[5] * A[10] - 
              A[0] * A[6] * A[9] - 
              A[4] * A[1] * A[10] + 
              A[4] * A[2] * A[9] + 
              A[8] * A[1] * A[6] - 
              A[8] * A[2] * A[5];

    det = A[0] * inv[0] + A[1] * inv[4] + A[2] * inv[8] + A[3] * inv[12];

    invdet = 1.0f / det;

    for (i = 0; i < 16; i++)
        B[i] = inv[i] * invdet;
    
    *det_out = det;
}

/**
*  @b Description
*  @n
*		This function computes the determinant of 4x4 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[out]  det
*		det = det(A);
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_matrixDet(float *A, float *det)
{
    float inv[4];

    inv[0] = A[5]  * A[10] * A[15] - 
             A[5]  * A[11] * A[14] - 
             A[9]  * A[6]  * A[15] + 
             A[9]  * A[7]  * A[14] +
             A[13] * A[6]  * A[11] - 
             A[13] * A[7]  * A[10];

    inv[1] = -A[4]  * A[10] * A[15] + 
              A[4]  * A[11] * A[14] + 
              A[8]  * A[6]  * A[15] - 
              A[8]  * A[7]  * A[14] - 
              A[12] * A[6]  * A[11] + 
              A[12] * A[7]  * A[10];

    inv[2] = A[4]  * A[9] * A[15] - 
             A[4]  * A[11] * A[13] - 
             A[8]  * A[5] * A[15] + 
             A[8]  * A[7] * A[13] + 
             A[12] * A[5] * A[11] - 
             A[12] * A[7] * A[9];

    inv[3] = -A[4]  * A[9] * A[14] + 
               A[4]  * A[10] * A[13] +
               A[8]  * A[5] * A[14] - 
               A[8]  * A[6] * A[13] - 
               A[12] * A[5] * A[10] + 
               A[12] * A[6] * A[9];

    *det = A[0] * inv[0] + A[1] * inv[1] + A[2] * inv[2] + A[3] * inv[3];
}
void tracking_matrixInv_2x2(const float *A, float *det_out, float *inv)
{
	float det;
	float invdet;
	det = A[0] * A[3] - A[1] * A[2];
	myMath_checkZero(&det);
	invdet = 1.f / det;
	inv[0] = A[3] * invdet;
	inv[1] = -1.0f * A[1] * invdet;
	inv[2] = -1.0f * A[2] * invdet;
	inv[3] = A[0] * invdet;
    *det_out = det;
}
/**
*  @b Description
*  @n
*		This function checks whether measurement point P is geometrically behind the target
*
*  @param[in]  p
*		Pointer to measurements (spheriacl form) vector
*  @param[in]  target
*		Pointer to measurements (spheriacl form) vector, representing target centroid
*  @retval
*      1 if behind, 0 is otherwise
*/
uint8_t tracking_isPointBehindTarget(sMeasurement_Sph_vector *p, sMeasurement_Sph_vector *target)
{
    if( (fabsf(p->azimuthRad - target->azimuthRad) < 2*PI/180) &&
        (fabsf(p->pitchRad - target->pitchRad) < 2*PI/180) &&
        (p->range > target->range) )        
        return 1U;
    else
        return 0;
}

/**
*  @b Description
*  @n
*		This function checks whether the point is inside the box boundary or not
*
*  @param[in]  c
*		pointer to a position in cartesian space
*  @param[in] box
*		pointer to a 3D box object
*
*  \ingroup tracking_ALG_UTILITY_FUNCTION
*
*  @retval
*      1 if inside, 0 otherwise
*/
uint8_t tracking_isPointInsideBox(sTracking_cartesian_position *c, sTracking_boundaryBox *box)
{
    if( (c->posX > box->x1) && (c->posX < box->x2) && 
        (c->posY > box->y1) && (c->posY < box->y2) && 
        (c->posZ > box->z1) && (c->posZ < box->z2) )
        return 1U;
    else
        return 0;
}

// Check wheter the point is inside the roi or not in measurement coordination
uint8_t isInRoi(float *mea, sTracking_boundaryBox *box)
{
	sTracking_cartesian_position cart;
	sTracking_measurementSphUnion v;
	memcpy(v.array, mea, sizeof(float)*MEASUREMENT_VECTOR_SIZE);
	tracking_sph2cart(&v.vector, &cart);
	if(tracking_isPointInsideBox(&cart, box)){
		return 1U;
	}
	else {
		return 0;
	}
}

// Get allocation parameters based on roi
uint8_t getAllocationParams(float *mea, sTracking_allocationParams *allocParams_ptr, \
	sTracking_advancedParameters *advParams, int flagMoving, sTracking_platformInfo* platformInfo)
{
	sTracking_boundaryBox box;
	box.x1 = 0.0f;
	box.x2 = 10.0f;
	box.y1 = -10.0f;
	box.y2 = 10.0f;
	box.z1 = -100.f;
	box.z2 = 100.f;
	if (allocParams_ptr == NULL || mea == NULL) {
		return 0;
	}
	memcpy(allocParams_ptr, &advParams->allocationParams, sizeof(sTracking_allocationParams));
	if (!isInRoi(mea, &(const_roiArea->roiBox[0]))) {
		memcpy(allocParams_ptr, &advParams->outOfRoiAllocationParams, sizeof(sTracking_allocationParams));
	}
	else{
#ifdef TA_PROJECT
		if(    fabsf(platformInfo->egoLineVel_lat)<0.1f \
			&& fabsf(platformInfo->egoLineVel_long)<0.1f ){
			allocParams_ptr->pointsThre = 1;
		}
#endif
	}
	if(flagMoving==0){
		allocParams_ptr->pointsThre = 1;
	}
	else if(isInRoi(mea, &box)){
		allocParams_ptr->pointsThre = 1;		//2
	}
	return 1U;
}

// Get StateParams based on roi
//uint8_t getStateParams(float *mea, sTracking_stateParams *stateParams_ptr, sTracking_advancedParameters *advParams, \
//	uint8_t hasExistingTrackerOfNearRegion, sTracking_enum_state state)
uint8_t getStateParams(float *mea, sTracking_stateParams *stateParams_ptr, sTracking_objectUnit *inst)
{
;
	if (stateParams_ptr == NULL || mea == NULL) {
		return 0;
	}
	if (g_scanType == TAS_SCAN)
	{
		memcpy(stateParams_ptr, &inst->tracking_params->advParams.tasStateParams, sizeof(sTracking_stateParams));
	}
	else
	{
		memcpy(stateParams_ptr, &inst->tracking_params->advParams.twsStateParams, sizeof(sTracking_stateParams));
	}
	
	return 1U;
}

// Get gatingParams based on roi
uint8_t getGatingParams(float *mea, sTracking_gatingParams *gatingParams_ptr, \
	sTracking_advancedParameters *advParams, sTracking_platformInfo* platformInfo)
{
	if (gatingParams_ptr == NULL || mea == NULL) {
		return 0;
	}
	memcpy(gatingParams_ptr, &advParams->gatingParams, sizeof(sTracking_gatingParams));
	if (!isInRoi(mea, &(const_roiArea->roiBox[0]))) {
		memcpy(gatingParams_ptr, &advParams->outOfRoiGatingParams, sizeof(sTracking_gatingParams));
	}
	return 1U;
}

// Get ex_gatingParams based on roi
uint8_t getExGatingParams(float *mea, sTracking_gatingParams *exGatingParams_ptr, \
	sTracking_advancedParameters *advParams, sTracking_platformInfo* platformInfo)
{
	if (exGatingParams_ptr == NULL || mea == NULL) {
		return 0;
	}
	memcpy(exGatingParams_ptr, &advParams->exGatingParams, sizeof(sTracking_gatingParams));
	if (!isInRoi(mea, &(const_roiArea->roiBox[0]))) {
		memcpy(exGatingParams_ptr, &advParams->exOutOfRoiGatingParams, sizeof(sTracking_gatingParams));
	}
#ifdef TA_PROJECT
	if(fabsf(platformInfo->egoOmega*RAD2DEG)>2.f){
		exGatingParams_ptr->limits.depth *= 1.5f;
	}
	if(fabsf(platformInfo->egoLineAcc_lat)>0.5f){
		exGatingParams_ptr->limits.height *= 1.5f;
	}
#endif
	return 1U;
}

uint8_t getMagMinParams(float *mea, float *thre)
{
	int i = 0;
	if (thre == NULL || mea == NULL) {
		return 0;
	}
	for(i=0; i<const_roiArea->numNoiseBoxes; i++){
		if (isInRoi(mea, &(const_roiArea->noiseBox[i]))) {
			*thre = const_roiArea->mag_min_thres[i] - 0.1f* mea[0];
			return 1U;
		}
	}
	*thre = 0;
	return 0;
}

uint8_t getMagAssoParams(float *mea, float *thre)
{
	int i = 0;
	if (thre == NULL || mea == NULL) {
		return 0;
	}
	for(i=0; i<const_roiArea->numNoiseBoxes; i++){
		if (isInRoi(mea, &(const_roiArea->noiseBox[i]))) {
			*thre = const_roiArea->mag_asso_inROIThres[i];
			return 1U;
		}
	}
	*thre = const_roiArea->mag_asso_outROIThres;
	return 1U;
}

uint8_t getSNRMinParams(float *mea, float *thre)
{
	int i = 0;
	if (thre == NULL || mea == NULL) {
		return 0;
	}
	for(i=0; i<const_roiArea->numNoiseBoxes; i++){
		if (isInRoi(mea, &(const_roiArea->noiseBox[i]))) {
			*thre = const_roiArea->snr_min_thres[i] - 0.1f* mea[0];
			return 1U;
		}
	}
	*thre = 0;
	return 0;
}

uint8_t getSNRAssoParams(float *mea, float *thre)
{
	int i = 0;
	if (thre == NULL || mea == NULL) {
		return 0;
	}
	for(i=0; i<const_roiArea->numNoiseBoxes; i++){
		if (isInRoi(mea, &(const_roiArea->noiseBox[i]))) {
			*thre = const_roiArea->snr_asso_inROIThres[i];
			return 1U;
		}
	}
	*thre = const_roiArea->snr_asso_outROIThres;
	return 1U;
}

uint8_t getRcsAssoParams(float *mea, float *thre)
{
	int i = 0;
	if (thre == NULL || mea == NULL) {
		return 0;
	}
	for(i=0; i<const_roiArea->numNoiseBoxes; i++){
		if (isInRoi(mea, &(const_roiArea->noiseBox[i]))) {
			*thre = const_roiArea->rcs_asso_inROIThres[i];
			return 1U;
		}
	}
	*thre = const_roiArea->rcs_asso_outROIThres;
	return 1U;
}

uint8_t isInVeryNearRegion(float* mea)
{
	sTracking_boundaryBox box;
	box.x1 = 0.0f;
	box.x2 = 4.0f;
	box.y1 = -5.0f;
	box.y2 = 5.0f;
	box.z1 = -100.f;
	box.z2 = 100.f;
	return isInRoi(mea, &box);
}

uint8_t isInZeroDegreeRegion(float* mea)
{
	sTracking_boundaryBox box;
	box.x1 = 0.0f;
	box.x2 = 10.0f;
	box.y1 = -3.0f;
	box.y2 = 3.0f;
	box.z1 = -100.f;
	box.z2 = 100.f;
	return isInRoi(mea, &box);
}

void getStaticThres(float* mea, float* vrad_thres, float* vx_thres, \
	float ego_velX, float ego_velY, float ego_omega)
{
	sTracking_boundaryBox box;
	sTracking_boundaryBox box1;
	box.x1 = 0.0f;
	box.x2 = 4.0f;
	box.y1 = -5.0f;
	box.y2 = 5.0f;
	box.z1 = -100.f;
	box.z2 = 100.f;
	box1.x1 = 0.0f;
	box1.x2 = 4.0f;
	box1.y1 = -70.0f;
	box1.y2 = 70.0f;
	box1.z1 = -100.f;
	box1.z2 = 100.f;
	
	if(fabsf(ego_velX)<0.1f && fabsf(ego_velY)<0.1f){
		*vrad_thres = 0.5f;
		*vx_thres = 0.25f;
	}
	else if(fabsf(ego_omega*RAD2DEG)>2.f&&fabsf(ego_omega*RAD2DEG) <= 4.f){
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD*1.5f;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY*1.5f;
	}
	else if (fabsf(ego_omega*RAD2DEG) > 4.f) {
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD * 2.f;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY * 2.f;
	}
	//else if(fabsf(mea[2]*RAD2DEG)>50.f && isInRoi(mea, &box1)){
	//	*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD*0.7f;
	//	*vx_thres = TRACKING_MIN_STATIC_VELOCITY*0.7f;
	//}
	else if(isInRoi(mea, &box) && fabsf(mea[1]*RAD2DEG)<10.f){
		//*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD*2.f; //0.5f;
		//*vx_thres = TRACKING_MIN_STATIC_VELOCITY*2.f; //0.5f;
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD*0.5f;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY*0.5f;
	}
#ifdef HUHANG
	else if(fabsf(ego_velY)>8.3f){
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD*2.0f;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY*2.0f;
	}
	else if (fabsf(ego_velY) > 4.0f) {
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD * 1.5f;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY * 1.5f;
	}
#else
	else if (fabsf(ego_velY) > 15.f) {
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD * 1.5f;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY * 1.5f;
	}
#endif
	else{
		*vrad_thres = TRACKING_MIN_STATIC_VELOCITY_RAD;
		*vx_thres = TRACKING_MIN_STATIC_VELOCITY;
	}
}





#endif
