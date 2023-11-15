
#include <math.h>
#include <float.h>
#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

#ifdef TRACKING_2D
#ifndef NANO_PROJECT
const float pInit[] = {0.5f, 0.5f, 0.04f, 0.04f, 0.25f, 0.25f};
#else
const float pInit[] = {0.5f, 0.5f, 0.5f, 0.5f, 1.f, 1.f};
#endif
const float spreadInit[] = {0.5f, 2.f*M_PI/180.f, 0.5f};
const float spreadMin[] = {0.25f, M_PI/180.f, 0.25f};

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
    float sinAngle, cosAngle;
    myMath_sincosd(v->azimuthRad*RAD2DEG,&sinAngle, &cosAngle);
    c->posX = v->range*cosAngle;
    c->posY = v->range*sinAngle;
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
		//limits->range = FLT_MAX;
		limits->range = 10.0f;
	else
	{
		limits->range = gate_limits->depth/2.f;
		if(limits->range>10.f)
		{
			limits->range = 10.f;
		}
	}

	if (gate_limits->width <= FLT_MIN)
		//limits->azimuthRad = FLT_MAX;
		limits->azimuthRad = 20.f*DEG2RAD;
	else
	{
		//limits->azimuthRad = atanf((gate_limits->width / 2) / range);
		limits->azimuthRad = myMath_atan2f(gate_limits->width / 2.f, range);
		if(limits->azimuthRad>60.f*DEG2RAD)
		{
			limits->azimuthRad = 60.f*DEG2RAD;
		}
		//limits->azimuth = gate_limits->width / 2; // atanf((gate_limits->width / 2) / range);
	}

	if(gate_limits->vel <= FLT_MIN)
		//limits->doppler = FLT_MAX;
		limits->doppler = 20.f;
	else
	{
		limits->doppler = gate_limits->vel/2.f;
		if(limits->doppler>20.f)
		{
			limits->doppler = 20.f;
		}
	}
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
    tDim[2] = mSpread[2];
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
    //  d = u1(1)^2 + u2(1)^2 - 2*u1(1)*u2(1)*cos(u1(2)-u2(2));
    float sinAngle, cosAngle;
    myMath_sincosd((p1->azimuthRad-p2->azimuthRad)*RAD2DEG,&sinAngle, &cosAngle);
    return p1->range*p1->range + p2->range*p2->range - 2*p1->range*p2->range*cosAngle;
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
*		This function is used to multiply two matrices.
*		First matrix P is of size 6x6, the second one is of the size 3x6.
*		The second matrix is being transposed first.
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  P
*		Matrix P
*  @param[in]  J
*		Matrix J
*  @param[out]  PJ
*		Matrix PJ = P(6,6) X J(3,6)T
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
	/* PJ[6x3] = P[6x6] x J[3x6]' */
	for (i = 0U, k = 0U; i < 18U; i+= 3U, k+=6U)
	{
		for (j = 0U, n = 0U; j < 3U; j++, n+=6U)
		{
            PJ[i + j] = (P[k + 0U] * J[n + 0U]) +
						(P[k + 1U] * J[n + 1U]) +
                        (P[k + 2U] * J[n + 2U]) +
                        (P[k + 3U] * J[n + 3U]) +
                        (P[k + 4U] * J[n + 4U]) +
                        (P[k + 5U] * J[n + 5U]);	
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply two matrices of the size 6x6. 
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[out]  C
*		Matrix C(6x6x) = A(6x6) X B(6x6)
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void tracking_matrixMultiply66(float *A, float *B, float *C)
{
	uint16_t i,j;
    for (i = 0U; i < 36U; i += 6U) 
	{
        for (j = 0U; j < 6U; j++) {
            C[i + j] = 	(A[i + 0U] * B[j +  0U]) +
						(A[i + 1U] * B[j +  6U]) +
                        (A[i + 2U] * B[j + 12U]) +
                        (A[i + 3U] * B[j + 18U]) +
                        (A[i + 4U] * B[j + 24U]) +
                        (A[i + 5U] * B[j + 30U]);	
		}
	}
}

/**
*  @b Description
*  @n
*		This function is used to multiply two matrices of size 6x6. 
*		Second Matrix is getting transposed first
*		Matrices are all real, single precision floating point.
*		Matrices are in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[in]  B
*		Matrix B
*  @param[out]  C
*		Matrix C(6x6) = A(6x6) X B(6x6)T
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void tracking_matrixMultiply66T(float *A, float *B, float *C)
{
	uint16_t i,j, k;
    for (i = 0; i < 36; i += 6) 
	{
        for (j = 0U, k = 0U; j < 6U; j++, k+=6U) {
            C[i + j] = 	(A[i + 0U] * B[k + 0U]) +
						(A[i + 1U] * B[k + 1U]) +
                        (A[i + 2U] * B[k + 2U]) +
                        (A[i + 3U] * B[k + 3U]) +
                        (A[i + 4U] * B[k + 4U]) +
                        (A[i + 5U] * B[k + 5U]);	
		}
	}
}

/**
*  @b Description
*  @n
*		This function computes the determinant of 3x3 matrix.
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
	*det = A[0] * (A[4]*A[8] - A[7]*A[5]) -
		A[1] * (A[3]*A[8] - A[5]*A[6]) +
		A[2] * (A[3]*A[7] - A[4]*A[6]);
}

/**
*  @b Description
*  @n
*		This function computes the inverse of 3x3 matrix.
*		Matrix is real, single precision floating point.
*		Matrix is in row-major order
*
*  @param[in]  A
*		Matrix A
*  @param[out]  det_out
*		det_out = determinant;
*  @param[out]  inv
*		inv = inverse(A);
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/

void tracking_matrixInv(const float *A, float *det_out, float *inv)
{
	float det;
	float invdet;

	det = A[0] * (A[4]*A[8] - A[7]*A[5]) -
		A[1] * (A[3]*A[8] - A[5]*A[6]) +
		A[2] * (A[3]*A[7] - A[4]*A[6]);

	myMath_checkZero(&det);
	invdet = 1 / det;

	inv[0] = (A[4] * A[8] - A[7] * A[5]) * invdet;
	inv[1] = (A[2] * A[7] - A[1] * A[8]) * invdet;
	inv[2] = (A[1] * A[5] - A[2] * A[4]) * invdet;
	inv[3] = (A[5] * A[6] - A[3] * A[8]) * invdet;
	inv[4] = (A[0] * A[8] - A[2] * A[6]) * invdet;
	inv[5] = (A[3] * A[2] - A[0] * A[5]) * invdet;
	inv[6] = (A[3] * A[7] - A[6] * A[4]) * invdet;
	inv[7] = (A[6] * A[1] - A[0] * A[7]) * invdet;
	inv[8] = (A[0] * A[4] - A[3] * A[1]) * invdet;

    *det_out = det;
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
*		This function computes Mahanalobis distance between vector v and distribution D.
*		Vector is of length 3. Vector represents the delta between distribution centroid and measurment vector.
*		Distribution is 3x3 matrix. Distribution represents the inverse of error covariance matrix.
*		Vector is real, single precision floating point array.
*		Matrix is real, single precision floating point array.
*		Matrix is in row-major order
*
*  @param[in]  v
*		Vector v
*  @param[in]  D
*		Matrix D
*  @param[out]  md
*		md is computed 3 dimensional mahanalobis distance
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
		v[0]*(v[0]*D[0]+v[1]*D[3]+v[2]*D[6])+
		v[1]*(v[0]*D[1]+v[1]*D[4]+v[2]*D[7])+
		v[2]*(v[0]*D[2]+v[1]*D[5]+v[2]*D[8]);	
}
/**
*  @b Description
*  @n
*		This function computes partial Mahanalobis distance between vector v and distribution D.
*		Vector is of length 3. Vector represents the delta between distribution centroid and measurment vector.
*		The last vector dimension is ignored
*		Distribution is 3x3 matrix. Distribution represents the inverse of error covariance matrix.
*		Vector is real, single precision floating point array.
*		Matrix is real, single precision floating point array.
*		Matrix is in row-major order
*
*  @param[in]  v
*		Vector v
*  @param[in]  D
*		Matrix D
*  @param[out]  mdp
*		md is computed 2 dimensional mahanalobis distance (ignoring third dimension, i.e v[2] = 0)
*		v[2]=0; mdp = v*D*v';
*
*  \ingroup tracking_ALG_MATH_FUNCTION
*
*  @retval
*      None
*/
void tracking_computeMahalanobisPartial(float *v, float *D, float *mdp)
{
	*mdp = 
		v[0]*(v[0]*D[0]+v[1]*D[3])+
		v[1]*(v[0]*D[1]+v[1]*D[4]);		
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
    if( (fabsf(p->azimuthRad - target->azimuthRad) < 2.f*M_PI/180.f) && (p->range > target->range))        
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
*		pointer to a 2D box object
*
*  \ingroup tracking_ALG_UTILITY_FUNCTION
*
*  @retval
*      1 if inside, 0 otherwise
*/
uint8_t tracking_isPointInsideBox(sTracking_cartesian_position *c, sTracking_boundaryBox *box)
{
    if( (c->posX > box->x1) && (c->posX < box->x2) &&
        (c->posY > box->y1) && (c->posY < box->y2) )
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
	sTracking_boundaryBox box;
	box.x1 = 0.0f;
	box.x2 = 10.0f;
	box.y1 = -5.0f;
	box.y2 = 5.0f;
	box.z1 = -100.f;
	box.z2 = 100.f;
	//float cart[4];
	//myMath_spherical2cartesian(mea, cart);
	if (stateParams_ptr == NULL || mea == NULL) {
		return 0;
	}
	memcpy(stateParams_ptr, &inst->tracking_params->advParams.stateParams, sizeof(sTracking_stateParams));
	if (!isInRoi(mea, &(const_roiArea->roiBox[0]))) {
		memcpy(stateParams_ptr, &inst->tracking_params->advParams.outOfRoiStateParams, sizeof(sTracking_stateParams));
	}
	else
	{
		if(inst->eDynamicProperty!=TRACKING_DYNAMIC_STATIC){
			stateParams_ptr->det2actThre = 5;
		}
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
#ifdef TA_PROJECT
	//if(fabsf(platformInfo->egoOmega*RAD2DEG)>2.f){
	//	gatingParams_ptr->limits.depth *= 2.f;
	//}
	//if(fabsf(platformInfo->egoLineAcc_lat)>0.5f){
	//	gatingParams_ptr->limits.height *= 2.f;
	//}
#endif
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

uint8_t isRearPoints(sMeasurement_SphPoint *point)
{
	if(    g_locMode== RIGHT_TARADAR \
		&& point->y<0 ){
		return 1U;
	}
	else if(    g_locMode== LEFT_TARADAR \
			 && point->y>0 ){
		return 1U;
	}
	return 0U;
}

uint8_t isTrailerWithVel(sMeasurement_SphPoint *point, sTracking_platformInfo* platformInfo)
{
	if(fabsf(fabsf(point->vector.doppler)-0.f)<0.1f){
		return 1U;
	}
	if(    (fabsf(platformInfo->egoOmega)>1.f*DEG2RAD) \
		/*SteeringAngle unit in ddegree/s*/
		|| (fabsf(platformInfo->egoSteeringAngle)>20.f) \
	){
		if(fabsf(fabsf(point->vector.doppler)-0.22f)<0.1f){
			return 1U;
		}
		if(fabsf(fabsf(point->vector.doppler)-0.66f)<0.1f){
			return 1U;
		}
	}
	return 0U;
}


uint8_t isTrailerPointVelZero(sMeasurement_SphPoint* point)
{
	float r = point->vector.range;
	float vel = fabs(point->vector.doppler);
	float angle = point->vector.azimuthRad * RAD2DEG;
	if (r < 20)
	{
		if (fabsf(vel) < 0.1f)
		{
			return 1U;
		}
	}
	return 0U;
}

uint8_t isTrailerPointVel022(sMeasurement_SphPoint* point)
{
	float r = point->vector.range;
	float vel = fabs(point->vector.doppler);
	float angle = point->vector.azimuthRad * RAD2DEG;

	if (g_locMode == RIGHT_TARADAR) 
	{
		if ((r < 20) && (angle < -84.0f))
		{
			if (fabsf(fabsf(vel) - 0.22f) < 0.1f)
			{
				return 1U;
			}
		}
		if ((r < 10) && (angle < -75.0f))
		{
			if (fabsf(fabsf(vel) - 0.22f) < 0.1f)
			{
				return 1U;
			}
		}
		if ((r < 5) && (angle < -50.0f))
		{
			if (fabsf(fabsf(vel) - 0.22f) < 0.1f)
			{
				return 1U;
			}
		}
	}
	else if (g_locMode == LEFT_TARADAR)
	{
		if ((r < 20) && (angle > 84.0f))
		{
			if (fabsf(fabsf(vel) - 0.22f) < 0.1f)
			{
				return 1U;
			}
		}
		if ((r < 10) && (angle > 75.0f))
		{
			if (fabsf(fabsf(vel) - 0.22f) < 0.1f)
			{
				return 1U;
			}
		}
		if ((r < 5) && (angle > 50.0f))
		{
			if (fabsf(fabsf(vel) - 0.22f) < 0.1f)
			{
				return 1U;
			}
		}
	}
	else
	{
		;
	}

	return 0U;
}



uint8_t isTrailerPointAngle(sMeasurement_SphPoint* point)
{
	float r = point->vector.range;
	float vel = fabsf(point->vector.doppler);
	float angle = point->vector.azimuthRad * RAD2DEG;
	if (r < 4.5 && vel < 0.5)
	{
		if (g_locMode == RIGHT_TARADAR)
		{
			if (point->flag_board == 0)
			{
				if (angle < -85)
				{
					return 1U;
				}
			}
			else if (point->flag_board == 1)
			{
				if (angle > 87)
				{
					return 1U;
				}
			}
			else
			{
				return 0U;
			}
		}
		else if (g_locMode == LEFT_TARADAR)
		{
			if (point->flag_board == 0)
			{
				if (angle < -87)
				{
					return 1U;
				}
			}
			else if (point->flag_board == 1)
			{
				if (angle > 85)
				{
					return 1U;
				}
			}
			else
			{
				return 0U;
			}
		}
		else
		{
			;
		}
		
	
	}
	return 0U;
}


uint8_t isTrailerPointRangeVel(sMeasurement_SphPoint* point)
{
	uint8_t flag_vel = 0, flag_xy = 0;
	if ((fabsf(fabsf(point->vector.doppler) - 0.f) < 0.1f) || (fabsf(fabsf(point->vector.doppler) - 0.22f) < 0.1f) \
		|| (fabsf(fabsf(point->vector.doppler) - 0.44f) < 0.1f) || (fabsf(fabsf(point->vector.doppler) - 0.66f) < 0.1f))
	{
		flag_vel = 1;
	}
	if ((point->x > 0) && (point->x < 0.75))
	{
		if (g_locMode == RIGHT_TARADAR)
		{
			if (((point->y > 0) && (point->y < 3)) || ((point->y > -5) && (point->y < 0)))
			{
				flag_xy = 1;
			}
		}
		else if(g_locMode == LEFT_TARADAR)
		{
			if (((point->y > 0) && (point->y < 5)) || ((point->y > -3) && (point->y < 0)))
			{
				flag_xy = 1;
			}
		}
		else
		{
			;
		}
	}

	if ((flag_vel == 1) && (flag_xy == 1))
	{
		return 1U;
	}
	else
	{
		return 0U;
	}
	return 0U;
}



#endif
