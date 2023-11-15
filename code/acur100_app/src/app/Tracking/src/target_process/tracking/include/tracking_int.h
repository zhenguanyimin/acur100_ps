
#ifndef TRACKING_INT_H__
#define TRACKING_INT_H__

#include "../include/tracking_common.h"

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif

#include <stdint.h>

#include "../include/tracking_listlib.h"
//#include "tracking_listlib.h"
//#include "tracking.h"
// define in tracking_utilities_2d.cpp or tracking_utilities_3d.cpp
extern const float pInit[];
extern const float spreadInit[];
extern const float spreadMin[];
extern const int pTriIndex[];
/**
 * @brief 
 *  3x3 Matrix Definition
 *
 * @details
 *  The structure describes the 3x3 matrix that can be viewed as an row-wise array OR addressed by individual element.
 */
typedef union {
	float a[9];
	struct
	{
		float e11; float e12; float e13;
		float e21; float e22; float e23;
		float e31; float e32; float e33;
	};
} MATRIX3x3; 

/**
 * @brief 
 *  4x4 Matrix Definition
 *
 * @details
 *  The structure describes the 4x4 matrix that can be viewed as an row-wise array OR addressed by individual element.
 */
typedef union {
	float a[16];
	struct
	{
		float e11; float e12; float e13; float e14;
		float e21; float e22; float e23; float e24;
		float e31; float e32; float e33; float e34;
		float e41; float e42; float e43; float e44;
	};
} MATRIX4x4;

/**
 * @brief 
 *  6x6 Matrix Definition
 *
 * @details
 *  The structure describes the 6x6 matrix that can be viewed as an row-wise array OR addressed by individual element.
 */
typedef union {
	float a[36];
	struct
	{
		float e11; float e12; float e13; float e14; float e15; float e16;
		float e21; float e22; float e23; float e24; float e25; float e26;
		float e31; float e32; float e33; float e34; float e35; float e36;
		float e41; float e42; float e43; float e44; float e45; float e46;
		float e51; float e52; float e53; float e54; float e55; float e56;
		float e61; float e62; float e63; float e64; float e65; float e66;
	};
} MATRIX6x6;

/**
 * @brief    Macros that define values to use for \ref TrackingParams::verbose when specifying the required verboseness
 */
#define VERBOSE_ERROR_INFO			0x00000001      /*!< Report Errors */
#define VERBOSE_WARNING_INFO		0x00000002      /*!< Report Warnings */
#define VERBOSE_DEBUG_INFO			0x00000004      /*!< Report Debuging information */
#define VERBOSE_MATRIX_INFO			0x00000008      /*!< Report Matrix math computations */
#define VERBOSE_UNROLL_INFO			0x00000010      /*!< Report velocity unrolling data */
#define VERBOSE_STATE_INFO			0x00000020      /*!< Report state transitions */
#define VERBOSE_ASSOSIATION_INFO	0x00000040      /*!< Report association data */
#define VERBOSE_GATEXY_INFO			0x00000080      /*!< Report gating in XY space */
#define VERBOSE_GATERA_INFO			0x00000100      /*!< Report gating in range/angle space */
#define VERBOSE_GATEG1_INFO			0x00000200      /*!< Report unitary gating */

void tracking_modulePredict(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_moduleAssociate(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_moduleAllocate(sTracking_moduleInstance *inst, sMeasOutput *measInfo);

void tracking_moduleUpdate(sTracking_moduleInstance *inst, sMeasOutput *point);
void tracking_moduleMerge(sTracking_moduleInstance *inst);
void tracking_moduleManager(sTracking_moduleInstance *inst);
void tracking_pointsClustering(void *handle, sMeasurement_SphPoint *point, uint16_t num, int32_t* pointClusterNum);
//int32_t tracking_DBscan(void* handle, sMeasurement_SphPoint* point, uint16_t num);
uint16_t tracking_DBscan_findNeighbors(sMeasurement_SphPoint *point, \
	uint16_t n, uint16_t *neigh, uint16_t numPoints, float epsilon2, \
	float epsilon, float weight, float vFactor, uint8_t *visited, uint16_t *newCount, \
	sTracking_platformInfo* platformInfo);

void *tracking_unitCreate(sTrackingParams *params, int32_t *errCode, sTracking_objectUnit *unitInst);
int tracking_unitStart(void *handle, sTracking_platformInfo *platformInfo, sMeasurement_SphPoint *measurement,\
	uint16_t* bestIndex, uint32_t wholeSpaceScanCycleCnt);
void tracking_unitPredict(void *handle, sTracking_platformInfo* platformInfo);
void tracking_tasPredict(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_twsPredict(sTracking_moduleInstance *inst);
void tracking_tasAssociate(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_twsAssociate(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_unitAssociate(void* handle, sTracking_platformInfo* platformInfo, sMeasOutput *measInfo, \
	float gating, uint16_t* bestIndex, uint32_t wholeSpaceScanCycleCnt, uint32_t lastWholeSpaceScanCycleCnt);
void tracking_tasUpdate(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_twsUpdate(sTracking_moduleInstance *inst, sMeasOutput *measInfo);
void tracking_unitUpdate(void *handle, sTracking_platformInfo *platformInfo, sMeasOutput *measInfo);
// Consider the ambiguity azimuth, given the multi-azimuth
void tracking_unitScore_multiAzim(void *handle, sMeasurement_SphPoint *point, float *bestScore, \
                      uint16_t *bestInd, uint8_t *isUnique, uint16_t num, uint8_t *isAssociatedActive, \
					  float *minMag_thre_array, float *mag_thre_array, float *minSNR_thre_array, float *snr_thre_array, \
					  float *rcs_thre_array, float *statVel_thre_array, void** hTrack, sTracking_platformInfo *platformInfo);
uint8_t tracking_unitScore_association( \
	sMeasurement_SphPoint* point, void* handle,\
	sTracking_gateLimits* gating_, sTracking_gateLimits* gating1_, sTracking_gateLimits* gating2_, \
	float azimuthRad, float sinAngle, float cosAngle, float doppler, float dx, \
	float dy, float dvrad, sTracking_platformInfo* platformInfo, uint8_t overtakeFlag);
uint8_t tracking_unitScore_pointsAssociation( \
	sMeasurement_SphPoint* point1, void* handle, sMeasurement_SphPoint* point0, \
	int ex_idx, sTracking_gateLimits* gating_, sTracking_gateLimits* gating1_, sTracking_gateLimits* gating2_, \
	float azimuthRad, float sinAngle, float cosAngle, float doppler, float dx, float dy, float dvrad);
uint8_t tracking_unit_cluster(sTracking_moduleInstance* inst, \
	sMeasurement_SphPoint* point, \
	uint16_t num, uint16_t n, sTracking_measurementSphUnion* mCenter, \
	sTracking_measurementSphUnion* mCurrent, \
	uint8_t* vIsClustered, uint16_t* vIndex, int* clu_num, uint8_t n_ex, \
	float* minMag_thre_array, float* minSNR_thre_array, float* maxVelThre_array);
void tracking_unitCurrentTarget(void *handle, sMeasurement_SphPoint *point, sMeasurement_Sph_vector *var,
                             uint16_t *pInd, uint8_t *isUnique, uint16_t num, sTracking_platformInfo *platformInfo);
void tracking_unitCurrentTargetWithWeightScale(void *handle, sMeasurement_SphPoint *point, sMeasurement_Sph_vector *var,
                             uint16_t *pInd, uint8_t *isUnique, uint16_t num, sTracking_platformInfo *platformInfo);
void tracking_unitEvent(void *handle, sTracking_platformInfo* platformInfo, sTrackingParams *tracking_params);
void tracking_tasManager(sTracking_moduleInstance *inst);
void setStateParams(sTracking_stateParams* stateParams, sTracking_objectUnit *inst);
void tracking_twsManager(sTracking_moduleInstance *inst);
void tracking_TasTwsTypeManagement(sTracking_moduleInstance *inst);
void tracking_TasTargetStateManagement(sTracking_objectUnit *inst);
void tracking_TwsTargetStateManagement(sTracking_objectUnit *inst);
void tracking_unit_dynamicUpdate(void* handle, sTracking_platformInfo* platformInfo);

//void tracking_unitClassification(void *handle, \

//int tracking_classifier_score(float* features, float* clsProb);
//void tracking_classifier_correction(sTracking_objectUnit *inst, float maxMagDiff);


void tracking_unitCalcCurrentTargetVelocityAndVar(void *handle, sTracking_platformInfo *platformInfo);
void tracking_calculateCurrentTargetRelativeVelocity(float *relVx, float *relVy, float fObjHeading, \
	sTracking_measurementSphUnion *unionSph, sTracking_measurementCartUnion *unionX, \
	sTracking_measurementCartUnion *unionY, float fVradIdealStationary, \
	float tracker_relVx, float tracker_relVy, float* fConf, uint8_t trackerMoving_flag, \
	uint8_t movingFlag, sTracking_platformInfo *platformInfo);
void tracking_unitCalcCurrentTargetVelocityAndVar(void *handle, sTracking_platformInfo *platformInfo);
void tracking_calculateCurrentTargetRelativeVelocityVar(sTracking_objectUnit *inst, sTracking_platformInfo *platformInfo);
void tracking_unitGetH(void *handle, float *H);

void tracking_velocityStateHandling(void *handle, sMeasurement_Sph_vector *v);
float tracking_unrollRadialVelocity(float rvMax, float rvExp, float rvIn);
void tracking_spherical2cartesian(sTracking_enum_state_vector_type format, float *sph, float *cart);
void tracking_cartesian2spherical(sTracking_enum_state_vector_type format, float *cart, float *sph);
void tracking_measurementCovCal(sTracking_objectUnit* inst);
void tracking_computeJacobian(sTracking_enum_state_vector_type format, float *cart, float *jac);
float tracking_getIdealStationaryVelocity(float fX, float fY, float fRange, sTracking_platformInfo *platformInfo);
void tracking_calculateTrackerHeading(sTracking_objectUnit *inst, sTracking_platformInfo *platformInfo);
void tracking_calculateTrackerAbsVelocity(sTracking_objectUnit *inst, sTracking_platformInfo *platformInfo);

void tracking_sph2cart(sMeasurement_Sph_vector *v, sTracking_cartesian_position *c);
float tracking_calcDistance(sMeasurement_Sph_vector *v1, sMeasurement_Sph_vector *v2);
void tracking_calcDim(float *mSpread, float R, float *tDim);
void tracking_calcMeasurementLimits(float range, sTracking_gateLimits *gate_limits, sMeasurement_Sph_vector *limits);
void tracking_calcMeasurementVar(sMeasurement_Sph_vector *v, sTracking_varParams *varP, sMeasurement_Sph_vector *var);

uint8_t tracking_isPointBehindTarget(sMeasurement_Sph_vector *v, sMeasurement_Sph_vector *target);
uint8_t tracking_isPointInsideBox(sTracking_cartesian_position *c, sTracking_boundaryBox *box);



uint8_t isInRoi(float *mea, sTracking_boundaryBox *box);
uint8_t getAllocationParams(float *mea, sTracking_allocationParams *allocParams_ptr, \
	sTracking_advancedParameters *advParams, int flagMoving, sTracking_platformInfo* platformInfo);
uint8_t getStateParams(float *mea, sTracking_stateParams *stateParams_ptr, sTracking_objectUnit *inst);
uint8_t getGatingParams(float *mea, sTracking_gatingParams *gatingParams_ptr, \
	sTracking_advancedParameters *advParams, sTracking_platformInfo* platformInfo);
uint8_t getExGatingParams(float *mea, sTracking_gatingParams *exGatingParams_ptr, \
	sTracking_advancedParameters *advParams, sTracking_platformInfo* platformInfo);
uint8_t getMagMinParams(float *mea, float *thre);
uint8_t getMagAssoParams(float *mea, float *thre);
uint8_t getSNRMinParams(float *mea, float *thre);
uint8_t getSNRAssoParams(float *mea, float *thre);
uint8_t getRcsAssoParams(float *mea, float *thre);

void tracking_matrixInit(uint16_t rows, uint16_t cols, float value, float *A);
void tracking_matrixEye(uint16_t size, float *A);
void tracking_matrixTriangleEye(uint16_t size, float *A);
void tracking_matrixSetDiag(uint16_t size, float *v, float *A);
void tracking_matrixGetDiag(uint16_t size, float *A, float *v);

void tracking_vectorInit(uint16_t size, float value, float *A);
void tracking_vectorAdd(uint16_t size, float *A, float *B, float *C);
void tracking_vectorSub(uint16_t size, float *A, float *B, float *C);
void tracking_vectorScalarMul(uint16_t size, float *A, float c, float *B);
void tracking_vectorScalarMulAcc(uint16_t size, float *A, float c, float *B);
void tracking_vectorFilter(uint16_t size, float *A, float alpha, float *B);

void tracking_matrixCovAcc(uint16_t size, float *A, float *v, float *mean);
void tracking_matrixCovNormalize(uint16_t size, float *A, uint16_t num);
void tracking_matrixCovFilter(uint16_t size, float *A, float *B, float alpha);

void tracking_matrixMultiply(uint16_t m1, uint16_t m2, uint16_t m3, float *A, float *B, float *C);
void tracking_matrixComputePJT(float *P, float *J, float *PJ);
void tracking_matrixUperTri2Matrix(float *P, float *P_);
void tracking_matrixMatrix2UperTri(float *P, float *P_);
void tracking_matrixTransposeMultiply(uint16_t rows, uint16_t m, uint16_t cols, float *A, float *B, float *C);
void tracking_matrixScalarMultiply(uint16_t m1, uint16_t m2, float *A, float c, float *B);
void tracking_matrixAdd(uint16_t m1, uint16_t m2, float *A, float *B, float *C);
void tracking_matrixSub(uint16_t m1, uint16_t m2, float *A, float *B, float *C);
void tracking_matrixInv(const float *A, float *det, float *Ainv);
void tracking_matrixInv_2x2(const float *A, float *det_out, float *Ainv);
void tracking_matrixMakeSymmetrical(uint16_t m, float *A, float *B);
void tracking_matrixCholesky(uint16_t dim, const float *A, float *G);
void tracking_computeMahalanobis(float *d, float *S, float *md);
void tracking_computeMahalanobisPartial(float *d, float *S, float *mdp);
uint8_t tracking_isInsideWithGating(const float dx, const float dy, const float dz, const float dvrad, \
	sTracking_gateLimits *gate_limits);
void getStaticThres(float* mea, float* vrad_thres, float* vx_thres, \
	float ego_velX, float ego_velY, float ego_omega);
uint8_t isInVeryNearRegion(float* mea);
uint8_t isInZeroDegreeRegion(float* mea);
uint16_t tracking_getImportanceScore(float range, int flagMoving);
void tracking_measurement_variance_cal(sTracking_objectUnit* pTrack, float* const_tws_measurement_noise, float* Meas_var);
uint8_t isRearPoints(sMeasurement_SphPoint *point);
uint8_t isTrailerWithVel(sMeasurement_SphPoint *point, sTracking_platformInfo* platformInfo);
//float poissonProb(int lamda, int k);

uint8_t isTrailerPointAngle(sMeasurement_SphPoint* point);
uint8_t isTrailerPointVelZero(sMeasurement_SphPoint* point);
uint8_t isTrailerPointRangeVel(sMeasurement_SphPoint* point);
uint8_t isTrailerPointVel022(sMeasurement_SphPoint* point);

void tracking_matrixPrint(uint16_t m1, uint16_t m2, float *A);
void tracking_matrixPrint_double(uint16_t m1, uint16_t m2, double *A);
void tracking_matrixPrint_int(uint16_t m1, uint16_t m2, int *A);
void tracking_matrixPrint2(uint16_t rows, uint16_t cols, float *A, float *B);

void tracking_cal_dt_transMatrix(void *handle);
void tracking_cal_FQ(sTracking_moduleInstance* inst, float dt);
#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif

#endif /* TRACKING_INT_LIB_H__ */
