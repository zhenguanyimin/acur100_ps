#ifndef MEASUREMENT_PROCESS_H_
#define MEASUREMENT_PROCESS_H_
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include "../../../utilities/common_struct.h"


/**
 * @brief
 *   Measurement point
 *
 * @details
 *  The structure describes measurement point format
 */

#define MEASUREMENT_VECTOR_SIZE (sizeof(sMeasurement_Sph_vector)/sizeof(float))
#define MSIZE_SPH (MEASUREMENT_VECTOR_SIZE)
#define MSIZE_X (2)

#define CLUSTER_POINT_UNKNOWN 100
#define CLUSTER_POINT_VISITED 1
#define CLUSTER_POINT_LONELY 255 //<2^8=256
#define MIN_NUM_POINTS_IN_CLUSTER 2


 //constant parameter
#define COCH_BEAM_NUM_TWS (8U)
#define COCH_BEAM_NUM_TAS (4U)
#define RANGE_THRESHOLD_WITHIN (8.0f)//(2.0f),2*Rres
#define VELOCITY_THRESHOLD_WITHIN (2.0f)//(2.0f)
#define RANGE_THRESHOLD_BETWEEN (8.0f)//(2.0f),2*Rres
#define VELOCITY_THRESHOLD_BETWEEN (2.0f)//(2.0f)
#define RANGE_THRESHOLD_TAS (10.0f)//(2.0f)
#define VELOCITY_THRESHOLD_TAS (10.0f)//(2.0f)
#define K_AZI (0.078f)//方位比幅测角权值，4.4699*DEG2RAD；
#define K_ELE (0.2053f)//俯仰比幅测角权值，11.7608*DEG2RAD=0.2053,8.9398*DEG2RAD=0.156；
#define DIFFER_SUM_RATIO_THRESHOLD_AZI (0.4455f)//(0.5f)abs(差和比)阈值，在此区域内的进行测角
#define DIFFER_SUM_RATIO_THRESHOLD_ELE (0.4218f)//(0.5f)abs(差和比)阈值，在此区域内的进行测角(0.4455f)

#define CLUTTER_PRO_CASE (10U)


 /**
  * @brief
  *  TRACKING Measurement vector
  * @details
  *  The structure defines tracker measurement vector format
  */
typedef struct
{
	/**  @brief   Range, m */
	float range;
	/**  @brief   Azimuth, rad */
	float azimuthRad;
	/**  @brief   Elevation, rad */
	float pitchRad;
	/**  @brief   Radial velocity, m/s */
	float doppler;
} sMeasurement_Sph_vector;
typedef struct
{
	uint16_t detId;
	union {
		/**  @brief   Measurement vector */
		sMeasurement_Sph_vector vector;
		float array[MSIZE_SPH];
	};
	float x;
	float y;
	float z;
	/**  @brief   Range detection SNR, linear */
	float mag;
	float snr;
	float rcs;
	float sinAzim;
	float cosAzim;
	float disambigVelReliable;
	int16_t aziBeamSin;			//*32767, cache the result within beam, sin value of azimuth direction beam, uint = 1/32767
	int16_t eleBeamSin;			//*32767, cache the result within beam, sin value of elevation direction beam, uint = 1/32767
	uint16_t detectionNum;		// detection numbers in cluster
	uint8_t flag_moving;
	uint16_t detectionId[MAX_NUM_DETECTS];   // detection ID in cluster

} sMeasurement_SphPoint;

typedef struct sMeasConfigParam {
	float gating;
}sMeasConfigParam;

typedef struct sMeasOutput {
	uint8_t num; //number of clustering targets 
	sScanType scanType;				// scantype,0-TWS,1-TAS
	uint16_t tasTargetId;
	float timestamp;  //时间戳, in s
	uint8_t condenceDone;  //聚类是否完成：0未完成，1完成
	sMeasurement_SphPoint measurement[MAX_NUM_DETECTS];			// output of measurement process module
}sMeasOutput;

typedef struct sDetTrackFrame {
	int16_t aziBeamSin[COCH_BEAM_NUM_TAS];			//*32767, cache the result within beam, sin value of azimuth direction beam, uint = 1/32767
	int16_t eleBeamSin[COCH_BEAM_NUM_TAS];			//*32767, cache the result within beam, sin value of elevation direction beam, uint = 1/32767
	sDetectFrame detectFrame;
}sDetTrackFrame;

typedef struct sMultiBeamMeas {
	uint8_t numTws; //number of clustering targets in cache TWS
	uint8_t numTas; //number of clustering targets in cache TAS
	sMeasOutput multiBeam_tws[COCH_BEAM_NUM_TWS];
	sDetTrackFrame multiBeam_tas[COCH_BEAM_NUM_TAS];
}sMultiBeamMeas;

typedef struct{	
	sMeasConfigParam* measConfigParam;
	void* handle;
	sMeasOutput* measOutput;
	sMultiBeamMeas* multiBeamMeas; //波位信息存储区
}sMeasProcessInst;

//! @brief The detection list of tracking algorithm
typedef struct {
	uint8_t clusterID[MAX_NUM_DETECTS];
	uint8_t clusterNum;
	uint8_t lonelyNum;
}sClusterDetect;

//! @brief The detection list of tracking algorithm
typedef struct {
	uint8_t clusterID[MAX_NUM_DETECTS];
	uint8_t clusterNum;
	uint8_t lonelyNum;
	uint8_t clusterMeasNum;
	sMeasurement_SphPoint measurement[MAX_NUM_DETECTS];			// output of measurement process module
}sClusterMeasure;



void measurement_process(sAlgObjData* algObj);

void tas_measurement_process(sAlgObjData* algObj);
void tws_measurement_process(sAlgObjData* algObj);

void clutterProcessing(sDetectFrame* detInst, sDetectFrame* detClutProOut);
uint8_t isReflectPoint(float range, float doppler, float mag, uint8_t n_det, sDetectPoint* detPointStr, uint8_t nNum);
uint8_t isPowerSupplyPoint(float range, float doppler, float mag, uint8_t n_det, sDetectPoint* detPointStr, uint8_t nNum);
void clusteringWithinBeam_tws(sDetectFrame* detClutProOut, sMultiBeamMeas* cacheBeam, sMeasOutput* clusterObjIn);
//void clusteringBetweenBeam_tws(uint8_t calcAngleMode, sMeasOutput* cacheMeasBeam, sMeasOutput* clusterObjIn, sMeasOutput* measObjOut);
void clusteringBetweenBeam_tws(uint8_t calcAngleMode, sMeasOutput* cacheMeasBeam, sMeasOutput* measObjOut);
void detChoseWithinBeam_tas(sDetectFrame* detClutProOut, sTrackingTarget* tasTrackStr, sMultiBeamMeas* cacheBeam);
void detClusteringBetweenBeam_tas(sDetTrackFrame* TrackDetStr, sTrackingTarget* tasTrackStr, uint8_t* cacheBeamTarNum, sMeasOutput* measObjOut);
void clusteringFunInBeam(sClusterDetect* clusterDet, sDetectFrame* detClutProOut, sMeasOutput* clusterObjIn);
void clusteringFunBetweenBeams(uint8_t N_curObj, uint8_t N_cacheObj, uint8_t calcAngleMode, sClusterMeasure* clusterMeas, sMeasOutput* measObjOut);
void cluster_DBscan(sClusterMeasure* clusterMeas, uint8_t N);
uint8_t detTrack_DBscan(sDetectFrame* detClutProOut, sTrackingTarget* tasTrackStr, uint8_t* clusterID);
uint8_t detect_DBscan_findNeighbors(sDetectPoint* detPointStr, uint8_t n, uint8_t* neigh, uint8_t numPoints, uint8_t* visited_in, uint8_t* newCount);
uint8_t cluster_DBscan_findNeighbors(sMeasurement_SphPoint* measPointStr, uint8_t n, uint8_t* neigh, uint8_t numPoints, uint8_t* visited_between, uint8_t* newCount);
uint8_t detTrack_DBscan_findNeighbors(sDetectPoint* detPointStr, sTrackingTarget* tasTrackStr, uint8_t* neigh, uint8_t numPoints, uint8_t* visited_in, uint8_t* newCount);


#endif // !MEASUREMENT_PROCESS_H_
