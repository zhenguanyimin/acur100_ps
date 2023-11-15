#ifndef EMBED_API_H
#define EMBED_API_H

#include <stdint.h>
#include "common_define.h"

#define ONE6FORMAT_SCALE (64.f) // 2^6
#define ONE7FORMAT_SCALE (128.f) // 2^7
#define ONE8FORMAT_SCALE (256.f) // 2^8
#define ONE14FORMAT_SCALE (16384.f) // 2^14
#define ONE15FORMAT_SCALE (32767.f) // 2^15-1
#define INV_ONE6FORMAT_SCALE (0.015625f) // 1/2^6
#define INV_ONE7FORMAT_SCALE (0.0078125f) // 1/2^7
#define INV_ONE8FORMAT_SCALE (0.00390625f) // 1/2^8
#define INV_ONE14FORMAT_SCALE (0.000061147f) // 1/2^14
#define INV_ONE15FORMAT_SCALE (0.0000305185f) // 1/32767,0.0000305185094759

#define PLATFORM_FORMAT_SCALE (8.3819e-08f)  // 90/2^30

#define MAX_NUM_POINTS_SHORTTERM_PREDICTIOIN 5
#define MAX_NUM_DETECTS 128
#define PT_NUM_ADD_TO_GATHER  6
#define DATA_RAND_MAX  10
#define MAX_NUM_TRAJS 30


//#define MAX_NUM_MAINOBJS 5
#define NUM_EX_DOPPLER 2
#define TRACKING_CLS_NUM (CLASS_INIT+1)
#define TRACKING_SLEFLANEPROB_NUM 5
#define TRACKING_SLEFLANE_NUM 2

#ifndef _WINDOWS
#pragma pack(1)
#endif
typedef struct {
	uint8_t classification; // 
	uint8_t classifyProb;		// 2^6
	uint8_t cohesionOkFlag; //cohesion has been finished  flag, 0:not; 1:yes
	uint8_t cohesionPntNum; //number of cohesion point
	uint8_t cohesionBeamNum; //number of cohesion beam num
}sDetProperty;

// the measurement data from mmwave in polar coordination
typedef struct{
	uint16_t id; // the id of detection
	int16_t azimuth; //deg, 2^6
	uint32_t range; // m, 2^6
	int16_t elevation; // deg, the most likely azimuth based on azimuth ambiguity, 2^6
	int16_t velocity;  //m/s, 2^6
	int16_t dopplerChn; // m, 
	uint16_t mag; // 2^6
	uint16_t objConfidence; // 2^6
	uint8_t ambiguous; // 
	sDetProperty detProperty; // the information of detection property
	uint16_t aziBeamID;			// azimuth direction beam index 0~29, for -58:4:58(degree)
	uint16_t eleBeamID;			// elevation direction beam index 0~3, for -15:10:15(degree)
	uint16_t reserve[4];
}sDetectPoint;

//! @brief The detection list of tracking algorithm
typedef struct {
	uint32_t frameID;
	uint32_t timestamp;            /**< timestamp, in ms*/
	uint8_t detectObjNum;                        /**< number of detect object */
	uint16_t detectObjByte;                       /**< byte number of each detect object */
	uint16_t trackTwsTasFlag; // the type of scan, 0-tws, 1-tas
    sDetectPoint detPointsList[MAX_NUM_DETECTS];           /**< information for each detection*/	
}sDetectFrame;

//! @brief A 3D trajectory information in cartesian coordination
typedef struct{
	int32_t x; // In 2^6, m
	int32_t y; // In 2^6, m
	int32_t z;
	int16_t vx; // In 2^6,  m/s
	int16_t vy; // In 2^6,  m/s
	int16_t vz; // In 2^6,  m/s
	int16_t ax; // In 2^6,  m/s2
	int16_t ay; // In 2^6,  m/s2
	int16_t az; // In 2^6,  m/s2
} sTrackingPoint;


//! @brief The trajectory unit
typedef struct {
	uint16_t id;
	int16_t	azimuth;
	uint32_t range;
	int16_t	elevation;
	int16_t	velocity;
	int16_t dopplerChn;
	uint16_t mag;
	uint8_t	ambiguous;
	uint8_t	classification;
	uint8_t	classifyProb;
	uint8_t	probOfExisting;
	int16_t	absVel;
	int16_t	orientationAngle;
	uint16_t alive;
	uint16_t twsTasFlag;
	sTrackingPoint traj;
	uint16_t variance[9];
	uint8_t	stateType;
	uint8_t	motionType;
	uint16_t forcastFrameNum;
	uint16_t associationNum;
	uint32_t assocBit0;
	uint32_t assocBit1;
	uint16_t reserve[4];
} sTrackingTarget;
#ifndef _WINDOWS
#pragma pack(0)
#endif

//! @brief The trajectory list of tracking algorithm
typedef struct {
	uint32_t frameID;
	uint32_t timestamp;           /**< timestamp, in s*/
	uint16_t trackObjNum;                        /**< number of traj */
	uint16_t trackTwsNum;                        /**< number of tws mode traj */
	uint16_t trackTasNum;                        /**< number of tas mode traj */
	uint16_t trackObjByte;                       /**< byte number of each track object */
	uint16_t trackTwsTasFlag;                    /**< the type of scan, 0-tws, 1-tas */
    sTrackingTarget trajList[MAX_NUM_TRAJS];           /**< information for each traj*/	
}sTrackingFrame;

typedef enum
{
	UNSTABLE_TRAJ = 0, // New trajectory
	STABLE_TRAJ, // Active or stable trajectory
	FREE_TRAJ // Free or deleted trajectory
} TRACK_STATE_TYPE;

/**
 * @brief 
 *  Updating State for Tracker
 */
typedef enum
{
	UPDATETYPE_ASSOCIATED = 0, // The trajectory is associcated
	UPDATETYPE_PREDICTED // There is not association for this trajectory, so just predict it
} TRACK_UPDATE_TYPE;

/**
 * @brief 
 *  The motion descriptor of Target
 */
typedef enum
{
	MOTION_UNKNOWN_DESC = 0,
	MOTION_POSITIVERADIAL,
	MOTION_NEGITIVERADIAL,
	MOTION_LEFTCROSSWISE,
	MOTION_RIGHTCROSSWISE,
	MOTION_STATIC,
	MOTION_STOP
} TARGET_MOTION_DESC;

typedef struct {
	int32_t	heading;
	int32_t	pitching;     // in 2^30/90
	int32_t	rolling;
	int32_t	longitude;
	int32_t	latitude;
	int32_t	altitude;
	int32_t	velocityNavi;
	uint16_t targetTimeMark[6];
	uint32_t sigProcRelativeTime;
	uint16_t reserve[4];
}sPlatformData;

typedef struct {
	uint32_t frameID;
	uint32_t timestamp;
	sPlatformData platformData;
} sPlatformInfo;

typedef struct {
	uint8_t	trSwitchCtrl;
	uint8_t	workMode;
	uint8_t	workWaveCode;
	uint8_t	workFreqCode;
	uint8_t	prfPeriod;
	uint8_t	accuNum;
	uint16_t noiseCoef;
	uint16_t clutterCoef;
	uint16_t cfarCoef;
	uint16_t focusRangeMin;
	uint16_t focusRangeMax;
	int16_t	clutterCurveNum;
	int16_t	lobeCompCoef;
	uint8_t	cohesionVelThre;
	uint8_t	cohesionRgnThre;
	uint8_t	clutterMapSwitch;
	uint8_t	clutterMapUpdateCoef;
	int8_t	aziCalcSlope;
	int8_t	aziCalcPhase;
	int8_t	eleCalcSlope;
	int8_t	eleCalcPhase;
	int8_t	aziScanCenter;
	uint8_t	aziScanScope;
	int8_t	eleScanCenter;
	uint8_t	eleScanScope;
	uint16_t coherentDetectSwitch;
	uint16_t reserve[3];
}sDebugAndCtrlData;
typedef struct {
	uint32_t frameID;
	uint32_t timestamp;
	sDebugAndCtrlData debugAndCtrlData;
}sDebugAndCtrlParam;

typedef struct sDispatchOutput {
	uint32_t frameID;
	uint32_t timestamp;			// uint = ms
	uint16_t aziBeamID;			// azimuth direction beam index 0~29, for -58:4:58(degree)
	uint16_t eleBeamID;			// elevation direction beam index 0~3, for -15:10:15(degree)
	int16_t aziBeamSin;			// sin value of azimuth direction beam, uint = 1/32767
	int16_t eleBeamSin;			// sin value of elevation direction beam, uint = 1/32767
	uint8_t tasBeamTotal;		// number of tas scan beam total, only valid for tas scan mode
	uint8_t tasBeamCntCur;		// number of current tas scan beam count, only valid for tas scan mode
	uint16_t tasObjId;			// index of tas object, only valid for tas scan mode
	uint16_t tasObjFilterNum;	// filter number of tas object, only valid for tas scan mode
	uint16_t tasObjRange;		// range unit number of tas object
	uint16_t samplePntStart;	// start of sample point, uint = 0.2us
	uint16_t samplePntDepth;	// depth of sample point, default 4096
	uint16_t beamSwitchTime;	// time of beam switch, uint = 0.2us
	uint32_t wholeSpaceScanCycleCnt;	// cycle counter of whole space scan
	uint16_t trackTwsTasFlag;	// the type of scan, 0-tws, 1-tas
	uint16_t reserve[3];		// reserve, default 0
}sDispatchOutput;
	




#endif // EMBED_API_H_






