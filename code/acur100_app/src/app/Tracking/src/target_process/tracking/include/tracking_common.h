
#ifndef TRACKING_COMMON_H
#define TRACKING_COMMON_H

#include <time.h>

#include "../../../utilities/common_struct.h"
#include"../../measurementProcess/include/measurementProcess.h"

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif

#include "../include/tracking_listlib.h"

// #define HEADING_FILTER_ENABLE
#define USING_WEIGHT_CURRENTTARGET
//#define ENABLE_DISAMBIGUITYVELOCITY
//#define	CLOSE_OBJECT_GATING_CTRL
#ifndef MMWAVE_3D
#define TRACKING_2D
#else
#define TRACKING_3D
#endif

#define CLOSE_RANGE (60)

/**
 * @brief   Base error code for TRACKING algorithm
 */
#define TRACKING_ERRNO_BASE		(-8000)

/**
 * @brief   Error code: No errors
 */
#define TRACKING_EOK				(0)

/**
 * @brief   Error Code: Invalid argument
 */
#define TRACKING_EINVAL			(TRACKING_ERRNO_BASE-1)

/**
 * @brief   Error Code: Operation cannot be implemented because a previous
 * operation is still not complete.
 */
#define TRACKING_EINUSE			(TRACKING_ERRNO_BASE-2)

/**
 * @brief   Error Code: Operation is not implemented.
 */
#define TRACKING_ENOTIMPL			(TRACKING_ERRNO_BASE-3)

/**
 * @brief   Error Code: Out of memory
 */
#define TRACKING_ENOMEM			(TRACKING_ERRNO_BASE-4)

/**
 * @brief   Set the max size of TAS ids to report, the reset be moved to TWS
 */
#define TAS_ID_MAX_SIZE			3

/** @}*/

/** 
@addtogroup TRACKING_ALG_EXTERNAL_DATA_STRUCTURE
@{ */

/**
 * @name Maximum supported configurations
 * @{ */
#define TRACKING_NUM_POINTS_MAX (MAX_NUM_DETECTS) /**< @brief Defines maximum possible number of measurments point the algorithm will accept at configuration time */
#define TRACKING_NUM_TRACKS_MAX (MAX_NUM_TRAJS)	/**< @brief Defines maximum possible number of tracking target the algorithm will accept at configuration time */
/** @} */

/**
 * @name Target ID definitions
 * @{ 
 * @details
 *  Target IDs are uint8_t, with valid IDs ranging from 0 to 249. Values 250 to 252 are reserved for future use, other values as defined below
 */
#define TRACKING_ID_POINT_TOO_WEAK            (1001U)   /**< @brief Point is not associated, is too weak */
#define TRACKING_ID_POINT_BEHIND_THE_WALL     (1002U)   /**< @brief Point is not associated, is behind the wall */
#define TRACKING_ID_POINT_NOT_ASSOCIATED      (1003U)   /**< @brief Point is not associated, noise */
#define TRACKING_ID_POINT_NOISE_VELOCITY      (1004U)   /**< @brief Point is not associated, noise */
#define TRACKING_ID_POINT_NOAINITIAL     (1005U)   /**< @brief Point is not initial, is behind the wall */

/** @} */
#define STATIC_CLUSTER_NUM (20U)
/**
 * @name Benchmarking results
 * @{  
 * @details
 *  During runtime execution, tracking step function can optionally return cycle counts for the sub-functions defined below
 *  Each count is 32bit unsigned integer value representing a timestamp of free runing clock 
 */
#define TRACKING_BENCHMARK_SETUP              (0U)    /**< @brief Cycle count at step setup */
#define TRACKING_BENCHMARK_PREDICT            (1U)    /**< @brief Cycle count after predict function */
#define TRACKING_BENCHMARK_ASSOCIATE          (2U)    /**< @brief Cycle count after associate function */
#define TRACKING_BENCHMARK_ALLOCATE           (3U)    /**< @brief Cycle count after allocate function */
#define TRACKING_BENCHMARK_UPDATE             (4U)    /**< @brief Cycle count after update function */
#define TRACKING_BENCHMARK_REPORT             (5U)    /**< @brief Cycle count after report function */

#define TRACKING_BENCHMARK_SIZE               (TRACKING_BENCHMARK_REPORT)    /**< @brief Size of benchmarking array */
/** @} */

#define TRACKING_POS_DIMENSION (0U)
#define TRACKING_VEL_DIMENSION (1U)
#define TRACKING_ACC_DIMENSION (2U)

#define C_LIGHTSPEED_MPS		300000000.0f	// c=3.0*1e8m/s
#define B_BANDWIDTH_HZ			39961000.0f		// B=4096/25MHz/205us*50MHz
#define F_FREQ_CENTER_HZ		24000000000.0f	// Fc=24.0GHz
#define TIME_CHIRP_S			(0.000235f * 2.f)		// Tc=235us
#define LAMBDA_M				(C_LIGHTSPEED_MPS / F_FREQ_CENTER_HZ)	// c/Fc
#define RANGE_RESO				(C_LIGHTSPEED_MPS / (2.f * B_BANDWIDTH_HZ))	// c/(2*B)
#define VELOCITY_RANGE			(LAMBDA_M / (2.f * TIME_CHIRP_S))	// lambda/(2*N*Tc)


#ifdef TRACKING_3D
#include "tracking_3d.h"
#else
#include "tracking_2d.h"
#endif
extern sScanType g_scanType;
/**
 * @name Boundary boxes
 * @{  
 * @details
 *  Application can configure tracker with scene boundries. Boundaries are defined as a boxes.
 */
#define TRACKING_MAX_BOUNDARY_BOXES           (2U)    /**< @brief Maximum number of boundary boxes. Points outside of boundary boxes are ignored */
#define TRACKING_MAX_STATIC_BOXES             (2U)    /**< @brief Maximum number of static boxes. Targets inside the static box can persist longer */
/** @} */
#define AZIMUTH_BOUNDARY (89.f*DEG2RAD)
/**
 * @brief
 *  TRACKING Box Structure
 *
 * @details
 *  The structure defines the box element used to describe the scenery 
 */
typedef struct {
    /**  @brief   Left boundary, m */
	float x1;
    /**  @brief   Right boundary, m */
	float x2;
    /**  @brief   Near boundary, m */
	float y1;
    /**  @brief   Far boundary, m */
	float y2;
    /**  @brief   Bottom boundary, m */
	float z1;
    /**  @brief   Top boundary, m */
	float z2;
} sTracking_boundaryBox; 

/**
 * @brief
 *  TRACKING Gate Limits
 *
 * @details
 *  The structure describes the limits the gating function will expand
 */
typedef struct {
    /**  @brief   Width limit, m */
	float depth;
    /**  @brief   Depth limit, m */
	float width;
    /**  @brief   Heigth limit, m */
	float height;
    /**  @brief   Radial velocity limit, m/s */
	float vel;
} sTracking_gateLimits; 


/**
 * @brief 
 *  TRACKING Update Function Parameters
 *
 * @details
 *  The structure describes default standard deviation values applied when no variance information provided in the point Cloud
 */
typedef struct {
    /**  @brief Expected standard deviation of measurements in target length dimension*/
    float widthStd;
    /**  @brief Expected standard deviation of measurements in target length dimension*/
    float depthStd;
    /**  @brief Expected standard deviation of measurements in target width dimension*/
    float heightStd;
    /**  @brief Expected standard deviation of measurements of target radial velocity */
    float dopplerStd;
} sTracking_varParams;

typedef struct {
	/**  @brief   azimuth boundary, deg */
	float aziScanScope;
	/**  @brief   elevation boundary, deg */
	float eleScanScope;
}sTracking_scanBoundary;
/**
 * @brief 
 *  TRACKING Scene Parameters
 *
 * @details
 *  This set of parameters describes the scenery. <br>
 *  It allows user to configure the tracker with expected boundaries, and areas of static behavior. <br>
 *  User can define up to \ref TRACKING_MAX_BOUNDARY_BOXES boundary boxes, and up to \ref TRACKING_MAX_STATIC_BOXES static boxes. <br>
 *  Boxes coordinates are in meters, sensor is assumed at (0, 0) of Cartesian (X, Y) space.
 */
typedef struct {
    /**  @brief Number of scene boundary boxes. If defined (numBoundaryBoxes > 0), only points within the boundary box(s) can be associated with tracks */
    uint8_t             numBoundaryBoxes;
    /**  @brief Scene boundary boxes */
    sTracking_boundaryBox  boundaryBox[TRACKING_MAX_BOUNDARY_BOXES];
    /**  @brief Number of scene static boxes. If defined (numStaticBoxes > 0), only targets within the static box(s) can persist as static */
    uint8_t             numStaticBoxes;
    /**  @brief Scene static boxes */
    sTracking_boundaryBox  staticBox[TRACKING_MAX_STATIC_BOXES];

	uint8_t numROIBoxes;
    sTracking_boundaryBox  roiBox[TRACKING_MAX_STATIC_BOXES];
	// In these areas, check the noise points based on mag, rcs
	uint8_t numNoiseBoxes;
    sTracking_boundaryBox  noiseBox[TRACKING_MAX_STATIC_BOXES];

	sTracking_scanBoundary  scanBoundary;

	float mag_min_thres[TRACKING_MAX_STATIC_BOXES]; // for noiseBox
	float mag_asso_inROIThres[TRACKING_MAX_STATIC_BOXES]; // for noiseBox
	float mag_asso_outROIThres;
	float rcs_asso_inROIThres[TRACKING_MAX_STATIC_BOXES]; // for noiseBox
	float rcs_asso_outROIThres;
	float snr_min_thres[TRACKING_MAX_STATIC_BOXES]; // for noiseBox
	float snr_asso_inROIThres[TRACKING_MAX_STATIC_BOXES]; // for noiseBox
	float snr_asso_outROIThres;
} sTracking_sceneryParams;


/**
 * @brief 
 *  TRACKING Gating Function Parameters
 *
 * @details
 *  The structure describes gating function parameters
 */
typedef struct {
    /**  @brief   Gain of the gating function */
	float		gain;
    /**  @brief   Gating function limits */
    union {
	    sTracking_gateLimits	limits;
        float limitsArray[4];
    };
} sTracking_gatingParams;

typedef struct {
	/**  @brief    */
	float		gating;
} sTracking_mdGatingParams;
/**
 * @brief 
 *  TRACKING Tracking Management Function Parameters
 *
 * @details
 *  The structure describes the thresholds for state changing counters
 */
typedef struct {
    /**  @brief  DETECTION => ACTIVE threshold */
	uint16_t det2actThre;
    /**  @brief  DETECTION => FREE threshold */
    uint16_t det2freeThre;
    /**  @brief  ACTIVE => FREE threshold */
    uint16_t active2freeThre;
    /**  @brief  EXIT ZONE => FREE threshold */
	uint16_t exit2freeThre;
	/**  @brief  ACTIVE => FREE time threshold */
	float unseenTimeThre;
} sTracking_stateParams;


/**
 * @brief 
 *  TRACKING Allocation Function Parameters
 *
 * @details
 *  The structure describes the thresholds used in Allocation function
 */
typedef struct {
    /**  @brief  Minimum total SNR */
    float snrThre;
    /**  @brief  Minimum total SNR when behind another target */
    float snrThreObscured;
    /**  @brief  Minimum initial velocity, m/s */
    float velocityThre;
    /**  @brief  Minimum number of points in a set */
	uint16_t pointsThre;
    /**  @brief  Maximum squared distance between points in a set */
    float	maxDistanceThre;
    /**  @brief  Maximum velocity delta between points in a set */
    float	maxVelThre;
	// Rectangle threshold
	sTracking_gateLimits limits;
} sTracking_allocationParams;

/**
 * @brief 
 *  TRACKING Unrolling Parameters
 *
 * @details
 *  The structure describes the filtering parameters used to switch unrolling states
 */
typedef struct {
    /**  @brief  Range rate filtering alpha */
	float alpha;
    /**  @brief  Range rate filtering confidence */
	float confidence;
} sTracking_unrollingParams;

typedef struct {
	float peakVal_min; // In ROI, remove the noise based on this
	float peakVal_asso; // The threshold of mag association
	float rcs_asso; // The threshold of rcs for association
}tracking_dbParams;

/**
 * @brief 
 *  TRACKING Advanced Parameters
 *
 * @details
 *  The structure describes advanced configuration parameters
 */
typedef struct {
	/**  @brief  Pointer to gating parameters */
	sTracking_gatingParams gatingParams;
	/**  @brief  Pointer to gating parameters for out of roi */
	sTracking_gatingParams outOfRoiGatingParams;
    /**  @brief  Pointer to allocation parameters */
	sTracking_allocationParams allocationParams;
    /**  @brief  Pointer to allocation parameters for out of roi */
	sTracking_allocationParams outOfRoiAllocationParams;
    /**  @brief  Pointer to unrolling parameters */
	sTracking_unrollingParams unrollingParams;
    /**  @brief  Pointer to tracking state parameters */
	sTracking_stateParams tasStateParams;
	/**  @brief  Pointer to tracking state parameters */
	sTracking_stateParams twsStateParams;
    /**  @brief  Pointer to tracking state parameters for out of roi */
	sTracking_stateParams outOfRoiStateParams;
    /**  @brief  Pointer to measurements variation parameters */
	sTracking_varParams   variationParams;
    /**  @brief  Pointer to scenery parameters */
	sTracking_sceneryParams sceneryParams;
    /**  @brief  For mag, rcs, snr parameters */
	tracking_dbParams dbParams;
	/**  @brief  Pointer to extended gating parameters, the final gating = dynamic_gating + ex_gating */
	sTracking_gatingParams exGatingParams;
	/**  @brief  Pointer to extended gating parameters for out of roi, the final gating = dynamic_gating + ex_gating */
	sTracking_gatingParams exOutOfRoiGatingParams;
	/** mahalanobis distance gating for tas targets*/
	sTracking_mdGatingParams tasMdGatingParams;
	/** mahalanobis distance gating for tws targets*/
	sTracking_mdGatingParams twsMdGatingParams;

} sTracking_advancedParameters;

/**
 * @brief 
 *  TRACKING State Vector
 *
 * @details
 *  Defines State vector options 
 *		2D - Depreciated, not supported		
 *		2DA - Supported
 *		3D, 3DA - Not supported (Future)
 */
typedef enum
{
    /**  @brief   2D motion model with constant velocity. State vector has four variables S={X,Y, Vx,Vy} */
	TRACKING_STATE_VECTORS_2DV = 0,
    /**  @brief   2D motion model with constant acceleration. State vector has six variables S={X,Y, Vx,Vy, Ax,Ay} */
	TRACKING_STATE_VECTORS_2DA,
    /**  @brief   3D motion model with constant velocity. State vector has six variables S={X,Y,Z, Vx,Vy,Vz} */
	TRACKING_STATE_VECTORS_3DV,
    /**  @brief   3D motion model with constant acceleration. State vector has nine variables S={X,Y,Z, Vx,Vy,Vz, Ax,Ay,Az} */
	TRACKING_STATE_VECTORS_3DA
} sTracking_enum_state_vector_type;


/**
 * @brief 
 *  TRACKING Verbose Level
 *
 * @details
 *  Defines Algorithm verboseness level 
 */
typedef enum
{
    /**  @brief   NONE */
	TRACKING_VERBOSE_NONE = 0,
    /**  @brief   ERROR Level, only errors are reported */
	TRACKING_VERBOSE_ERROR,
    /**  @brief   WARNING Level, errors and warnings are reported */
	TRACKING_VERBOSE_WARNING,
    /**  @brief   DEBUG Level, errors, warnings, and state transitions are reported */
	TRACKING_VERBOSE_DEBUG,
    /**  @brief   MATRIX Level, previous level plus all intermediate computation results are reported */
	TRACKING_VERBOSE_MATRIX,
    /**  @brief   MAXIMUM Level, maximum amount of details are reported */
	TRACKING_VERBOSE_MAXIMUM
} sTracking_enum_verbose_type;

///**
// * @brief 
// *  TRACKING Configuration
// *
// * @details
// *  The structure describes the TRACKING algorithm configuration options. 
// */
//typedef struct
//{
//    /**  @brief   State Vector Type */
//    TRACKING_STATE_VECTOR_TYPE stateVectorType;
//    /**  @brief   Verboseness Level */
//    TRACKING_VERBOSE_TYPE verbose;
//    /**  @brief   Maximum Number of Measurement Points per frame. Up to \ref TRACKING_NUM_POINTS_MAX supported */
//	uint16_t maxNumPoints;
//    /**  @brief   Maximum Number of Tracking Objects. Up to \ref TRACKING_NUM_TRACKS_MAX supported */
//	uint16_t maxNumTracks;

//    /**  @brief   Expected target radial velocity at the moment of detection, m/s */
//    float initialRadialVelocity;
//    /**  @brief   Maximum radial velocity reported by sensor +/- m/s */
//	float maxRadialVelocity;
//    /**  @brief   Radial Velocity resolution, m/s */
//	float radialVelocityResolution;
//    /**  @brief   Maximum expected target acceleration in lateral (X), longitudinal (Y), and vertical (Z) directions, m/s2 */
//    float maxAcceleration[3];
//    /**  @brief   Frame duration, s */
//    float dt;

//    /**  @brief   Advanced parameters, set to NULL for defaults */
//    TRACKING_advancedParameters advParams;
//	//int enable_confCoef;
//} TRACKING_moduleConfig;
#ifdef TRACKING_2D
#define TRACKING_STATE_VECTOR_SIZE (sizeof(sTracking_vector_pos_vel_acc)*2/sizeof(float))
#elif defined TRACKING_3D
#define TRACKING_STATE_VECTOR_SIZE (sizeof(sTracking_vector_pos_vel_acc)*3/sizeof(float))
#endif // TRACKING_2D


#define TRACKING_STATE_SINGLE_VECTOR_SIZE (sizeof(sTracking_vector_pos_vel_acc)/sizeof(float))
#define TRACKING_MEASUREMENT_CART_SINGLE_VECTOR_SIZE (sizeof(sTracking_vector_pos_vel)/sizeof(float))

#define GC_SIZE (3) // x, y, Vrad
#define SSIZE_X (TRACKING_STATE_SINGLE_VECTOR_SIZE)
#define SSIZE_XY (TRACKING_STATE_VECTOR_SIZE)
#define SSIZE_XY_TRIANGLE  (21)

#define TRACKING_NUM_SECTORS (8U)

#ifdef TA_PROJECT
#define TRACKING_MIN_STATIC_VELOCITY                  (0.5f)  /* 0.2 m/s */
#define TRACKING_MIN_STATIC_VELOCITY_RAD                  (1.0f)  /* 0.2 m/s */

#else
#define TRACKING_MIN_STATIC_VELOCITY                  (0.2f)  /* 0.2 m/s */
#define TRACKING_MIN_STATIC_VELOCITY_RAD                  (0.5f)  /* 0.2 m/s */
#endif


typedef union {
    /**  @brief   Measurement vector */
    sMeasurement_Sph_vector vector;
    float array[MSIZE_SPH];
} sTracking_measurementSphUnion;

typedef union {
    /**  @brief   Measurement vector */
	sTracking_vector_pos_vel vector;
    float array[MSIZE_X];
} sTracking_measurementCartUnion;

/**
 * @brief 
 *  TRACKING Unit Parameters structure
 *
 * @details
 *  The structure describes the parameters used to instantiate TRACKING unit instance
 */
typedef struct {
    /**  @brief  Track Unit Identifier (UID) */
	uint16_t uid;
    /**  @brief  State vector type to use */
	sTracking_enum_state_vector_type stateVectorType;
    /**  @brief  Verbosenes mask to use */
    uint64_t verbose;
	uint8_t workMode;
	/**  @brief Configured target maximum acceleration, in m/s2 */
	float   maxAcceleration[3];    
    /**  @brief Expected target radial velocity at the moment of detection, m/s */
    float   initialRadialVelocity;
    /**  @brief Absolute value of maximum radial velocity, m/s */
    float   maxRadialVelocity;
    /**  @brief Radial velocity resolution, m/s */
    float   radialVelocityResolution;
	/**  @brief Frame duration, s */
	float   dt;
	/**  @brief Number of Dimensions in State Vector, ex. 2 for constant velocity, 3 for constnat acceleration */
    uint16_t	stateVectorDimNum;
	/**  @brief Length of each dimensions in State Vector, ex. 2 for XY, 3 for XYZ */
    uint16_t	stateVectorDimLength;

	sTracking_advancedParameters advParams;
	/**  @brief Pointer to current Transition matrix */
    float   F[SSIZE_XY*SSIZE_XY];
	/**  @brief Pointer to current Process Noise matrix */
    float   Q[SSIZE_XY*SSIZE_XY];
		/**  @brief Pointer to current Transition matrix */
	// the number of extern doppler
	uint16_t numExDoppler;
	// the parameters of cluster module
	float dbscan_epsilon; // neighbor distance threshold in meter
	uint8_t dbscan_minNumPoints; // the min number of points in neighbor
	float cluster_weight;
	float cluster_vThres; // Neighbor speed threshold in m/s
} sTrackingParams;

/**
 * @brief 
 *  TRACKING Unit State
 *
 * @details
 *  The structure describes TRACKING unit states
 */
typedef enum {
    /**  @brief Free (not allocated) */
    TRACK_STATE_FREE = 0,
    /**  @brief INIT */
    TRACK_STATE_INIT,
    /**  @brief DETECTION State */
    TRACK_STATE_DETECTION,
    /**  @brief NEW ACTIVE State */
    TRACK_STATE_NEW_ACTIVE,
    /**  @brief ACTIVE State */
    TRACK_STATE_ACTIVE
} sTracking_enum_state;

/**
 * @brief 
 *  TRACKING Unit Velocity Handling State
 *
 * @details
 *  The structure describes TRACKING velocity handling states
 */
typedef enum {
    /**  @brief INIT */
    VELOCITY_INIT = 0,
    /**  @brief Range Rate filtering */
    VELOCITY_RATE_FILTER,
    /**  @brief Stabilizing Velocity Estimation */
    VELOCITY_TRACKING,
    /**  @brief Locked */
    VELOCITY_LOCKED
} sTracking_enum_velocityHandlingState;

// Current Target after association based on detections
typedef struct{
	sTracking_measurementSphUnion unionSph;
	sTracking_measurementSphUnion unionSphVar;
	sTracking_measurementCartUnion unionX;
	sTracking_measurementCartUnion unionXVar;
	sTracking_measurementCartUnion unionY;
	sTracking_measurementCartUnion unionYVar;
	float fPDH0; // probability that the current target is false P(D/H0)
	float fRCS; // RCS of current target
	float fMag; // Peak value of current target
	float fAngel;
	float fMagCoorX;
	float fMagCoorY;
	float fSNR; // SNR of current target
	float fIdealStatVrad; // Ideal radial velocity for stationary at current target
	uint16_t uNumOfDetections; // the number of detections at current target
	/**  @brief Estimated spread of the measurements */
	sTracking_measurementSphUnion estSpread;
	// Measurement Noise covariance matrix
	float Rm[MSIZE_SPH];
	// Group Dispersion
	float gD[MSIZE_SPH];

	uint8_t uFlagNearOrFarScan; // 0: near scan, 1: far scan
	uint8_t uFlagFastOrSlowScan; // 0: near scan, 1: far scan
	uint8_t uFlagMoving; // 0: static target, 1: moving target
	float fCurTargetConf; // the confidence of curTarget
	float mdScore; // the Mahalanobis distance between tracker and curTarget
}sTracking_curTarget;
// The parameters for close object
typedef struct sTrackingCloseObj {
	int closeID;
	uint8_t lateralSide;  // 1 - closeID object is on the right of tracker, 2 -  left object
	float lateralGating;  // right(lateralSide=0) or left(lateralSide=1)  lateral Gating
	uint8_t overtakeFlag;	// 1  - overtake or be overtaked
}sTrackingCloseObj;

typedef struct{
	uint8_t cycFrameID;
	uint8_t assoDetects[20];
    uint32_t frameID[4];
}sTracking_AssociateDetects;
// The distribution of associated detections based on object
typedef struct{
	uint8_t uSectorCountMaxIndex; // the sector index of max count
	uint8_t uSectorOccupationSum; // the detections occupation cnt of all sectors based on tracker
	uint8_t uSectorOccupation[TRACKING_NUM_SECTORS];
}sTracking_assoDistribution;

// The dynamic classification for object
typedef enum{
	TRACKING_DYNAMIC_UNKNOWN = 0,
	TRACKING_DYNAMIC_LEAVING,
	TRACKING_DYNAMIC_ONCOMING,
	TRACKING_DYNAMIC_CROSSING,
	TRACKING_DYNAMIC_STOP,
	TRACKING_DYNAMIC_STATIC,
	TRACKING_DYNAMIC_MOVING,
}sTracking_enum_objDynamicProperty;

// the kalman state of object
typedef struct{
	/**  @brief State matrix for moving object, estimated */
    float S_hat[SSIZE_XY];
	/**  @brief Process matrix for moving object, estimated */
	float P_hat[SSIZE_XY*SSIZE_XY];
	/**  @brief Expected Measurement matrix */
    sTracking_measurementSphUnion H_s;
	// Measurement Noise Corvariance for kalman filter
    float Rc[MSIZE_SPH];
	// tracker heading over ground
	float fTrackerHeading;
	// The velocity over ground
	float fVabsX;
	float fVabsY;
	uint8_t uType; // 0: KF, 1: EKF
}sTracking_objKalman;

// The parameters or property for association
typedef struct{
	/**  @brief Limits for associated measurments */
	sTracking_measurementSphUnion H_limits;
	// Association gating
	sTracking_gateLimits assocGating; 
	sTracking_gateLimits assocDynamicGating; 
	sTracking_measurementSphUnion assocDynamic_H_limits;
	/**  @brief Inverse of Group Covariance matrix */
	float gC_inv[MSIZE_SPH*MSIZE_SPH]; // GC_SIZE
	/**  @brief determinant of Group Covariance matrix */
	float gC_det;
	/**  @brief Gain used in association function */
	float G;
	//// The gating volume
	float ellipseVolume;

	float assMahDistance;
	/**  @brief Bit Array of indicators which detections are associated with tracker */
	uint8_t bitIndicatorsOfDetAsso[TRACKING_NUM_POINTS_MAX >> 3];
	// The sum of delta between tracker and associated detection
	sMeasurement_SphPoint u_tilda_sum;
	//// The distribution of associated detections based on object
	//sTracking_assoDistribution assoDistribution;
	int8_t assoCondenceMeasId;		// 
	float lastSeenTime;
	uint32_t lastSeenWholeScanId;
}sTracking_associationProperty;

typedef struct {
	float x0;
	float y0;
	float z0;
}sTracking_initPosition;
// The parameters and properties for object management
typedef struct{
	/**  @brief TimeStamp */
	uint64_t	heartBeatCount;
	/**  @brief Allocation Time */
	float	allocationTime;
	/**  @brief Allocation Range */
	float		allocationRange;
	/**  @brief Allocation Radial Velocity */
	float		allocationVelocity;
	/**  @brief Estimated Number of Points */
	float		estNumOfPoints;
	/**  @brief Detection state count to active */
	uint16_t	detect2activeCount;
	/**  @brief Detection state count to free */
	uint8_t	detect2freeCount;
	/**  @brief Active state count to free */
	uint8_t	active2freeCount;
	// lifetime
	int lifetime;
	/**  @brief The update status in the past 8 frames,by hxj*/
	uint8_t	updatePast8frame;

	uint8_t	forcastFrameNum;

	/**  @brief Current State */
	sTracking_enum_state	state;
	// Is new tracker or not. 1: new tracker, 0: old tracker
	// New Tracker, means it is created in current frame
	uint8_t isNewTracker;
//	uint8_t abnormalTrackerCnter; // the counter of abnormal tracker
}sTracking_objManagement;

// The object kinematic parameters
typedef struct{
    /**  @brief Current velocity handling State */
	sTracking_enum_velocityHandlingState	velocityHandling;
    /**  @brief Current Range Rate value*/
    float		rangeRate;
	/**  @brief the maxmum distance to init position*/
	float maxDistance;
	/**  @brief init position*/
	sTracking_initPosition initPosition;
}sTracking_kinematic;

// The parameters and properties of object geometry
typedef struct{
	/**  @brief Scoring */
	uint16_t score;
	float max_rcs;
	float max_snr;
	float max_peakVal;
	float max_angelSin;
	float max_peakValCoorY;
	float max_peakValCoorX;
	float length; // The length of tracker in m
	float width; // The width of tracker in m
	float height; // The height of tracker in m
	uint8_t last_asso_state; // The association state in last frame
	uint8_t flag_truck;    //Is truck or bus
	float existingProb; // The probability of existing trajectory
	uint8_t hasTrackerOfNearRegion; // Is existing tracker in near region
	uint8_t hasStbTrkOfNrRgnOv30SpdNewTrk; // new tracker around Is existing stabel tracker in speed over 30km/h
	//0: not;1:lateral distance is 1m;3:lateral distance is 3m;5:is 5m; 
	uint8_t hasStbTrkOfNrRgnOv30SpdAsso;//stable traker around Is existing stabel tracker in speed over 30km/h
	//0: not;1:lateral distance is 1m;3:lateral distance is 3m;
	uint8_t hasStbTrkNotAssoAllPointOv30Spd;
	float historyTrajAbsLength; // the absolute lenght of trajectory for tracker in history
	float historyTrajAbsLongLenWithoutAssoc; // the absolute long lenght of trajectory for tracker in history when no association
	float historyTrajAbsLatLenWithoutAssoc; // the absolute lat lenght of trajectory for tracker in history when no association
	float trackerLatInFirstFrame; // the lat of tracker in first frame when generating the tracker
	uint8_t zeroRegionCnt; // the counter in near region for this object
	uint8_t assocStatCounter; // The frame counter of associated stationary target. If associating with stat, plus 1. If not, subtract 5
	uint8_t assocMovingCounter; // The frame counter of associated moving target. If associating with stat, plus 1. If not, subtract 5
	uint8_t assocMovingMaxCounter; // The max frame counter of associated moving target
	// 1. The tracker array of same root for multi-hypothesis initialization, 
	//    and each tracker has 3 initializated hypothesis including its own.
	// 2. The value also means the tracker id of same root
	uint16_t sameRootTrackers[NUM_EXTERN_DOPPLER];
	uint8_t numSameRootTrackers; // The count of sameRootTrackers's elements
	uint8_t flag_behindRoadBorder; // 0: no, 1: yes
	uint8_t flag_car; //  0: no, 1:yes   // benyao 1206
}sTracking_objGeometryAndProperty;

// Object list
typedef struct
{
    /**  @brief Tracking Unit identifier */
    uint16_t	    uid;
    /**  @brief Target identifier */
    uint32_t	tid;
	float timestamp;
	// the kalman state of object
	sTracking_objKalman kalman_state;
	// Current target, TODO: near and far scan
	sTracking_curTarget curTarget;
	// The dynamic classification for object
	sTracking_enum_objDynamicProperty eDynamicProperty;
	// The parameters or property for association
	sTracking_associationProperty assoProperty;
	// The parameters and properties for object management
	sTracking_objManagement objManagement;
	// The object kinematic parameters
	sTracking_kinematic objKinematic;
	// The object classification
	sTrackingParams *tracking_params;
    //associated detections point information
	sTracking_AssociateDetects assoArray;
} sTracking_objectUnit;

typedef struct {
	// m/s, the ego-vehicle linear velocity
	float egoLineVel_long;
	float egoLineVel_lat;
	// rad/s, the ego-vehicle omega
	float egoOmega;
	float egoSteeringAngle; // rad/s
	// m/s2, the ego-vehicle acceleration
	float egoLineAcc_long;
	float egoLineAcc_lat;
	// m/s2, the delta ego-vehicle acceleration
	float delta_egoLineAcc_long;
	float delta_egoLineAcc_lat;
	// m, the longitudinal and lateral distance between radar and center of rotation
	float radarDist_long;
	float radarDist_lat;
	float radarCalibAngle;
	float radartofront_dis;
}sTracking_platformInfo;
/**
 * @brief 
 *  TRACKING Module instance structure
 *
 * @details
 *  The structure describes the TRACKING module instance
 */
typedef struct {

	/**  @brief Maximum number of measurement points per frame */
	uint16_t maxNumPoints;
	/**  @brief Maximum number of Tracking objects */
	uint16_t maxNumTracks;
	/**  @brief Tracking Unit Parameters  */
	sTrackingParams *tracking_params;
	int8_t tasTargetId;
	uint32_t wholeSpaceScanCycleCnt;
	uint32_t lastWholeSpaceScanCycleCnt;
	/**  @brief TimeStamp  */
	uint64_t	heartBeat;

    /**  @brief verboseness Mask */
	uint64_t	verbose;
	/**  @brief Array of best scores  */
	float		bestScore[TRACKING_NUM_POINTS_MAX];
	/**  @brief Array of best score authors (UIDs)  */
	uint16_t	bestIndex[TRACKING_NUM_POINTS_MAX];
	/**  @brief Bit Array of indicators whether best score author is unique */
	uint8_t	    isUniqueIndex[TRACKING_NUM_POINTS_MAX >> 3];
	/**  @brief Temporary Array of measurement indices for set-under-construction */
	uint16_t	allocIndex[TRACKING_NUM_POINTS_MAX];
	/**  @brief Array of Tracking Unit handles */
	void		**hTrack;
	/**  @brief List of currently tas active Tracking Units (UIDs) */
	sTracking_ListObj tasTrackList;
	/**  @brief List of currently tws active Tracking Units (UIDs) */
	sTracking_ListObj twsTrackList;
	/**  @brief List of Free Tracking Units (UIDs) */
	sTracking_ListObj freeList;
	/**  @brief Array of UID elements */
	sTracking_ListElem uidElem[TRACKING_NUM_TRACKS_MAX];

	///**  @brief Array of Target descriptors */
	//TRACKING_targetDesc	*targetDesc;
    /**  @brief Total number of tracked targets */
    uint32_t	        targetNumTotal;
	/**  @brief Number of currently tracked Targets */
	uint16_t			targetNumCurrent;
	// Ego information
	sTracking_platformInfo platformInfo;

} sTracking_moduleInstance;

typedef struct{
	sTracking_moduleInstance moduleInst; // sizeof(TRACKINGModuleInstance)
	sTracking_objectUnit* unitTrackList[MAX_NUM_TRAJS]; // sizeof(TRACKINGUnitInstance*)*TRACKING_NUM_TRACKS_MAX
	sTracking_objectUnit unitInst[MAX_NUM_TRAJS]; // sizeof(TRACKINGUnitInstance)*TRACKING_NUM_TRACKS_MAX

	uint8_t isAssociatedActive[MAX_NUM_DETECTS]; // size: TRACKING_NUM_POINTS_MAX
	uint8_t visited[MAX_NUM_DETECTS]; // size: TRACKING_NUM_POINTS_MAX
	//uint16_t *scope; // size: TRACKING_NUM_POINTS_MAX
	uint16_t neighs[MAX_NUM_DETECTS]; // size: TRACKING_NUM_POINTS_MAX
	uint16_t vIndex[MAX_NUM_DETECTS]; // size: TRACKING_NUM_POINTS_MAX
	uint8_t vIsClustered[MAX_NUM_DETECTS]; 
}sTracking_insideScratchPad;

// Process noise variance and Measurement noise variance
extern float *const_process_noise;
extern float *const_max_acceleration;
extern float *const_tws_measurement_noise;
extern float *const_tas_measurement_noise;


// Set the roi area in measurement coordination
//extern measUnitData2D *const_roiArea;
extern sTracking_sceneryParams *const_roiArea;
extern sTracking_insideScratchPad *gScratchPadData;

extern const float const_deviation;
extern const int const_clusLamda;
extern const int const_lifeLamda;

extern float g_curTimestamp;

extern void *tracking_create(sTrackingParams *config, int32_t *errCode, sTracking_insideScratchPad *scratchPadData);
extern void tracking_step(void *handle, sMeasOutput *point);

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif

#endif /* TRACKING_COMMON_H */

