
#ifndef COMMON_DEFINE_H_
#define COMMON_DEFINE_H_

/******* Tracking algorithm version **********
Example: TRK_01.110110
           |--------- tracking algorithm
			  |--------- main version number
				|------- version number of measurement process
				 |------- version number of tracking association, tracking filter
				  |------- version number of tracking management, tracking start ...
				   |------- version number of tracking post processer (including classification, merge ...)
					|------- version number of dispatch process 
					 |------- version number of SIL interface ... 
*/
#define DEFAULT_SCAN_TYPE 0
#define TRACK_ALG_VERSION "TRK_0.616221"
#define TRACK_VERSION_STR_LEN (14)
#define false 0
#define true 1

#ifdef _WINDOWS
#define ENABLE_OUTRESULT
#ifdef ENABLE_OUTRESULT
//#define ENABLE_GETCLASSGT
#ifndef ENABLE_GETCLASSGT
//#define ENABLE_OUTMAG
#endif
#endif
#ifndef _DEBUG1
#define VISUALIZATION_GTRACK
#define VISUAL_RECORD_TRACK
#endif
#endif

#ifdef _WINDOWS
//#define DEBUG_LOG
//#define DEBUG_LOG_HXJ
//#define DEBUG_LOG_ZXL
//#define DEBUG_LOG_ZQ
//#define PRINT_TXT
//#define DEBUG_LOG_V2
#endif

//#define STATE_DETECTION_OUTPUT
#define TRANSFORM_COORDINATE

#define MMWAVE_3D
#define DEMO_ACUR
//#define USING_DUMMY_TEST

// For JPDAF
#define JPDAF_ENABLE
#define JPDAF_SIGMA 0.1f
//#define JPDAF_DEBUGLOG
//#define DATA_ASSOCIATION_USING_CLUSTERING
// For existing probability of tracker
//#define TRACKER_PROB_DEBUGLOG

#define M_PI (3.1415926f)
#define DEG2RAD (0.01745329f) //M_PI/180.0f
#define RAD2DEG (57.2957795f) //180.0f/M_PI
#define const_e (2.71828f)

#define ERROR_CODE_OFFSET 100

#define iRANGE 0
#define iAZIMUTH 1
#define iDOPPLER 2

#define NUM_EXTERN_DOPPLER (2)
#ifdef TA_PROJECT
#define DISAMBIGUITY_MAXVEL_FASTSCAN 9.24f //12.98528f // Just set it if only one scan mode (TA Radar)
#else
#define DISAMBIGUITY_MAXVEL_FASTSCAN 100.f //12.699f // Just set it if only one scan mode (TA Radar)
#endif
#define DISAMBIGUITY_MAXVEL_SLOWSCAN 100.f //10.f

#define FRAME_TIME_TWS 0.008f
#define FRAME_TIME_TAS 0.200f

#define NUMBER_RADAR 2
// OnlineExterCalibration
#define EXTERCALIBR_FRAMES_NUM 120 //100
#define EXTERCALIBR_FRAMES_RESET 20
#define EXTERCALIBR_FRAMES_OBSERVER 200 //200

// For tracking module
#ifdef USING_DUMMY_TEST
#define CONST_DELTA_T  (1/120.f) //0.05f //0.075f
#else
//#define CONST_DELTA_T 0.075f
#endif

// Collision checker
#define MAX_NUM_MAINOBJS 5
#define TTC_THRES 3.f // time to collision, s
#define TTC_THRES_MIN 1.4f // time to collision, s
// warning level zone
#define ZONE2_END (-30.f)
#define ZONE2_ZONE1_BOUNDARY (-7.f)
#define ZONE1_ZONE3_BOUNDARY (2.f)
#define ZONE3_FRONT (7.f)
#define REARAXISTOBUSREARDIST (2.545f); 

// For existing probability of tracker
#define TRACKEREXISTING_MAXPROBCLUSNUM 10
#define TRACKEREXISTING_MAXPROBLIFENUM 50
#define TRACKEREXISTING_MAXPROBMOTIONNUM 10
#define TRACKEREXISTING_W1 0.4f
#define TRACKEREXISTING_W2 0.6f
//another method,by hxj
#define TRACKEREXISTING_OR_MAXPROBMOTIONNUM 12
#define TRACKEREXISTING_OR_MAXPROBCLUSNUM 5
#define TRACKEREXISTING_OR_W1 0.3f
#define TRACKEREXISTING_OR_W2 0.4f
#define TRACKEREXISTING_OR_W3 0.3f

// For trajectory prediction in short term
#define SHORTTERM_PREDICTIOIN 3.0f // In s

// For grid map and visualizer
#define IMG_WIDTH  1600 //720 //1440 //720 //480 // The times of 12
#define IMG_HEIGHT  800 //2400 //400 //600 // The times of 10
#define IMG_WIDTH_SCALE  0.8//10 //20 //5
#define IMG_HEIGHT_SCALE  0.8 //5 //20 //5

// For classification of motion status
#define TIMEWINDOW 1
//#ifndef AEB_PORTING
#define STATIC_V 0.9f //0.9
#define MOVE_V 1.2f //1.2
#define MOVING_VEL_X 1.2f
#define MOVING_VEL_Y 1.2f
#define VX_VY_RATIO 1.3f
#define STATIC2POS_MIN_VX MOVE_V
#define STATIC2POS_MAX_VY STATIC_V
#define STATIC2NEG_MIN_VX (-1.0f*MOVE_V)
#define STATIC2NEG_MAX_VY STATIC_V
#define STATIC2LEFT_MIN_VY MOVE_V
#define STATIC2LEFT_MAX_VX STATIC_V
#define STATIC2RIGHT_MIN_VY (-1.0f*MOVE_V)
#define STATIC2RIGHT_MAX_VX STATIC_V
#define UNKNOW2STATIC_MAX_V STATIC_V
#define MOVE2STOP_MAX_V STATIC_V
#define STOP2POS_MIN_VX MOVE_V
#define STOP2POS_MAX_VY STATIC_V
#define STOP2NEG_MIN_VX (-1.0f*MOVE_V)
#define STOP2NEG_MAX_VY STATIC_V
#define STOP2LEFT_MIN_VY MOVE_V
#define STOP2LEFT_MAX_VX STATIC_V
#define STOP2RIGHT_MIN_VY (-1.0f*MOVE_V)
#define STOP2RIGHT_MAX_VX STATIC_V
//#endif

#define ASSO_MIN_ANGLE 2.0f * DEG2RAD
#define REPORT_FOV 52.f * DEG2RAD

// For classification of target status
#define STORE_POINT 6


#define OUTRADARCOORRAVH



#endif // COMMON_DEFINE_H_
