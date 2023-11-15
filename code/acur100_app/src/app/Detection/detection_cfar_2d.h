#ifndef DETECTION_CFAR_2D_H
#define DETECTION_CFAR_2D_H


/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "../../inc/radar_common.h"
#include "detection_interface.h"

#ifdef __cplusplus
extern "C" {
#endif


/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

#define ENABLE_PS_STATIC 				FALSE //TRUE, FALSE

#define MAX_OUTPUT_CFAR_TARGET			64

////1020 OK
//#define HIST_THRESHOLD_GUARD			3
//#define CFAR_RANGE_THRESHOLD_SEG_1ST	2048	/* cfar range threshold segment 1st, 24 */
//#define CFAR_RANGE_THRESHOLD_SEG_2ND	1275	/* cfar range threshold segment 2nd, 15 */
//#define CFAR_RANGE_THRESHOLD_SEG_3RD	1020	/* cfar range threshold segment 3rd, 12 */
//#define CFAR_RANGE_THRESHOLD_SEG_4TH	935		/* cfar range threshold segment 4th, 11 */
//#define CFAR_RANGE_THRESHOLD_SEG_5TH	7650	/* cfar range threshold segment 5th, 90 */
//#define CFAR_RANGE_THRESHOLD_SEG_6TH	7650	/* cfar range threshold segment 6th, 90 */
//
//#define CFAR_DOPPLER_THRESHOLD_SEG_1ST	1700	/* cfar doppler threshold segment 1st, 20 */
//#define CFAR_DOPPLER_THRESHOLD_SEG_2ND	1105	/* cfar doppler threshold segment 2nd, 13 */
//#define CFAR_DOPPLER_THRESHOLD_SEG_3RD	935		/* cfar doppler threshold segment 3rd, 11 */
//#define CFAR_DOPPLER_THRESHOLD_SEG_4TH	850		/* cfar doppler threshold segment 4th, 10 */
//#define CFAR_DOPPLER_THRESHOLD_SEG_5TH	9350	/* cfar doppler threshold segment 5th, 110 */
//#define CFAR_DOPPLER_THRESHOLD_SEG_6TH	9350	/* cfar doppler threshold segment 6th, 110 */
//
//#define CFAR_GLOBAL_THRESHOLD_SEG_1ST	1275	/* cfar global threshold segment 1st, 15 */
//#define CFAR_GLOBAL_THRESHOLD_SEG_2ND	1105	/* cfar global threshold segment 2nd, 13 */
//#define CFAR_GLOBAL_THRESHOLD_SEG_3RD	1020	/* cfar global threshold segment 3rd, 12 */
//#define CFAR_GLOBAL_THRESHOLD_SEG_4TH	850		/* cfar global threshold segment 4th, 10 */
//#define CFAR_GLOBAL_THRESHOLD_SEG_5TH	7650	/* cfar global threshold segment 5th, 110 */
//#define CFAR_GLOBAL_THRESHOLD_SEG_6TH	7650	/* cfar global threshold segment 6th, 110 */

//1020test
#define HIST_THRESHOLD_GUARD			3
#define CFAR_RANGE_THRESHOLD_SEG_1ST	1871	/* cfar range threshold segment 1st, 22 */
#define CFAR_RANGE_THRESHOLD_SEG_2ND	1105	/* cfar range threshold segment 2nd, 13 */
#define CFAR_RANGE_THRESHOLD_SEG_3RD	935		/* cfar range threshold segment 3rd, 11 */
#define CFAR_RANGE_THRESHOLD_SEG_4TH	850		/* cfar range threshold segment 4th, 10 */
#define CFAR_RANGE_THRESHOLD_SEG_5TH	7650	/* cfar range threshold segment 5th, 90 */
#define CFAR_RANGE_THRESHOLD_SEG_6TH	7650	/* cfar range threshold segment 6th, 90 */

#define CFAR_DOPPLER_THRESHOLD_SEG_1ST	1615	/* cfar doppler threshold segment 1st, 19 */
#define CFAR_DOPPLER_THRESHOLD_SEG_2ND	1020	/* cfar doppler threshold segment 2nd, 12 */
#define CFAR_DOPPLER_THRESHOLD_SEG_3RD	850		/* cfar doppler threshold segment 3rd, 10 */
#define CFAR_DOPPLER_THRESHOLD_SEG_4TH	765		/* cfar doppler threshold segment 4th, 9 */
#define CFAR_DOPPLER_THRESHOLD_SEG_5TH	9350	/* cfar doppler threshold segment 5th, 110 */
#define CFAR_DOPPLER_THRESHOLD_SEG_6TH	9350	/* cfar doppler threshold segment 6th, 110 */

#define CFAR_GLOBAL_THRESHOLD_SEG_1ST	1190	/* cfar global threshold segment 1st, 14 */
#define CFAR_GLOBAL_THRESHOLD_SEG_2ND	1020	/* cfar global threshold segment 2nd, 13 */
#define CFAR_GLOBAL_THRESHOLD_SEG_3RD	935		/* cfar global threshold segment 3rd, 12 */
#define CFAR_GLOBAL_THRESHOLD_SEG_4TH	765		/* cfar global threshold segment 4th, 9 */
#define CFAR_GLOBAL_THRESHOLD_SEG_5TH	7650	/* cfar global threshold segment 5th, 110 */
#define CFAR_GLOBAL_THRESHOLD_SEG_6TH	7650	/* cfar global threshold segment 6th, 110 */



#define RANGE_CUT_INDEX_1ST				2		/* rangeBin cut index 1st 2(7.5m) */
#define RANGE_CUT_INDEX_2ND				10		/* rangeBin cut index 2nd 10(37.5m) */
#define RANGE_CUT_INDEX_3RD				40		/* rangeBin cut index 3rd 38(142.5m), 32(120m), 40(150m) */
#define RANGE_CUT_INDEX_4TH				54		/* rangeBin cut index 4th 54(202.5m) */
#define RANGE_CUT_INDEX_5TH				150		/* rangeBin cut index 5th 72(270m), 150(562.5m) */
#define RANGE_CUT_INDEX_6TH				430		/* rangeBin cut index 6th 430(1612.5m) */


#define RANGE_CUT_INDEX_REAR			500		/* range cut index rear */
#define DOPPLER_STATIC_SCOPE			6		/* doppler static scope */
#define OS_K							0.5f
#define APPROXIMATE_ZERO				0.000000001f
#define COEF_OF_SNR_CALC				(0.0039*3.0103*100)	/* log2 to log10 */
#define COEF_OF_HIST_MAG				256

#define DEFAULT_RANGE_WIN				4//3
#define DEFAULT_RANGE_GUARD				4//1
#define DEFAULT_DOPPLER_WIN				4
#define DEFAULT_DOPPLER_GUARD			4
#define DEFAULT_OS_RANGE				150


#define DEFAULT_C_LIGHTSPEED_MPS		300000000.0f	// c=3.0*1e8m/s
#define DEFAULT_B_BANDWIDTH_HZ			41059732.0f//39961000.0f		// B=4096/25MHz/205us*50MHz
#define DEFAULT_F_FREQ_CENTER_HZ		24000000000.0f	// Fc=24.0GHz
#define DEFAULT_TIME_CHIRP_S			(0.000235f * 2)		// Tc=235us
#define DEFAULT_LAMBDA_M				(DEFAULT_C_LIGHTSPEED_MPS / DEFAULT_F_FREQ_CENTER_HZ)	// c/Fc
#define DEFAULT_RANGE_RESO				(DEFAULT_C_LIGHTSPEED_MPS / (2 * DEFAULT_B_BANDWIDTH_HZ))	// c/(2*B)
#define DEFAULT_VELOCITY_RESO			(DEFAULT_LAMBDA_M / (2 * DEFAULT_DOPPLER_BINS * DEFAULT_TIME_CHIRP_S))	// lambda/(2*N*Tc)
#define DEFAULT_MAG_RESO				(0.0039*3.0103f)	// 1/256*3.0103f

#define MAX_OUTPUT_PS_POINTS 			1024
#define DETECT_RANGE_START_INDEX		4	//start index of range direction peak search

#define RDM_MAG_PER_HIST_BIN_EXP		8	//2^8
#define RDM_MAG_PER_HIST_BIN			(1 << RDM_MAG_PER_HIST_BIN_EXP) //2^8

#ifndef DEG2RAD
#define DEG2RAD 						(0.01745329f) //M_PI/180.0f
#endif
#ifndef RAD2DEG
#define RAD2DEG 						(57.2957795f) //180.0f/M_PI
#endif

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/

/*=================================================================================================
*                                             ENUMS
=================================================================================================*/

/*=================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
=================================================================================================*/
typedef struct sResolutionDetect
{
    float RangeResotion;				/* m, range resolution */
    float VelocityResotion;				/* m/s, velocity resolution */
    float MagResotion;					/* dB, magnitude resolution */
}sResolutionDetect_t;

typedef struct sCfarParameter
{
    int16_t rangeBins;					/* range dimension bins */
    int16_t dopplerBins;				/* doppler dimension bins */
    uint16_t rangeWin;					/* range dimension win cell */
    uint16_t rangeGuard;				/* range dimension guard cell */
    uint16_t dopplerWin;				/* doppler dimension win cell */
    uint16_t dopplerGuard;				/* doppler dimension guard cell */
    uint16_t rangeThresholdLen;			/* range dimension threshold length */
    uint16_t dopplerThresholdLen;		/* doppler dimension threshold length */
    uint16_t osRange;					/* range dimension oscfar bin */
    uint16_t osRangeK;					/* range dimension oscfar noise cell, biggest is K */
    uint16_t *pRangeThresholdStatic;	/* range dimension threshold for static target */
    uint16_t *pDopplerThresholdStatic;	/* doppler dimension threshold for static target */
    uint16_t *pRangeThresholdMoving;	/* range dimension threshold for moving target */
    uint16_t *pDopplerThresholdMoving;	/* doppler dimension threshold for moving target */
}sCfarParameter_t;

/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/


/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void GetRdmapData(uint32_t *rdmBuff);
ret_code_t CalcHistogram(uint32_t *rdmBuff, uint16_t *histBuff);
ret_code_t CalcThreshold(uint16_t *histBuff, uint16_t *histMaxIndex, uint16_t *threshBuff, int16_t nrRangeBins);
ret_code_t CalcTransposeRdm(uint32_t *rdmBuff, uint32_t *rdmTransposeBuff, int32_t rangeBins, int32_t dopplerBins);
ret_code_t CalcPeakSearchBitmap(uint32_t *rdmBuff, uint32_t *rdmTransposeBuff, uint16_t *threshBuff, \
		uint32_t *detectBitmap, uint8_t *rdmapDooplerDimPeakFlag, int32_t rangeBins, int32_t dopplerBins);
int32_t Range_OSCFAR(int32_t idx, int32_t ROW, int32_t COL, int32_t CFAR_WINDOW, int32_t CFAR_GUARD, uint32_t *RangeData, int32_t osRangeK);
int32_t Range_CACFAR(int32_t idx, int32_t ROW, int32_t COL, int32_t CFAR_WINDOW, int32_t CFAR_GUARD, uint32_t *RangeData);
int32_t Doppler_CACFAR(int32_t idx, int32_t ROW, int32_t COL, int32_t CFAR_WINDOW, int32_t CFAR_GUARD, uint32_t *DopplerData, bool RemoveMax_Flag);
// ret_code_t CfarDetection(uint32_t *cfarInput, uint32_t *detectBitmapInput, protocol_object_list_detected_t *detectList);
ret_code_t CfarDetection(uint32_t *cfarInput, uint32_t *detectBitmapInput, uint16_t *threshBuff, protocol_object_list_detected_t *detectList);

extern ret_code_t DetectionAlgProcess(sDetectObjData_t *detectObj);

/**
 * @}
 * @endif
 */

#ifdef __cplusplus
}
#endif

#endif /*DETECTION_CFAR_2D_H*/
