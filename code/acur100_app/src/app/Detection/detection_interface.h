#ifndef DETECTION_INIT_H_
#define DETECTION_INIT_H_


/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "../app_init.h"
#include "../../inc/radar_common.h"

/***** Detection algorithm version  *****
det_01.110110
     | --------- main detection algorithm version number
	   | ------- version number of wave type
         | ------- version number of band width, adc sampling frequency, sampling time, fft window function ...
          | ------- version number of coherent / non - coherent accumulation, CFAR ...
           | ------- version number of range, velocity estimation ...
            | ------- version number of DOA
             | ------- version number of detection postprocess
*/
#define DETEC_ALG_VERSION "DET_0.003000"
#define DETEC_VERSION_STR_LEN (14)
/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define DEFAULT_RANGE_BINS_EXP			9//11	// 2^11
#define DEFAULT_RANGE_BINS				(1 << DEFAULT_RANGE_BINS_EXP) 	// 2048=2^11
#define DEFAULT_DOPPLER_BINS_EXP		5	// 2^5
#define DEFAULT_DOPPLER_BINS			(1 << DEFAULT_DOPPLER_BINS_EXP)	// 32=2^5
#define DEFAULT_HIST_BINS_EXP			6	// 2^6
#define DEFAULT_HIST_BINS				(1 << DEFAULT_HIST_BINS_EXP)	// 64=2^6
#define PACKED_BITMAP_ELEMENT_SIZE_EXP	5	// 2^5
#define PACKED_BITMAP_ELEMENT_SIZE 		(1 << PACKED_BITMAP_ELEMENT_SIZE_EXP)	// 32=2^5
#define PACKED_BITMAP_NUM_PER_DOPPLER_EXP	(DEFAULT_RANGE_BINS_EXP - PACKED_BITMAP_ELEMENT_SIZE_EXP) // 2^(11-5)
#define PACKED_BITMAP_NUM_PER_DOPPLER 	(1 <<PACKED_BITMAP_NUM_PER_DOPPLER_EXP) // 64=2^(11-5)
#define PACKED_BITMAP_NUM_ALL_RDM	 	(DEFAULT_RANGE_BINS / PACKED_BITMAP_ELEMENT_SIZE * DEFAULT_DOPPLER_BINS)


/*=================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
=================================================================================================*/
typedef struct sDetectObjData
{
	protocol_object_list_detected_t *detectList; // the output of signal process module
	uint32_t *rdmapData;
	uint32_t *rdmapTransposeData;
	uint16_t *histBuff;
	uint16_t *hististMaxIndex;
	uint16_t *threshold;
	uint32_t *peakBitmap;
	uint8_t *rdmapDooplerDimPeakFlag;
}sDetectObjData_t;


/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
char* getDetAlgVersion();
void DataPathCB(u8 *pBuf, s32 len);
int32_t DetectBufferAlloc(sDetectObjData_t *detectObj);
extern void DetectAlgInit(void);
extern s32 runDetectAlgBlocking(void);


#endif /*DETECTION_INIT_H_*/
