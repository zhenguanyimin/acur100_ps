
#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "xtime_l.h"
#include "xil_printf.h"
#include "detection_interface.h"
#include "detection_cfar_2d.h"
#include "../DataPath/data_path.h"

char g_det_version[DETEC_VERSION_STR_LEN] = DETEC_ALG_VERSION;

static SemaphoreHandle_t handleSem = NULL;

sDetectObjData_t gDetectObjData[1] = { 0 };

int32_t gDetectAlgInitFlag = -1;

uint32_t gTimeStampCur = 0;	/* time from system startup at new frame beginning */
uint32_t gTimeStampLast = 0;

uint32_t gFrameID = 0;
uint32_t getFrameID(void)
{
	return gFrameID;
}

void setFrameID(uint32_t uiFrameID)
{
	gFrameID = uiFrameID;
}


uint32_t gRdmapData[DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS] = { 0 };	// 256k Byte, 2048*32*4 Byte
uint32_t gRdmapTransposeData[DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS] = { 0 };	// 256k Byte, 2048*32*4 Byte
uint16_t gHistBuff[DEFAULT_RANGE_BINS * DEFAULT_HIST_BINS] = { 0 };	// 256k Byte, 2048*64*2 Byte
uint16_t gHististMaxIndex[DEFAULT_RANGE_BINS] = { 0 };	// 4k Byte, 2048*2 Byte
uint16_t gThreshold[DEFAULT_RANGE_BINS] = { 0 };	// 4k Byte, 2048*2 Byte
uint32_t gPeakBitmap[PACKED_BITMAP_NUM_ALL_RDM] = { 0 };	// 8k Byte, 2048/32*32*4 Byte
uint8_t gRdmapDooplerDimPeakFlag[DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS];	// 64k Byte, 2048*32*1 Byte

char* getDetAlgVersion()
{
	return g_det_version;
}
void DataPathCB(u8 *pBuf, s32 len)
{
//	memcpy(gDetectObjData[0].rdmapData, pBuf, len);	// total len = 2048*32*4=256KB
	memcpy(gDetectObjData[0].rdmapData, pBuf, sizeof(uint32_t) * DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS);	// current len = 512*32*4=64KB

	memset(gDetectObjData[0].rdmapData, 0, sizeof(uint32_t) * DETECT_RANGE_START_INDEX * DEFAULT_DOPPLER_BINS);

	xSemaphoreGive(handleSem);
}

int32_t DetectBufferAlloc(sDetectObjData_t *detectObj)
{
	detectObj->detectList = gDetectList;
	detectObj->rdmapData = gRdmapData;
	detectObj->rdmapTransposeData = gRdmapTransposeData;
	detectObj->histBuff = gHistBuff;
	detectObj->hististMaxIndex = gHististMaxIndex;
	detectObj->threshold = gThreshold;
	detectObj->peakBitmap = gPeakBitmap;
	detectObj->rdmapDooplerDimPeakFlag = gRdmapDooplerDimPeakFlag;
	return 0;
}


void DetectAlgInit(void)
{
	if (DetectBufferAlloc(gDetectObjData) < 0)
	{
		gDetectAlgInitFlag = -1;
	}

	gDetectAlgInitFlag = 0;

	handleSem = xSemaphoreCreateBinary();
	if(!handleSem)
	{
		xil_printf("xSemaphoreCreateBinary error\r\n");
		return ;
	}
	else
	{
		data_path_setEvCallBack(DataPathCB);
	}
}


s32 runDetectAlgBlocking(void)
{
	XTime tCur = 0;
	uint32_t timeStampDiff = 0;

	if(!handleSem)
		return -1;

	xSemaphoreTake(handleSem, portMAX_DELAY);

	XTime_GetTime(&tCur);
	gTimeStampCur = tCur * 1000 / (COUNTS_PER_SECOND);

	gDetectList[0].stInfoHeader.frameID = gFrameID;
	gDetectList[0].stInfoHeader.timestamp = gTimeStampCur;
	gDetectList[0].trackTwsTasFlag = gBeamInfo[0].trackTwsTasFlag;

	DetectionAlgProcess(gDetectObjData);

	timeStampDiff = gTimeStampCur - gTimeStampLast;
	gTimeStampLast = gTimeStampCur;
	gFrameID++;
//	xil_printf("	gTimeStamp diff is %d ms\r\n", timeStampDiff);

	return 0;
}


/*================================================================================================*/
#ifdef __cplusplus
}
#endif
