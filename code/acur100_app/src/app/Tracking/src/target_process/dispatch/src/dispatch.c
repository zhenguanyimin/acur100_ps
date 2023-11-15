
/*========================================================================================
*                                     INCLUDE FILES
=========================================================================================*/
#include "../include/dispatch.h"
#include "../../tracking/include/tracking_int.h"

/*========================================================================================
*                                    GLOBAL CONSTANTS
=========================================================================================*/
uint32_t gBeamMode = 2 ; //1 for fix beam form ,2 for beam scan
uint32_t gTasScanBeamCnt = 0;	// beam counter of tas scan mode, each beam add 1
uint32_t gWholeSpaceScanCycleCnt = 0;	// cycle counter of whole space scan
int32_t gBeamParaCalcInit = -1;

/* azimuth direction beam */
int16_t gAziFov = DEFAULT_AZI_FOV;
int16_t gAziFovCenter = DEFAULT_AZI_FOV_CENTER;
int16_t gAziBeamNum = DEFAULT_AZI_BEAM_NUM;
uint16_t gAziBeamIndexCur = 0;
uint16_t gAziBeamIndexLast = 0;
// gAziBeamIndex = 0:1:29
uint16_t gAziBeamIndex[DEFAULT_AZI_BEAM_NUM] = { 0 };
//{
//	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
//};
// gAziBeamAngle = -58:4:58, in degree
int16_t gAziBeamAngle[DEFAULT_AZI_BEAM_NUM] = { 0 };
//{
//	-58, -54, -50, -46, -42, -38, -34, -30, -26, -22,
//	-18, -14, -10, -6, -2, 2, 6, 10, 14, 18,
//	22, 26, 30, 34, 38, 42, 46, 50, 54, 58
//};
// gAziBeamAngleSin = sinf(gAziBeamAngle * DEG2RAD), in radian
float gAziBeamAngleSin[DEFAULT_AZI_BEAM_NUM] = { 0 };
//{
//	-0.848f, -0.809f, -0.766f, -0.719f, -0.669f, -0.616f, -0.559f, -0.5f, -0.438f, -0.375f,
//	-0.309f, -0.242f, -0.174f, -0.105f, -0.035f, 0.035f, 0.105f, 0.174f, 0.242f, 0.309f,
//	0.375f, 0.438f, 0.5f, 0.559f, 0.616f, 0.669f, 0.719f, 0.766f, 0.809f, 0.848f
//};
// gAziBeamAngleSinInt16 = round(gAziBeamAngleSin * 32767)
int16_t gAziBeamAngleSinInt16[DEFAULT_AZI_BEAM_NUM] = { 0 };
//{
//	-27787, -26509, -25100, -23560, -21922, -20185, -18317, -16384, -14352, -12288,
//	-10125, -7930, -5702, -3441, -1147,	1147, 3441, 5702, 7930, 10125,
//	12288, 14352, 16384, 18317, 20185, 21922, 23560, 25100, 26509, 27787
//};

/* elevation direction beam */
int16_t gEleFov = DEFAULT_ELE_FOV;
int16_t gEleFovCenter = DEFAULT_ELE_FOV_CENTER;
int16_t gEleBeamNum = DEFAULT_ELE_BEAM_NUM;
uint16_t gEleBeamIndexCur = 0;
uint16_t gEleBeamIndexLast = 0;
// gEleBeamIndex = 0:1:3
uint16_t gEleBeamIndex[DEFAULT_ELE_BEAM_NUM] = { 0 };
//{ 0, 1, 2, 3 };
// gEleBeamAngle = -15:10:15, in degree
int16_t gEleBeamAngle[DEFAULT_ELE_BEAM_NUM] = { 0 };
//{ -15, -5, 5, 15 };
// gEleBeamAngleSin = sinf(gEleBeam * DEG2RAD), in radian
float gEleBeamAngleSin[DEFAULT_ELE_BEAM_NUM] = { 0 };
//{ -0.259f, -0.087f, 0.087f, 0.259f };
// gEleBeamAngleSinInt16 = round(gEleBeamAngleSin * 32767)
int16_t gEleBeamAngleSinInt16[DEFAULT_ELE_BEAM_NUM] = { 0 };
//{ -8487, -2851, 2851, 8487 };

// default offset angle(down, up, left, right) of tas target azimuth angle, in degree
int16_t gTasAziOffsetAngle[TAS_CYCLE_TOTAL_BEAM_NUM] = { 0, 0, DEFAULT_TAS_AZI_OFFSET_LEFT, DEFAULT_TAS_AZI_OFFSET_RIGHT };
// default offset angle(down, up, left, right) of tas target elevation angle, in degree
int16_t gTasEleOffsetAngle[TAS_CYCLE_TOTAL_BEAM_NUM] = { DEFAULT_TAS_ELE_OFFSET_DOWN, DEFAULT_TAS_ELE_OFFSET_UP, 0, 0 };


/*========================================================================================
*                                    FUNCTIONS
=========================================================================================*/
void DispatchProcess(sAlgObjData *algObj)
{
	sDispatchInst *dispatch_inst = (sDispatchInst *)algObj->dispatch_inst;
	sTrackingInst *tracking_inst = (sTrackingInst *)algObj->tracking_inst;
	sTrackingFrame *tracksListOutput = tracking_inst->trajInfoOutput;
	sTracking_moduleInstance *instModule = (sTracking_moduleInstance *)tracking_inst->handle;

	if (-1 == gBeamParaCalcInit)
	{
		dispatch_inst->dispatchConfigParam->aziScanCenter = DEFAULT_AZI_FOV_CENTER;
		dispatch_inst->dispatchConfigParam->aziScanScope = DEFAULT_AZI_FOV;
		dispatch_inst->dispatchConfigParam->eleScanCenter = DEFAULT_ELE_FOV_CENTER;
		dispatch_inst->dispatchConfigParam->eleScanScope = DEFAULT_ELE_FOV;
		DispatchCfgParaUpdate(dispatch_inst->dispatchConfigParam);
		gBeamParaCalcInit = 0;
	}

	if (dispatch_inst->dispatchConfigParam->valid)
	{// valid
		if ((gAziFovCenter != dispatch_inst->dispatchConfigParam->aziScanCenter) ||
			(gAziFov != dispatch_inst->dispatchConfigParam->aziScanScope) ||
			(gEleFovCenter != dispatch_inst->dispatchConfigParam->eleScanCenter) ||
			(gEleFov != dispatch_inst->dispatchConfigParam->eleScanScope))
		{// if azimuth or elevation config params has changed
			DispatchCfgParaUpdate(dispatch_inst->dispatchConfigParam);
		}
		gAziFovCenter = dispatch_inst->dispatchConfigParam->aziScanCenter;
		gAziFov = dispatch_inst->dispatchConfigParam->aziScanScope;
		gEleFovCenter = dispatch_inst->dispatchConfigParam->eleScanCenter;
		gEleFov = dispatch_inst->dispatchConfigParam->eleScanScope;
	}

	if (dispatch_inst->dispatchConfigParam->workMode == 0)
	{// TWS_SCAN, first elevation scan, then azimuth scan
		DispatchTwsScan(dispatch_inst, tracksListOutput);
	}
	else
	{// TAS_SCAN
		DispatchTasScan(dispatch_inst, tracksListOutput, instModule);
	}
}


void DispatchCfgParaUpdate(sDispatchConfigParam *param)
{
	uint16_t i = 0;
	int8_t aziScanCenter = 0;
	int8_t aziScanScope = 0;
	int8_t aziScanScopeLowerBound = 0;
	int8_t eleScanCenter = 0;
	int8_t eleScanScope = 0;
	int8_t eleScanScopeLowerBound = 0;

	/* 1. update azimuth beam config params */
	aziScanCenter = param->aziScanCenter;
	if (((param->aziScanScope / 2) + (int8_t)(fabsf(aziScanCenter))) <= DEFAULT_AZI_FOV_UPPER)
	{
		aziScanScope = param->aziScanScope;
	}
	else
	{
		aziScanScope = (DEFAULT_AZI_FOV_UPPER - (int8_t)(fabsf(aziScanCenter))) * 2;
	}

	aziScanScopeLowerBound = aziScanCenter - (aziScanScope / 2);
	if (aziScanScope >= DEFAULT_AZI_BEAM_INTER)
	{
		gAziBeamAngle[0] = aziScanScopeLowerBound + DEFAULT_AZI_BEAM_INTER / 2;
	}
	else
	{
		gAziBeamAngle[0] = aziScanScopeLowerBound + aziScanScope / 2;
	}
	gAziBeamNum = (int8_t)(aziScanScope / DEFAULT_AZI_BEAM_INTER);
	if (gAziBeamNum < 1)
		gAziBeamNum = 1;
	for (i = 0; i < gAziBeamNum; i++)
	{
		gAziBeamIndex[i] = i;
		gAziBeamAngle[i] = gAziBeamAngle[0] + (i * DEFAULT_AZI_BEAM_INTER);
		gAziBeamAngleSin[i] = (float)(sinf(1.0 * gAziBeamAngle[i] * DEG2RAD));
		gAziBeamAngleSinInt16[i] = (int16_t)(gAziBeamAngleSin[i] * INT16_SCALE);
	}
	gAziBeamIndexCur = gAziBeamIndex[0];

	/* 2. update elevation beam config params */
	eleScanCenter = param->eleScanCenter;
	if (((param->eleScanScope / 2) + (int8_t)(fabsf(eleScanCenter))) <= DEFAULT_ELE_FOV_UPPER)
	{
		eleScanScope = param->eleScanScope;
	}
	else
	{
		eleScanScope = (DEFAULT_ELE_FOV_UPPER - (int8_t)(fabsf(eleScanCenter))) * 2;
	}

	eleScanScopeLowerBound = eleScanCenter - (eleScanScope / 2);
	if (eleScanScope >= DEFAULT_ELE_BEAM_INTER)
	{
		gEleBeamAngle[0] = eleScanScopeLowerBound + DEFAULT_ELE_BEAM_INTER / 2;
	}
	else
	{
		gEleBeamAngle[0] = eleScanScopeLowerBound + eleScanScope / 2;
	}
	gEleBeamNum = (int8_t)(eleScanScope / DEFAULT_ELE_BEAM_INTER);
	if (gEleBeamNum < 1)
		gEleBeamNum = 1;
	for (i = 0; i < gEleBeamNum; i++)
	{
		gEleBeamIndex[i] = i;
		gEleBeamAngle[i] = gEleBeamAngle[0] + (i * DEFAULT_ELE_BEAM_INTER);
		gEleBeamAngleSin[i] = (float)(sinf(1.0 * gEleBeamAngle[i] * DEG2RAD));
		gEleBeamAngleSinInt16[i] = (int16_t)(gEleBeamAngleSin[i] * INT16_SCALE);
	}
	gEleBeamIndexCur = gEleBeamIndex[0];
}


void DispatchTwsScan(sDispatchInst *dispatch_inst, sTrackingFrame *tracksListOutput)
{
	if (gEleBeamIndexCur < gEleBeamIndex[gEleBeamNum - 1])
	{// elevation beam scan first
		gEleBeamIndexCur++;
	}
	else
	{// elevation beam scan finished, reset gEleBeamIndexCur, and switch to next azimuth direction beam
		gEleBeamIndexCur = gEleBeamIndex[0];
		gAziBeamIndexCur++;
		if (gAziBeamIndexCur > gAziBeamIndex[gAziBeamNum - 1])
		{// azimuth beam scan finished, reset gAziBeamIndexCur, and switch to next azimuth direction beam
			gAziBeamIndexCur = gAziBeamIndex[0];
            if (2 == gBeamMode)
            	gWholeSpaceScanCycleCnt++;	//normal mode 20220916
		}
	}

	dispatch_inst->dispatchOutput->frameID = tracksListOutput->frameID;
	dispatch_inst->dispatchOutput->timestamp = tracksListOutput->timestamp;

	if (2 == gBeamMode)
	{
		dispatch_inst->dispatchOutput->aziBeamID = gAziBeamIndexCur;
		dispatch_inst->dispatchOutput->eleBeamID = gEleBeamIndexCur;
		dispatch_inst->dispatchOutput->aziBeamSin = gAziBeamAngleSinInt16[gAziBeamIndexCur];
		dispatch_inst->dispatchOutput->eleBeamSin = gEleBeamAngleSinInt16[gEleBeamIndexCur];
	}
	else
	{
		dispatch_inst->dispatchOutput->aziBeamID = 15;
		dispatch_inst->dispatchOutput->eleBeamID = 2;
		dispatch_inst->dispatchOutput->aziBeamSin = \
				(int16_t)(sinf(1.0 * dispatch_inst->dispatchConfigParam->aziScanCenter * DEG2RAD) * INT16_SCALE);
		dispatch_inst->dispatchOutput->eleBeamSin = \
				(int16_t)(sinf(1.0 * dispatch_inst->dispatchConfigParam->eleScanCenter * DEG2RAD) * INT16_SCALE);
	}

	dispatch_inst->dispatchOutput->tasBeamTotal = 0;
	dispatch_inst->dispatchOutput->tasBeamCntCur = 0;
	dispatch_inst->dispatchOutput->tasObjId = INVALID_TARGET_ID;
	dispatch_inst->dispatchOutput->tasObjFilterNum = INVALID_FIELD_DATA;
	dispatch_inst->dispatchOutput->tasObjRange = INVALID_FIELD_DATA;
	dispatch_inst->dispatchOutput->samplePntStart = INVALID_FIELD_DATA;
	dispatch_inst->dispatchOutput->samplePntDepth = INVALID_FIELD_DATA;
	dispatch_inst->dispatchOutput->beamSwitchTime = 0;
	dispatch_inst->dispatchOutput->wholeSpaceScanCycleCnt = gWholeSpaceScanCycleCnt;
	dispatch_inst->dispatchOutput->trackTwsTasFlag = TWS_SCAN;

	if ( 1 == gBeamMode)
		gWholeSpaceScanCycleCnt++;	//test mode 20220916

#ifdef DEBUG_LOG_ZXL
	my_printf("********* dispatch ********** azi, id:%d, a:%d aSin:%d,	ele, id:%d, a:%d aSin:%d,	azi:%.3f, ele:%.3f\n", \
		dispatch_inst->dispatchOutput->aziBeamID, gAziBeamAngle[gAziBeamIndexCur], gAziBeamAngleSinInt16[gAziBeamIndexCur], \
		dispatch_inst->dispatchOutput->eleBeamID, gEleBeamAngle[gEleBeamIndexCur], gEleBeamAngleSinInt16[gEleBeamIndexCur], \
		asinf(1.0 * dispatch_inst->dispatchOutput->aziBeamSin / INT16_SCALE) * RAD2DEG, \
		asinf(1.0 * dispatch_inst->dispatchOutput->eleBeamSin / INT16_SCALE) * RAD2DEG);
#endif // DEBUG_LOG_ZXL
}


void DispatchTasScan(sDispatchInst *dispatch_inst, sTrackingFrame *tracksListOutput, sTracking_moduleInstance *inst)
{
	uint16_t i = 0;
	uint16_t uid = 0;
	sTracking_ListElem *tElem = NULL;
	sTracking_objectUnit *pTracker;
	sDispatchOutput dispatchOutputTmp = { 0 };
	float statePredict[SSIZE_XY] = { 0 };
	sTracking_measurementSphUnion trackerPolar;

	/* 1. get tasTrackCurrentCycle in current tas scan cycle */
	if ((gTasScanBeamCnt % TAS_CYCLE_BEAM_NUM) == 0)
	{// start current tas scan cycle
		i = 0;
		dispatch_inst->dispatchTasTrackInfo->tasDispatchFlag = true;
		dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt = 0;
		dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt = 0;
		memset(dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle, 0, sizeof(sDispatchTrackInfo) * TAS_REAL_TARGET_MAX_NUM);

		/* 1.1. get trackTasNum */
		if (TAS_REAL_TARGET_MAX_NUM <= tracking_listGetCount(&inst->tasTrackList))
		{
			dispatch_inst->dispatchTasTrackInfo->trackTasNum = TAS_REAL_TARGET_MAX_NUM;
		}
		else
		{
			dispatch_inst->dispatchTasTrackInfo->trackTasNum = tracking_listGetCount(&inst->tasTrackList);
		}

		/* 1.2. get tasTrackCurrentCycle */
		if (dispatch_inst->dispatchTasTrackInfo->trackTasNum > 0)
		{
			tElem = tracking_listGetFirst(&inst->tasTrackList);
			while (tElem != 0)
			{
				uid = tElem->data;
				pTracker = (sTracking_objectUnit *)inst->hTrack[uid];
				tracking_cal_FQ(inst, g_curTimestamp + DEFAULT_CPI_MS / 1000.f - pTracker->timestamp);
				tracking_matrixMultiply(SSIZE_XY, SSIZE_XY, 1, inst->tracking_params->F, \
					pTracker->kalman_state.S_hat, statePredict);
				tracking_cartesian2spherical(inst->tracking_params->stateVectorType, \
					statePredict, trackerPolar.array);
				dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[i].timestamp = pTracker->timestamp;
				dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[i].id = pTracker->uid;
				dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[i].range = trackerPolar.vector.range;
				dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[i].azimuth = trackerPolar.vector.azimuthRad;
				dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[i].elevation = trackerPolar.vector.pitchRad;
				dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[i].velocity = trackerPolar.vector.doppler;
				tElem = tracking_listGetNext(tElem);
				i++;
			}
		}
	}

	/* 2. continuous 4(down/up/left/right) beam scan for all the trackTasNum tas target, then go on normal search */
	if ((dispatch_inst->dispatchTasTrackInfo->tasDispatchFlag == true) && \
		(dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt < dispatch_inst->dispatchTasTrackInfo->trackTasNum))
	{// 2.1 continuous 4(down/up/left/right) beam scan for all the trackTasNum tas target
		if (dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt < TAS_CYCLE_TOTAL_BEAM_NUM)
		{
			dispatchOutputTmp.aziBeamID = INVALID_FIELD_DATA;
			dispatchOutputTmp.eleBeamID = INVALID_FIELD_DATA;
			dispatchOutputTmp.aziBeamSin = (int16_t)(sinf(dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt].azimuth \
				+ gTasAziOffsetAngle[dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt] * DEG2RAD) * INT16_SCALE);
			dispatchOutputTmp.eleBeamSin = (int16_t)(sinf(dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt].elevation \
				+ gTasEleOffsetAngle[dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt] * DEG2RAD) * INT16_SCALE);
			dispatchOutputTmp.tasBeamTotal = TAS_CYCLE_TOTAL_BEAM_NUM;
			dispatchOutputTmp.tasBeamCntCur = dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt + 1;
			dispatchOutputTmp.tasObjId = dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt].id;
			dispatchOutputTmp.tasObjFilterNum = INVALID_FIELD_DATA;
			dispatchOutputTmp.tasObjRange = (uint16_t)(dispatch_inst->dispatchTasTrackInfo->tasTrackCurrentCycle[dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt].range / DEFAULT_RANGE_UNIT_M);
			dispatchOutputTmp.samplePntStart = DEFAULT_SAMPLE_PNT_START;
			dispatchOutputTmp.samplePntDepth = DEFAULT_SAMPLE_PNT_DEPTH;
			dispatchOutputTmp.beamSwitchTime = DEFAULT_BEAM_SWITCH_TIME;

			dispatch_inst->dispatchOutput->frameID = tracksListOutput->frameID;
			dispatch_inst->dispatchOutput->timestamp = tracksListOutput->timestamp;
			dispatch_inst->dispatchOutput->aziBeamID = dispatchOutputTmp.aziBeamID;
			dispatch_inst->dispatchOutput->eleBeamID = dispatchOutputTmp.eleBeamID;
			dispatch_inst->dispatchOutput->aziBeamSin = dispatchOutputTmp.aziBeamSin;
			dispatch_inst->dispatchOutput->eleBeamSin = dispatchOutputTmp.eleBeamSin;
			dispatch_inst->dispatchOutput->tasBeamTotal = dispatchOutputTmp.tasBeamTotal;
			dispatch_inst->dispatchOutput->tasBeamCntCur = dispatchOutputTmp.tasBeamCntCur;
			dispatch_inst->dispatchOutput->tasObjId = dispatchOutputTmp.tasObjId;
			dispatch_inst->dispatchOutput->tasObjFilterNum = dispatchOutputTmp.tasObjFilterNum;
			dispatch_inst->dispatchOutput->tasObjRange = dispatchOutputTmp.tasObjRange;
			dispatch_inst->dispatchOutput->samplePntStart = dispatchOutputTmp.samplePntStart;
			dispatch_inst->dispatchOutput->samplePntDepth = dispatchOutputTmp.samplePntDepth;
			dispatch_inst->dispatchOutput->beamSwitchTime = dispatchOutputTmp.beamSwitchTime;
			dispatch_inst->dispatchOutput->wholeSpaceScanCycleCnt = gWholeSpaceScanCycleCnt;
			dispatch_inst->dispatchOutput->trackTwsTasFlag = TAS_SCAN;
			dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt++;

			if (dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt == TAS_CYCLE_TOTAL_BEAM_NUM)
			{
				dispatch_inst->dispatchTasTrackInfo->tasScanBeamInCycleCnt = 0;
				dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt++;
				if (dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt == dispatch_inst->dispatchTasTrackInfo->trackTasNum)
				{
					dispatch_inst->dispatchTasTrackInfo->trackTasProcCnt = 0;
					dispatch_inst->dispatchTasTrackInfo->tasDispatchFlag = false;
				}
			}
		}
	}
	else
	{// 2.2 go on normal search
		if (gEleBeamIndexCur < gEleBeamIndex[gEleBeamNum - 1])
		{// elevation beam scan first
			gEleBeamIndexCur++;
		}
		else
		{// elevation beam scan finished, reset gEleBeamIndexCur, and switch to next azimuth direction beam
			gEleBeamIndexCur = gEleBeamIndex[0];
			gAziBeamIndexCur++;
			if (gAziBeamIndexCur > gAziBeamIndex[gAziBeamNum - 1])
			{// azimuth beam scan finished, reset gAziBeamIndexCur, and switch to next azimuth direction beam
				gAziBeamIndexCur = gAziBeamIndex[0];
				gWholeSpaceScanCycleCnt++;
			}
		}

		dispatch_inst->dispatchOutput->frameID = tracksListOutput->frameID;
		dispatch_inst->dispatchOutput->timestamp = tracksListOutput->timestamp;
		dispatch_inst->dispatchOutput->aziBeamID = gAziBeamIndexCur;
		dispatch_inst->dispatchOutput->eleBeamID = gEleBeamIndexCur;
		dispatch_inst->dispatchOutput->aziBeamSin = gAziBeamAngleSinInt16[gAziBeamIndexCur];
		dispatch_inst->dispatchOutput->eleBeamSin = gEleBeamAngleSinInt16[gEleBeamIndexCur];
		dispatch_inst->dispatchOutput->tasBeamTotal = 0;
		dispatch_inst->dispatchOutput->tasBeamCntCur = 0;
		dispatch_inst->dispatchOutput->tasObjId = INVALID_TARGET_ID;
		dispatch_inst->dispatchOutput->tasObjFilterNum = INVALID_FIELD_DATA;
		dispatch_inst->dispatchOutput->tasObjRange = INVALID_FIELD_DATA;
		dispatch_inst->dispatchOutput->samplePntStart = INVALID_FIELD_DATA;
		dispatch_inst->dispatchOutput->samplePntDepth = INVALID_FIELD_DATA;
		dispatch_inst->dispatchOutput->beamSwitchTime = 0;
		dispatch_inst->dispatchOutput->wholeSpaceScanCycleCnt = gWholeSpaceScanCycleCnt;
		dispatch_inst->dispatchOutput->trackTwsTasFlag = TWS_SCAN;
	}

	gTasScanBeamCnt++;
}
