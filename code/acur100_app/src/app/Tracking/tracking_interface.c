
#include "xtime_l.h"
#include "xil_printf.h"
#include "tracking_interface.h"
#include "src/target_process/dispatch/include/dispatch.h"
#include "../app_init.h"
#include "../../app/beam_calculate/bk_cal.h"


protocol_object_list_detected_t g_obj_list_detected_send;
protocol_object_list_tracked_t g_obj_list_tracked_send;
protocol_beam_scheduling_t g_beam_scheduling_send;
protocol_radar_status_t g_radar_status_send;
protocol_beam_scheduling_t gBeamInfoLast[1] = { 0 };

sAlgObjData *gAlgData = NULL;

sDummyRadarMeasUnit dummyData[MAX_NUM_DETECTS];
float curTime_dummy = 0.f;
uint32_t frameID = 0;


void runTrackingAlg()
{
	XTime tCur1;
	XTime tEnd1;
	uint32_t tUsed1;
	uint8_t fre_num = 0;
	int16_t az_deg = 0, al_deg = 0;

	XTime_GetTime(&tCur1);

	if (gAlgInitFlag != 0)
	{
		gAlgData = alg_init();
	}

	memcpy(gBeamInfoLast, gBeamInfo, sizeof(gBeamInfoLast));

#ifdef USING_DUMMY_TEST
	process_dummy3D(gDetectList, gTrackList, gBeamInfo, gPlatformInfo, gRadarStatus);
#else
	trackingAlgProcess(gDetectList, gTrackList, gBeamInfo, gPlatformInfo, gConfigParmInfo, gRadarStatus);
#endif // USING_DUMMY_TEST

	// 计算两维有源阵列的相位码、衰减码
	az_deg = asinf(gBeamInfo[0].aziBeamSin * INV_ONE15FORMAT_SCALE) * RAD2DEG;
	al_deg = asinf(gBeamInfo[0].eleBeamSin * INV_ONE15FORMAT_SCALE) * RAD2DEG;
	bk_cak_func(fre_num, az_deg, al_deg);
	bk_tran_func(TRUE);


	XTime_GetTime(&tEnd1);
	tUsed1 = ((tEnd1 - tCur1) * 1000000) / (COUNTS_PER_SECOND);
	xil_printf("runTrackingAlg time elapsed is %d us\r\n\n", tUsed1);
}


void trackingAlgProcess(protocol_object_list_detected_t *detect_list, \
		protocol_object_list_tracked_t *track_list, \
		protocol_beam_scheduling_t *beam_scheduling, \
		protocol_radar_platfrom_info_t *platformInfo, \
		protocol_cfg_param_t *configParmInfo, \
		protocol_radar_status_t *radar_status)
{
	sDetectFrame *detectsListOutput = gAlgData->detectsListOutput;

	gAlgData->platformInfo->frameID = platformInfo->stInfoHeader.frameID;
	gAlgData->platformInfo->timestamp = platformInfo->stInfoHeader.timestamp;
	memcpy(&gAlgData->platformInfo->platformData, &platformInfo->heading, sizeof(sPlatformData));

	// ctrl info input
	gAlgData->debugAndCtrlParam->frameID = configParmInfo->stInfoHeader.frameID;
	gAlgData->debugAndCtrlParam->timestamp = configParmInfo->stInfoHeader.timestamp;
//	configParmInfo->aziScanScope = 60;
//	configParmInfo->eleScanScope = 20;
	memcpy(&gAlgData->debugAndCtrlParam->debugAndCtrlData, &configParmInfo->trSwitchCtrl, sizeof(sDebugAndCtrlData));

	if (gAlgInitFlag != -1)
	{
		setDebugAndCtrlParam(gAlgData, gAlgData->platformInfo, gAlgData->debugAndCtrlParam);
	}

	detectsListOutput->detectObjNum = detect_list->detectObjNum;
	detectsListOutput->frameID = detect_list->stInfoHeader.frameID;
	detectsListOutput->timestamp = detect_list->stInfoHeader.timestamp;
	detectsListOutput->trackTwsTasFlag = detect_list->trackTwsTasFlag;
	memcpy(detectsListOutput->detPointsList, detect_list->detectPoint, sizeof(sDetectPoint)*detect_list->detectObjNum);

	alg_process(gAlgData);

	trackingReport(detect_list, track_list, beam_scheduling, radar_status);
}


void process_dummy3D(protocol_object_list_detected_t *detect_list, \
		protocol_object_list_tracked_t *track_list, \
		protocol_beam_scheduling_t *beam_scheduling, \
		protocol_radar_platfrom_info_t *platformInfo, \
		protocol_radar_status_t *radar_status)
{
	sDetectFrame *detectsListOutput = gAlgData->detectsListOutput;
	sDetectPoint *detPointsList = gAlgData->detectsListOutput->detPointsList;
	sDispatchInst *dispatch_inst = (sDispatchInst *)gAlgData->dispatch_inst;
	sDispatchOutput *dispatchOutput = dispatch_inst->dispatchOutput;
	int num_dets = 1;//const_nd;  //cdx
	int nd = 0;//[0,num_dets)
	sPlatformInfo dummyPlatformInfo = { 0 };
	sDebugAndCtrlParam dummyCtrlInfo = { 0 };
	dummyCtrlInfo.debugAndCtrlData.workMode = 0;
	float dt = 1. / 120.f;


	detectsListOutput->timestamp = (uint32_t)(curTime_dummy*1000.f);
	curTime_dummy += dt;
	detectsListOutput->trackTwsTasFlag = dispatchOutput->trackTwsTasFlag;
	detectsListOutput->frameID = (uint32_t)(frameID++);

#ifdef DEBUG_LOG
	my_printf("----------frameId %d  time %.2f  scanType %d ---------------", \
		frameID, curTime_dummy, dispatchOutput->trackTwsTasFlag);
#endif // DEBUG_LOG

	//sTARadar_enumType locMode = RIGHT_TARADAR;
	if (gAlgInitFlag != -1) {
		setDebugAndCtrlParam(gAlgData, &dummyPlatformInfo, &dummyCtrlInfo);
	}
	myMath_generateDummyRadarData(dummyData, &num_dets, -3.f, 3.f, 0.f, 0U); // disable stationary target

	//consider the first (seq 0) per 120(1 s)
	float aziBeam = asinf((float)(dispatchOutput->aziBeamSin) / 32767.f)*RAD2DEG;
	float eleBeam = asinf((float)(dispatchOutput->eleBeamSin) / 32767.f)*RAD2DEG;
#ifdef DEBUG_LOG
	my_printf("dispatchInfo: wholeScanNum %d, azi pitch [%.2f %.2f]", \
		dispatchOutput->wholeSpaceScanCycleCnt, aziBeam, eleBeam);
	my_printf("dummy det azi pitch [%.2f %.2f]", dummyData[0].azimuth, fabsf(dummyData[0].pitch));
#endif // DEBUG_LOG

	for (int detId = 0; detId < (int)(num_dets); detId++)//num_dets:1
	{
		if (detId == MAX_NUM_DETECTS) {
			break;
		}
		if (fabsf(dummyData[detId].azimuth - aziBeam) > 3.0f || \
			fabsf(dummyData[detId].pitch - eleBeam) > 6.0f)
			continue;

		detPointsList[nd].id = (uint16_t)(nd);
		detPointsList[nd].range = (uint32_t)(dummyData[detId].range*ONE6FORMAT_SCALE);
		detPointsList[nd].azimuth = (int16_t)(aziBeam*ONE6FORMAT_SCALE);
		detPointsList[nd].elevation = (int16_t)(eleBeam*ONE6FORMAT_SCALE);
		detPointsList[nd].velocity = (int16_t)(dummyData[detId].doppler*ONE6FORMAT_SCALE);
		detPointsList[nd].mag = (uint16_t)((rand()%21+6)*ONE6FORMAT_SCALE);//(uint16_t)(20.f*ONE6FORMAT_SCALE)
		detPointsList[nd].ambiguous = (uint16_t)(0);
		detPointsList[nd].detProperty.classification = (uint8_t)(1);
		detPointsList[nd].detProperty.cohesionOkFlag = (uint8_t)(1);
		detPointsList[nd].detProperty.cohesionPntNum = (uint8_t)(1);
		detPointsList[nd].detProperty.cohesionBeamNum = (uint8_t)(1);
		detPointsList[nd].detProperty.classifyProb = (uint16_t)(1);
		detPointsList[nd].aziBeamID = (uint16_t)(0.f*ONE6FORMAT_SCALE);
		detPointsList[nd].eleBeamID = (uint16_t)(0.f*ONE6FORMAT_SCALE);		//
#ifdef DEBUG_LOG
		my_printf("dummy detId %d, r a p v[%.2f,%.2f,%.2f,%.2f]",\
			nd,
			dummyData[detId].range, dummyData[detId].azimuth,\
			dummyData[detId].pitch, dummyData[detId].doppler);
#endif // DEBUG_LOG
		nd++;

	}
	gAlgData->detectsListOutput->detectObjNum = nd;//(uint16_t)(num_masterPoints);
	gAlgData->detectsListOutput->trackTwsTasFlag = 0;

	alg_process(gAlgData);

	trackingReport(detect_list, track_list, beam_scheduling, radar_status);
}


void trackingReport(protocol_object_list_detected_t *detect_list, \
		protocol_object_list_tracked_t *track_list, \
		protocol_beam_scheduling_t *beam_scheduling, \
		protocol_radar_status_t *radar_status)
{
	sDetectFrame *detectsListOutput = gAlgData->detectsListOutput;
	sTrackingInst *tracking_inst = (sTrackingInst *)gAlgData->tracking_inst;
	sTrackingFrame *retTrajs = tracking_inst->trajInfoOutput;
	sDispatchInst *dispatch_inst = (sDispatchInst *)gAlgData->dispatch_inst;
	sDispatchOutput *dispatchOutput = dispatch_inst->dispatchOutput;

	/* 1. detect_list */
	detect_list->stInfoHeader.frameID = detectsListOutput->frameID;
	detect_list->stInfoHeader.timestamp = detectsListOutput->timestamp;
	detect_list->stInfoHeader.infoType = PIT_DET_OBJ;
	detect_list->detectObjNum = detectsListOutput->detectObjNum;
	detect_list->detectObjByte = sizeof(protocol_object_item_detected_t);
	detect_list->trackTwsTasFlag = detectsListOutput->trackTwsTasFlag;
	memcpy(detect_list->detectPoint, detectsListOutput->detPointsList, sizeof(sDetectPoint)*detect_list->detectObjNum);

	/* 2. detect_list */
	track_list->stInfoHeader.frameID = retTrajs->frameID;
	track_list->stInfoHeader.timestamp = retTrajs->timestamp;
	track_list->stInfoHeader.infoType = PIT_TRK_OBJ;
	track_list->trackObjNum = retTrajs->trackObjNum;
	track_list->trackTwsNum = retTrajs->trackTwsNum;
	track_list->trackTasNum = retTrajs->trackTasNum;
	track_list->trackObjByte = sizeof(protocol_object_item_tracked_t);
	track_list->trackTwsTasFlag = retTrajs->trackTwsTasFlag;
	memcpy(track_list->trackPoint, retTrajs->trajList, sizeof(sTrackingTarget)*track_list->trackObjNum);

	/* 3. beam_scheduling */
	beam_scheduling->stInfoHeader.frameID = dispatchOutput->frameID;
	beam_scheduling->stInfoHeader.timestamp = dispatchOutput->timestamp;
	beam_scheduling->stInfoHeader.infoType = PIT_BEAM_SCHEDULING;
	beam_scheduling->aziBeamID = dispatchOutput->aziBeamID;
	beam_scheduling->eleBeamID = dispatchOutput->eleBeamID;
	beam_scheduling->aziBeamSin = dispatchOutput->aziBeamSin;
	beam_scheduling->eleBeamSin = dispatchOutput->eleBeamSin;
	beam_scheduling->tasBeamTotal = dispatchOutput->tasBeamTotal;
	beam_scheduling->tasBeamCntCur = dispatchOutput->tasBeamCntCur;
	beam_scheduling->tasObjId = dispatchOutput->tasObjId;
	beam_scheduling->tasObjFilterNum = dispatchOutput->tasObjFilterNum;
	beam_scheduling->tasObjRange = dispatchOutput->tasObjRange;
	beam_scheduling->samplePntStart = dispatchOutput->samplePntStart;
	beam_scheduling->samplePntDepth = dispatchOutput->samplePntDepth;
	beam_scheduling->beamSwitchTime = dispatchOutput->beamSwitchTime;
	beam_scheduling->wholeSpaceScanCycleCnt = dispatchOutput->wholeSpaceScanCycleCnt;
	beam_scheduling->trackTwsTasFlag = dispatchOutput->trackTwsTasFlag;

	/* 4. radar_status */
	radar_status->stInfoHeader.frameID = dispatchOutput->frameID;
	radar_status->stInfoHeader.timestamp = dispatchOutput->timestamp;
	radar_status->stInfoHeader.infoType = PIT_RADAR_STATUS;
	radar_status->isFailFlag = 0;
	radar_status->failBitData1 = 0;
	radar_status->failBitData2 = 0;
	radar_status->batteryPower = (uint16_t)((90 + 10 * (rand()*1.0f / RAND_MAX)) * ONE7FORMAT_SCALE);

	trans_byte_order_obj_list_detected(detect_list);
	trans_byte_order_obj_list_tracked(track_list);
//	trans_byte_order_beam_scheduling(beam_scheduling);
	trans_byte_order_beam_scheduling(gBeamInfoLast);	//use gBeamInfoLast
	trans_byte_order_radar_status(radar_status);

	protocol_send_object_list_detected(&g_obj_list_detected_send);
	protocol_send_object_list_tracked(&g_obj_list_tracked_send);
	protocol_send_beam_scheduling(&g_beam_scheduling_send);
	protocol_send_radar_status(&g_radar_status_send);
}


void trans_byte_order_obj_list_detected(protocol_object_list_detected_t *detect_list)
{
	uint16_t crc = 0;
	uint32_t i = 0;

	memcpy(&g_obj_list_detected_send, detect_list, sizeof(g_obj_list_detected_send));
	g_obj_list_detected_send.stInfoHeader.infoSync = htonl(INFO_HEAD_FLAG);
	g_obj_list_detected_send.stInfoHeader.infoLength = htonl(sizeof(g_obj_list_detected_send));
	g_obj_list_detected_send.stInfoHeader.frameID = htonl(detect_list->stInfoHeader.frameID);
	g_obj_list_detected_send.stInfoHeader.timestamp = htonl(detect_list->stInfoHeader.timestamp);
	g_obj_list_detected_send.stInfoHeader.infoType = htons(detect_list->stInfoHeader.infoType);
	g_obj_list_detected_send.stInfoHeader.terminalID = htons(detect_list->stInfoHeader.terminalID);
	g_obj_list_detected_send.detectObjNum = htons(detect_list->detectObjNum);
	g_obj_list_detected_send.detectObjByte = htons(detect_list->detectObjByte);
	g_obj_list_detected_send.trackTwsTasFlag = htons(detect_list->trackTwsTasFlag);
	for (i = 0; i < detect_list->detectObjNum; i++)
	{
		g_obj_list_detected_send.detectPoint[i].id = htons(detect_list->detectPoint[i].id);
		g_obj_list_detected_send.detectPoint[i].azimuth = htons(detect_list->detectPoint[i].azimuth);
		g_obj_list_detected_send.detectPoint[i].range = htonl(detect_list->detectPoint[i].range);
		g_obj_list_detected_send.detectPoint[i].elevation = htons(detect_list->detectPoint[i].elevation);
		g_obj_list_detected_send.detectPoint[i].velocity = htons(detect_list->detectPoint[i].velocity);
		g_obj_list_detected_send.detectPoint[i].dopplerChn = htons(detect_list->detectPoint[i].dopplerChn);
		g_obj_list_detected_send.detectPoint[i].mag = htons(detect_list->detectPoint[i].mag);
		g_obj_list_detected_send.detectPoint[i].objConfidence = htons(detect_list->detectPoint[i].objConfidence);
		g_obj_list_detected_send.detectPoint[i].aziBeamID = htons(detect_list->detectPoint[i].aziBeamID);
		g_obj_list_detected_send.detectPoint[i].eleBeamID = htons(detect_list->detectPoint[i].eleBeamID);
	}
	crc = crc_16bits_compute((uint8_t *)&g_obj_list_detected_send, sizeof(g_obj_list_detected_send) - sizeof(g_obj_list_detected_send.stInfoTail));
	g_obj_list_detected_send.stInfoTail.crc = htons(crc);
}


void trans_byte_order_obj_list_tracked(protocol_object_list_tracked_t *track_list)
{
	uint16_t crc = 0;
	uint32_t i = 0;

	memcpy(&g_obj_list_tracked_send, track_list, sizeof(g_obj_list_tracked_send));
	g_obj_list_tracked_send.stInfoHeader.infoSync = htonl(INFO_HEAD_FLAG);
	g_obj_list_tracked_send.stInfoHeader.infoLength = htonl(sizeof(g_obj_list_tracked_send));
	g_obj_list_tracked_send.stInfoHeader.frameID = htonl(track_list->stInfoHeader.frameID);
	g_obj_list_tracked_send.stInfoHeader.timestamp = htonl(track_list->stInfoHeader.timestamp);
	g_obj_list_tracked_send.stInfoHeader.infoType = htons(track_list->stInfoHeader.infoType);
	g_obj_list_tracked_send.stInfoHeader.terminalID = htons(track_list->stInfoHeader.terminalID);
	g_obj_list_tracked_send.trackObjNum = htons(track_list->trackObjNum);
	g_obj_list_tracked_send.trackTwsNum = htons(track_list->trackTwsNum);
	g_obj_list_tracked_send.trackTasNum = htons(track_list->trackTasNum);
	g_obj_list_tracked_send.trackObjByte = htons(track_list->trackObjByte);
	g_obj_list_tracked_send.trackTwsTasFlag = htons(track_list->trackTwsTasFlag);
	for (i = 0; i < track_list->trackObjNum; i++)
	{
		g_obj_list_tracked_send.trackPoint[i].id = htons(track_list->trackPoint[i].id);
		g_obj_list_tracked_send.trackPoint[i].azimuth = htons(track_list->trackPoint[i].azimuth);
		g_obj_list_tracked_send.trackPoint[i].range = htonl(track_list->trackPoint[i].range);
		g_obj_list_tracked_send.trackPoint[i].elevation = htons(track_list->trackPoint[i].elevation);
		g_obj_list_tracked_send.trackPoint[i].velocity = htons(track_list->trackPoint[i].velocity);
		g_obj_list_tracked_send.trackPoint[i].dopplerChn = htons(track_list->trackPoint[i].dopplerChn);
		g_obj_list_tracked_send.trackPoint[i].mag = htons(track_list->trackPoint[i].mag);
		g_obj_list_tracked_send.trackPoint[i].absVel = htons(track_list->trackPoint[i].absVel);
		g_obj_list_tracked_send.trackPoint[i].orientationAngle = htons(track_list->trackPoint[i].orientationAngle);
		g_obj_list_tracked_send.trackPoint[i].alive = htons(track_list->trackPoint[i].alive);
		g_obj_list_tracked_send.trackPoint[i].twsTasFlag = htons(track_list->trackPoint[i].twsTasFlag);
		g_obj_list_tracked_send.trackPoint[i].x = htonl(track_list->trackPoint[i].x);
		g_obj_list_tracked_send.trackPoint[i].y = htonl(track_list->trackPoint[i].y);
		g_obj_list_tracked_send.trackPoint[i].z = htonl(track_list->trackPoint[i].z);
		g_obj_list_tracked_send.trackPoint[i].vx = htons(track_list->trackPoint[i].vx);
		g_obj_list_tracked_send.trackPoint[i].vy = htons(track_list->trackPoint[i].vy);
		g_obj_list_tracked_send.trackPoint[i].vz = htons(track_list->trackPoint[i].vz);
		g_obj_list_tracked_send.trackPoint[i].ax = htons(track_list->trackPoint[i].ax);
		g_obj_list_tracked_send.trackPoint[i].ay = htons(track_list->trackPoint[i].ay);
		g_obj_list_tracked_send.trackPoint[i].az = htons(track_list->trackPoint[i].az);
		g_obj_list_tracked_send.trackPoint[i].x_variance = htons(track_list->trackPoint[i].x_variance);
		g_obj_list_tracked_send.trackPoint[i].y_variance = htons(track_list->trackPoint[i].y_variance);
		g_obj_list_tracked_send.trackPoint[i].z_variance = htons(track_list->trackPoint[i].z_variance);
		g_obj_list_tracked_send.trackPoint[i].vx_variance = htons(track_list->trackPoint[i].vx_variance);
		g_obj_list_tracked_send.trackPoint[i].vy_variance = htons(track_list->trackPoint[i].vy_variance);
		g_obj_list_tracked_send.trackPoint[i].vz_variance = htons(track_list->trackPoint[i].vz_variance);
		g_obj_list_tracked_send.trackPoint[i].ax_variance = htons(track_list->trackPoint[i].ax_variance);
		g_obj_list_tracked_send.trackPoint[i].ay_variance = htons(track_list->trackPoint[i].ay_variance);
		g_obj_list_tracked_send.trackPoint[i].az_variance = htons(track_list->trackPoint[i].az_variance);
		g_obj_list_tracked_send.trackPoint[i].forcastFrameNum = htons(track_list->trackPoint[i].forcastFrameNum);
		g_obj_list_tracked_send.trackPoint[i].associationNum = htons(track_list->trackPoint[i].associationNum);
		g_obj_list_tracked_send.trackPoint[i].assocBit0 = htonl(track_list->trackPoint[i].assocBit0);
		g_obj_list_tracked_send.trackPoint[i].assocBit1 = htonl(track_list->trackPoint[i].assocBit1);
	}
	crc = crc_16bits_compute((uint8_t *)&g_obj_list_tracked_send, sizeof(g_obj_list_tracked_send) - sizeof(g_obj_list_tracked_send.stInfoTail));
	g_obj_list_tracked_send.stInfoTail.crc = htons(crc);
}


void trans_byte_order_beam_scheduling(protocol_beam_scheduling_t *beam_scheduling)
{
	uint16_t crc = 0;

	memcpy(&g_beam_scheduling_send, beam_scheduling, sizeof(g_beam_scheduling_send));
	g_beam_scheduling_send.stInfoHeader.infoSync = htonl(INFO_HEAD_FLAG);
	g_beam_scheduling_send.stInfoHeader.infoLength = htonl(sizeof(g_beam_scheduling_send));
	g_beam_scheduling_send.stInfoHeader.frameID = htonl(beam_scheduling->stInfoHeader.frameID);
	g_beam_scheduling_send.stInfoHeader.timestamp = htonl(beam_scheduling->stInfoHeader.timestamp);
	g_beam_scheduling_send.stInfoHeader.infoType = htons(beam_scheduling->stInfoHeader.infoType);
	g_beam_scheduling_send.stInfoHeader.terminalID = htons(beam_scheduling->stInfoHeader.terminalID);
	g_beam_scheduling_send.aziBeamID = htons(beam_scheduling->aziBeamID);
	g_beam_scheduling_send.eleBeamID = htons(beam_scheduling->eleBeamID);
	g_beam_scheduling_send.aziBeamSin = htons(beam_scheduling->aziBeamSin);
	g_beam_scheduling_send.eleBeamSin = htons(beam_scheduling->eleBeamSin);
	g_beam_scheduling_send.tasBeamTotal = htons(beam_scheduling->tasBeamTotal);
	g_beam_scheduling_send.tasBeamCntCur = htons(beam_scheduling->tasBeamCntCur);
	g_beam_scheduling_send.tasObjId = htons(beam_scheduling->tasObjId);
	g_beam_scheduling_send.tasObjFilterNum = htons(beam_scheduling->tasObjFilterNum);
	g_beam_scheduling_send.tasObjRange = htons(beam_scheduling->tasObjRange);
	g_beam_scheduling_send.samplePntStart = htons(beam_scheduling->samplePntStart);
	g_beam_scheduling_send.samplePntDepth = htons(beam_scheduling->samplePntDepth);
	g_beam_scheduling_send.beamSwitchTime = htons(beam_scheduling->beamSwitchTime);
	g_beam_scheduling_send.wholeSpaceScanCycleCnt = htonl(beam_scheduling->wholeSpaceScanCycleCnt);
	g_beam_scheduling_send.trackTwsTasFlag = htons(beam_scheduling->trackTwsTasFlag);

	crc = crc_16bits_compute((uint8_t *)&g_beam_scheduling_send, sizeof(g_beam_scheduling_send) - sizeof(g_beam_scheduling_send.stInfoTail));
	g_beam_scheduling_send.stInfoTail.crc = htons(crc);
}


void trans_byte_order_radar_status(protocol_radar_status_t *radar_status)
{
	uint16_t crc = 0;

	memcpy(&g_radar_status_send, radar_status, sizeof(g_radar_status_send));
	g_radar_status_send.stInfoHeader.infoSync = htonl(INFO_HEAD_FLAG);
	g_radar_status_send.stInfoHeader.infoLength = htonl(sizeof(g_radar_status_send));
	g_radar_status_send.stInfoHeader.frameID = htonl(radar_status->stInfoHeader.frameID);
	g_radar_status_send.stInfoHeader.timestamp = htonl(radar_status->stInfoHeader.timestamp);
	g_radar_status_send.stInfoHeader.infoType = htons(radar_status->stInfoHeader.infoType);
	g_radar_status_send.stInfoHeader.terminalID = htons(radar_status->stInfoHeader.terminalID);
	g_radar_status_send.isFailFlag = htons(radar_status->isFailFlag);
	g_radar_status_send.failBitData1 = htons(radar_status->failBitData1);
	g_radar_status_send.failBitData2 = htons(radar_status->failBitData2);
	g_radar_status_send.batteryPower = htons(radar_status->batteryPower);

	crc = crc_16bits_compute((uint8_t *)&g_radar_status_send, sizeof(g_radar_status_send) - sizeof(g_beam_scheduling_send.stInfoTail));
	g_radar_status_send.stInfoTail.crc = htons(crc);
}

