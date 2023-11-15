
#include "../include/measurementProcess.h"
#include"../../dispatch/include/dispatch.h"

// Buffer initialization


sMultiBeamMeas gMultiBeamMeas[1] = { 0 };
sClusterDetect gClusterDet[1] = { 0 };
sClusterMeasure gClusterMeas[1] = { 0 };
sDetTrackFrame gTrackDetStr[1] = { 0 };
sDetectFrame gDetObjOut[1] = { 0 };
sMeasOutput gClusterObj[1] = { 0 };

float rangeThershold[CLUTTER_PRO_CASE] = { 37.5f,142.5f,202.5f,300.0f,500.0f,1000.0f,1500.0f,2000.0f,2500.0f,3000.0f };
//float magThershold[CLUTTER_PRO_CASE + 1] = { 60.0f,39.0f,33.0f,30.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f };
//float rangeThershold[CLUTTER_PRO_CASE] = { 39.0f,100.0f,220.0f,600.0f,800.0f,1000.0f,1500.0f,2000.0f,2500.0f,3000.0f };
//float magThershold[CLUTTER_PRO_CASE + 1] = { 35.0f,25.0f,20.0f,10.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f };//1019 OK
float magThershold[CLUTTER_PRO_CASE + 1] = { 30.0f,22.0f,18.0f,9.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,1.0f };//1020test



void measurement_process(sAlgObjData* algObj)
{

    //scheduling mode
    if (algObj->scanType == TAS_SCAN)
    {
        tas_measurement_process(algObj);
    }
    else
    {
        tws_measurement_process(algObj);
    }

}


////** TAS mode **//
//void tas_measurement_process(sAlgObjData* algObj)
//{
//
//
//}



//** TAS mode **//
void tas_measurement_process(sAlgObjData* algObj)
{
    uint8_t i_tas, add;
    uint8_t numTas; //number of clustering targets in cache TWS
    uint16_t tasObjId;			// index of tas object, only valid for tas scan mode
    uint8_t cacheBeamTarNum[COCH_BEAM_NUM_TAS] = { 0 };
    sDetectFrame* detClutProOut = (sDetectFrame*)gDetObjOut;//�Ӳ������������ṹ��
    sMultiBeamMeas* cacheBeam = (sMultiBeamMeas*)gMultiBeamMeas;//���Ŀ�껺��ṹ��
    sDetectFrame* detInst = (sDetectFrame*)algObj->detectsListOutput;//����Ĳ�λ�����Ϣ
    sDispatchInst* dispatchInst = (sDispatchInst*)algObj->dispatch_inst;//����Ĳ�λ������Ϣ
    sDispatchOutput* dispatchOutput = dispatchInst->dispatchOutput;
    sMeasProcessInst* measInst = (sMeasProcessInst*)algObj->measurementProcess_inst;//��λ�㼣��Ϣ�洢��   
    sTrackingInst* tracking_inst = (sTrackingInst*)(algObj->tracking_inst);//������Ϣ
    sTrackingTarget* tasTrackStr;
    sDetTrackFrame* TrackDetStr = gTrackDetStr;//TASĿ��DBScan����ɸѡ�ļ���ṹ��
    sDetTrackFrame* cacheTas;
    sMeasOutput* measObjOut;

    measObjOut = measInst->measOutput;//�㼣��������ṹ��

    memset(TrackDetStr, 0, sizeof(sDetTrackFrame)); /* ����Ŀ��������ṹ���ʼ�� */
    memset(measObjOut, 0, sizeof(sMeasOutput));     /* ����ṹ���ʼ�� */

    //*** ״̬������ֵ ***//
    measObjOut->scanType = TAS_SCAN;// dispatchOutput->trackTwsTasFlag;//TWS TASģʽ
    measObjOut->tasTargetId = dispatchOutput->tasObjId;
    measObjOut->timestamp = (float)(detInst->timestamp) / 1000.f;//ms to s
    measObjOut->condenceDone = 0;//��ʼ������δ���

    for (i_tas = 0; i_tas < detInst->detectObjNum; i_tas++)
    {
#ifdef DEBUG_LOG_HXJ
        my_printf("111111111-TAS-111111111 detNum[ %d ], detId[ %d ], r v a p m[ %.5f,%.5f,%.5f,%.5f,%.5f ]\n", \
            detInst->detectObjNum, detInst->detPointsList[i_tas].id,
            detInst->detPointsList[i_tas].range * INV_ONE6FORMAT_SCALE, detInst->detPointsList[i_tas].velocity * INV_ONE6FORMAT_SCALE, \
            detInst->detPointsList[i_tas].azimuth * INV_ONE6FORMAT_SCALE * DEG2RAD, detInst->detPointsList[i_tas].elevation * INV_ONE6FORMAT_SCALE * DEG2RAD, \
            detInst->detPointsList[i_tas].mag * INV_ONE6FORMAT_SCALE);
#endif // DEBUG_LOG_HXJ
    }

     //*** 0���Ӳ����� ***//
    clutterProcessing(detInst, detClutProOut);

    //*** 1.��ȡTAS����Ŀ����Ϣ ***//
    tasObjId = dispatchOutput->tasObjId;
    tasTrackStr = &tracking_inst->trajInfoOutput->trajList[tasObjId];//TASĿ����ٽṹ��

    //*** 2����λ�ڵ㼣��ɸ+�洢 ***//
    detChoseWithinBeam_tas(detClutProOut, tasTrackStr, cacheBeam);//clusterObjIn

    //*** 3.����TAS���в�λ����в�λ�����۴��� ***//
    numTas = cacheBeam->numTas;
    if (numTas != COCH_BEAM_NUM_TAS)
    {
        return;
    }
    else
    {      
        measObjOut->condenceDone = 1;//TAS���в�λ���������

        cacheTas = (sDetTrackFrame*)cacheBeam->multiBeam_tas;
        add = 0;
        for (i_tas = 0; i_tas < COCH_BEAM_NUM_TAS; i_tas++)//the last one is the courrent beam
        {
            cacheBeamTarNum[i_tas] = cacheTas[i_tas].detectFrame.detectObjNum;
            memcpy(&TrackDetStr->detectFrame.detPointsList[add], cacheTas[i_tas].detectFrame.detPointsList, cacheBeamTarNum[i_tas] * sizeof(sDetectPoint));
            TrackDetStr->aziBeamSin[i_tas] = cacheTas[i_tas].aziBeamSin[0];
            TrackDetStr->eleBeamSin[i_tas] = cacheTas[i_tas].eleBeamSin[0];
                      
            add = add + cacheBeamTarNum[i_tas];
        }
        TrackDetStr->detectFrame.detectObjNum = add;//TASĿ��4��λ�ɾ����������
        TrackDetStr->detectFrame.frameID = detClutProOut->frameID;
        TrackDetStr->detectFrame.timestamp = detClutProOut->timestamp;
        TrackDetStr->detectFrame.trackTwsTasFlag = detClutProOut->trackTwsTasFlag;
        TrackDetStr->detectFrame.detectObjByte = detClutProOut->detectObjByte;

        //TAS���в�λ�ɾ���Ŀ����о��ദ��
        if (TrackDetStr->detectFrame.detectObjNum < 1)
        {
            return;
        }
        else
        { 
             detClusteringBetweenBeam_tas(TrackDetStr, tasTrackStr, cacheBeamTarNum, measObjOut);
        }
    }

    for (i_tas = 0; i_tas < measObjOut->num; i_tas++)
    {
#ifdef DEBUG_LOG_HXJ
        my_printf("22222222-TAS-2222222222 measNum[ %d ], measId[ %d ],condenceDone[ %d ], r v a p m[ %.5f,%.5f,%.5f,%.5f,%.5f ]\n", \
            measObjOut->num, measObjOut->measurement[i_tas].detId, measObjOut->condenceDone,
            measObjOut->measurement[i_tas].vector.range, measObjOut->measurement[i_tas].vector.doppler, \
            measObjOut->measurement[i_tas].vector.azimuthRad, measObjOut->measurement[i_tas].vector.pitchRad, \
            measObjOut->measurement[i_tas].mag);
#endif // DEBUG_LOG_HXJ
    }

}


//** TWS mode **//
void tws_measurement_process(sAlgObjData *algObj)
{
    uint8_t i_tws, aziGap = 0, eleGap = 0;
    uint8_t clusterTarNum, cacheTarNum;
    uint8_t aziComplete = 0, eleComplete = 0;
    uint8_t numTws; //number of clustering targets in cache TWS
    uint8_t calcAngleMode = 0;//���ģʽ��1��λ��2����
    int16_t aziBeamSin_meas = 0, eleBeamSin_meas = 0;
	sDetectFrame* detClutProOut = (sDetectFrame*)gDetObjOut;//�Ӳ������������ṹ��
    sMeasOutput* clusterObjIn = (sMeasOutput*)gClusterObj;//��λ���������Ŀ��ṹ��
    sMultiBeamMeas* cacheBeam = (sMultiBeamMeas*)gMultiBeamMeas;//���Ŀ�껺��ṹ��
    sMeasOutput* measObjOut;
    sMeasOutput* cacheMeasBeam;
    sMeasurement_SphPoint* cacheMeasStr;
    sDetectFrame* detInst = (sDetectFrame*)algObj->detectsListOutput;//����Ĳ�λ�����Ϣ
	sDispatchInst* dispatchInst = (sDispatchInst*)algObj->dispatch_inst;//����Ĳ�λ�����Ϣ
	//sDispatchOutput *dispatchOutput = dispatchInst->dispatchOutput;
    sMeasProcessInst* measInst = (sMeasProcessInst*)algObj->measurementProcess_inst;//��λ�㼣��Ϣ�洢��   
    measObjOut = measInst->measOutput;//�㼣��������ṹ��

    memset(measObjOut, 0, sizeof(sMeasOutput));/* ����ṹ���ʼ�� */

     for (i_tws = 0; i_tws < detInst->detectObjNum; i_tws++)
    {
#ifdef DEBUG_LOG_HXJ
        my_printf("111111111-TWS-111111111 frameId %d, detNum[ %d ], detId[ %d ], r v a p m[ %.5f,%.5f,%.5f,%.5f,%.5f ],rSNR %d, dSNR %d\n", \
            detInst->frameID, detInst->detectObjNum, detInst->detPointsList[i_tws].id,
            detInst->detPointsList[i_tws].range * INV_ONE6FORMAT_SCALE, detInst->detPointsList[i_tws].velocity * INV_ONE6FORMAT_SCALE, \
            detInst->detPointsList[i_tws].azimuth * INV_ONE6FORMAT_SCALE * DEG2RAD, detInst->detPointsList[i_tws].elevation * INV_ONE6FORMAT_SCALE * DEG2RAD, \
            detInst->detPointsList[i_tws].mag * INV_ONE6FORMAT_SCALE, detInst->detPointsList[i_tws].detProperty.classification, detInst->detPointsList[i_tws].detProperty.cohesionOkFlag);
#endif // DEBUG_LOG_HXJ
    }

    //*** 0���Ӳ����� ***//
    clutterProcessing(detInst, detClutProOut);

    //*** 1����λ�ڵ㼣���� ***//
    clusteringWithinBeam_tws(detClutProOut, cacheBeam, clusterObjIn);//detInst

    memcpy(measObjOut, clusterObjIn, sizeof(sMeasOutput));/* ����ṹ�帳ֵ��ǰ��λ��Ϣ */

    //*** 2�������洢������������ṹ�� ***//
    clusterTarNum = clusterObjIn->num;
    numTws = cacheBeam->numTws;
    if (clusterTarNum > 0)
    {          
        if (numTws > 1)
        {   
            for ( i_tws = 0; i_tws < numTws-1; i_tws++)//the last one is the courrent beam
            {              
                cacheMeasBeam = &cacheBeam->multiBeam_tws[i_tws];
                cacheTarNum = cacheMeasBeam->num;

                if (cacheTarNum > 0)
                {
                    cacheMeasStr = &cacheMeasBeam->measurement[0];
                    aziBeamSin_meas = clusterObjIn->measurement[0].aziBeamSin;
                    eleBeamSin_meas = clusterObjIn->measurement[0].eleBeamSin;
                   
                     if ((aziBeamSin_meas != cacheMeasStr->aziBeamSin) && (eleBeamSin_meas == cacheMeasStr->eleBeamSin))
                    {
                        calcAngleMode = 1;//�ⷽλ��
                        //clusteringBetweenBeam_tws(calcAngleMode, cacheMeasBeam, clusterObjIn, measObjOut);
                        clusteringBetweenBeam_tws(calcAngleMode, cacheMeasBeam, measObjOut);
                        aziComplete = 1;
                        aziGap = (numTws - 1 - i_tws);
#ifdef DEBUG_LOG_HXJ
                       /* my_printf("111111111-Azi-111111111 frame_gap[ %d ],calcAngleMode[ %d ]\n", \
                            (numTws-1-i_tws), calcAngleMode);*/
#endif // DEBUG_LOG_HXJ
                    }
                    else if ((aziBeamSin_meas == cacheMeasStr->aziBeamSin) && (eleBeamSin_meas != cacheMeasStr->eleBeamSin))
                    {
                         if (i_tws == numTws - 2)
                         {
                             calcAngleMode = 2;//�⸩����
                             //clusteringBetweenBeam_tws(calcAngleMode, cacheMeasBeam, clusterObjIn, measObjOut);
                             clusteringBetweenBeam_tws(calcAngleMode, cacheMeasBeam, measObjOut);
                             eleComplete = 1;
                             eleGap = (numTws - 1 - i_tws);
#ifdef DEBUG_LOG_HXJ
                            /* my_printf("111111111-Ele-111111111 frame_gap[ %d ],calcAngleMode[ %d ]\n", \
                             (numTws - 1 - i_tws), calcAngleMode);*/
#endif // DEBUG_LOG_HXJ
                         }
                    }

                }
            }
        }
    }

  
    //*** ״̬������ֵ ***//
    measObjOut->scanType = TWS_SCAN;//TWS TASģʽ
    measObjOut->tasTargetId = -1;
    measObjOut->timestamp = (float)(detInst->timestamp) / 1000.f;//ms to s
    measObjOut->condenceDone = 0;//��ʼ������δ���
    //������ɱ�־
    if (aziComplete + eleComplete == 2) 
    {
        measObjOut->condenceDone = 1;
    }
#ifdef DEBUG_LOG_ZQ
	my_printf("frameId %d measOut num %d conde %d  r a p v [%.2f %.2f %.2f %.2f]", \
		detInst->frameID, measObjOut->num, \
		measObjOut->condenceDone, measObjOut->measurement[0].vector.range, \
		measObjOut->measurement[0].vector.azimuthRad*RAD2DEG, \
		measObjOut->measurement[0].vector.pitchRad*RAD2DEG, \
		measObjOut->measurement[0].vector.doppler);
#endif // DEBUG_LOG_HXJ

    for (i_tws = 0; i_tws < measObjOut->num; i_tws++)
    {
#ifdef DEBUG_LOG_HXJ
        my_printf("22222222-TWS-2222222222 frameId %d, measNum[ %d ],measId[ %d ],numTws[ %d ],aziComp[ %d ],eleComp[ %d ],condenceDone[ %d ], r v a p m[ %.5f,%.5f,%.5f,%.5f,%.5f ],detNum[ %d ],detID[ %d,%d,%d,%d,%d,%d,%d,%d ]\n", \
            detInst->frameID, measObjOut->num, measObjOut->measurement[i_tws].detId, numTws, aziGap, eleGap, measObjOut->condenceDone,
            measObjOut->measurement[i_tws].vector.range, measObjOut->measurement[i_tws].vector.doppler, \
            measObjOut->measurement[i_tws].vector.azimuthRad, measObjOut->measurement[i_tws].vector.pitchRad, \
            measObjOut->measurement[i_tws].mag, measObjOut->measurement[i_tws].detectionNum, measObjOut->measurement[i_tws].detectionId[0], measObjOut->measurement[i_tws].detectionId[1], measObjOut->measurement[i_tws].detectionId[2], \
            measObjOut->measurement[i_tws].detectionId[3], measObjOut->measurement[i_tws].detectionId[4], measObjOut->measurement[i_tws].detectionId[5], measObjOut->measurement[i_tws].detectionId[6], measObjOut->measurement[i_tws].detectionId[7]);
#endif // DEBUG_LOG_HXJ
    }

}



//**** �Ӳ�����ģ�� ****//
void clutterProcessing(sDetectFrame* detInst, sDetectFrame* detClutProOut)
{
    uint8_t n_det, n_out;
    sDetectPoint* detPointStr = detInst->detPointsList;
    uint8_t numPointIn;
    float range,mag;
    float mag_thershold;
	float doppler = 0.f;
	float azimuth = 0.f;
    numPointIn = detInst->detectObjNum;
    memset(detClutProOut, 0, sizeof(sDetectFrame));
    
    n_out = 0;
    for (n_det = 0; n_det < numPointIn; n_det++)
    {
        range = detPointStr[n_det].range * INV_ONE6FORMAT_SCALE;
        mag = detPointStr[n_det].mag * INV_ONE6FORMAT_SCALE;
		doppler = detPointStr[n_det].velocity* INV_ONE6FORMAT_SCALE;
		azimuth = detPointStr[n_det].azimuth*INV_ONE6FORMAT_SCALE;
        mag_thershold = ((range <= rangeThershold[0]) ? magThershold[0] : ((range <= rangeThershold[1]) ? magThershold[1] : ((range <= rangeThershold[2]) ? magThershold[2] : ((range <= rangeThershold[3]) ? magThershold[3] : ((range <= rangeThershold[4]) ? magThershold[4] : ((range <= rangeThershold[5]) ? magThershold[5] : ((range <= rangeThershold[6]) ? magThershold[6] : ((range <= rangeThershold[7]) ? magThershold[7] : ((range <= rangeThershold[8]) ? magThershold[8] : ((range <= rangeThershold[9]) ? magThershold[9] : ((range <= rangeThershold[10]) ? magThershold[10] : magThershold[11])))))))))));
       // TODO: supress clutter
		if (fabsf(doppler) > 5.3f || ((range>228)&&(range<241)&&fabsf(doppler)>3.5f&&fabsf(doppler)<5.5f))
			continue;
		if (range > 292.f && range < 298.f)
			continue;
		// TODO: supress clutter
//		if (fabsf(azimuth) > 7.f)
		if (fabsf(azimuth) > 15.f)
			continue;
		if (isReflectPoint(range, doppler, mag, n_det, detPointStr, numPointIn))
			continue;
		if (isPowerSupplyPoint(range, doppler, mag, n_det, detPointStr, numPointIn))
		{
#ifdef DEBUG_LOG
			my_printf("power point %d", detPointStr[n_det].id);
#endif // DEBUG_ZQ

			continue;
		}
			
        if (mag >= mag_thershold)
        {
            memcpy(&detClutProOut->detPointsList[n_out], &detPointStr[n_det], sizeof(sDetectPoint));
            n_out++;
        }
    }

    detClutProOut->detectObjNum = n_out;
    detClutProOut->detectObjByte = detInst->detectObjByte;
    detClutProOut->frameID = detInst->frameID;
    detClutProOut->timestamp = detInst->timestamp;
    detClutProOut->trackTwsTasFlag = detInst->trackTwsTasFlag;


}
uint8_t isReflectPoint(float range, float doppler, float mag,uint8_t n_det, sDetectPoint* detPointStr,uint8_t nNum)
{
	uint8_t iDet = 0;
	float range0, mag0;
	float doppler0 = 0.f;
	uint8_t reflectFlag = 0;
	uint8_t multiRefNum = 11;
	uint8_t iReflect = 2;
	float dopplerAmbi = 0.f;
	float dopplerScope = 12.8823f;
	int n = 0;
	for (iDet = 0; iDet < nNum; iDet++)
	{
		range0 = detPointStr[iDet].range * INV_ONE6FORMAT_SCALE;
		mag0 = detPointStr[iDet].mag * INV_ONE6FORMAT_SCALE;
		doppler0 = detPointStr[iDet].velocity* INV_ONE6FORMAT_SCALE;
		if (iDet == n_det)
			continue;
		if (range0 > range)
			continue;
		for (iReflect = 2; iReflect < multiRefNum; iReflect++)
		{
			n = floor(((float)(iReflect)*doppler0 + 6.649f) / dopplerScope);
			if (n == 0)
			{
				dopplerAmbi = (float)(iReflect)*doppler0;
			}
			else
			{
				dopplerAmbi = (float)(iReflect)*doppler0 - (float)(n)*dopplerScope;
			}
			if (fabsf((float)iReflect*range0 - range) < 5.f+(iReflect-2.f)*2.f&& \
				fabsf(dopplerAmbi - doppler) < 0.5f + (iReflect - 2.f)*0.1f&& \
				mag0 > mag)
			{
				reflectFlag = 1;
				break;
			}
		}
		if (reflectFlag == 1)
			break;
	}
	return reflectFlag;
}

uint8_t isPowerSupplyPoint(float range, float doppler, float mag, uint8_t n_det, sDetectPoint* detPointStr, uint8_t nNum)
{
	uint8_t iDet = 0;
	float range0;
	float doppler0 = 0.f;
	float range304 = 0.f;
	float doppler304 = 0.f;
	float range304_ = 0.f;
	float doppler304_ = 0.f;
	uint8_t powerSupplyFlag = 0;
	float dopplerAmbi = 0.f;
	uint8_t powerSupply304Flag = 0;
	float dopplerScope = 12.8823f;
	int n = 0;
	if (range > 300.f && range < 308.f)
	{
		powerSupplyFlag = 1;
		return powerSupplyFlag;
	}
	for (iDet = 0; iDet < nNum; iDet++)
	{
		range304_ = detPointStr[iDet].range * INV_ONE6FORMAT_SCALE;
		if (range304_ > 300.f&&range304_ < 305.f)
		{
			doppler304_ = detPointStr[iDet].velocity* INV_ONE6FORMAT_SCALE;
			if (fabsf(doppler304_) > fabsf(doppler304))
			{
				doppler304 = doppler304_;
				range304 = range304_;
				powerSupply304Flag = 1;
			}
		}
	}
	if (powerSupply304Flag == 0)
	{
		return powerSupplyFlag;
	}
	if (range > 150.f && powerSupply304Flag == 1)
	{
		for (iDet = 0; iDet < nNum; iDet++)
		{
			range0 = detPointStr[iDet].range * INV_ONE6FORMAT_SCALE;
			doppler0 = detPointStr[iDet].velocity* INV_ONE6FORMAT_SCALE;
			if (iDet == n_det)
				continue;
			if (range0 > range || range0 > 150.f)
				continue;
			n = floor((doppler + doppler0 + 6.649f) / dopplerScope);
			if (n == 0)
			{
				dopplerAmbi = doppler + doppler0;
			}
			else
			{
				dopplerAmbi = doppler + doppler0 - (float)(n)*dopplerScope;
			}

			if ((fabsf(range + range0 - 304.f) < 5.f&&fabsf(dopplerAmbi - doppler304) < 1.f))
			{
				powerSupplyFlag = 1;
				break;
			}
			n = floor((doppler - doppler0 + 6.649f) / dopplerScope);
			if (n == 0)
			{
				dopplerAmbi = doppler - doppler0;
			}
			else
			{
				dopplerAmbi = doppler - doppler0 - (float)(n)*dopplerScope;
			}
			if ((fabsf(range - range0 - 304.f) < 5.f&&fabsf(dopplerAmbi - doppler304) < 0.5f))
			{
				powerSupplyFlag = 1;
				break;
			}
		}
	}
	return powerSupplyFlag;
}
//**** TWSģʽ-��λ�ڵ㼣����ģ�� ****//
void clusteringWithinBeam_tws(sDetectFrame * detClutProOut, sMultiBeamMeas* cacheBeam, sMeasOutput* clusterObjIn)
    {
        uint8_t n_det, i_current, i_neigh, i_beam;
        uint8_t* neighLast;
        uint8_t* neighCurrent;
        uint8_t neighCount, newCount;
        uint8_t clustTarID = 0, lonelyTarID = 0;
        uint8_t numPoints = 0;
        uint8_t visited_in[MAX_NUM_DETECTS] = { 0 };
        uint8_t scope[MAX_NUM_DETECTS] = { 0 };
        uint8_t neighbors[MAX_NUM_DETECTS] = { 0 };
        uint8_t numTWS;//, numTAS
        sDetectPoint* detPointStr = detClutProOut->detPointsList;
        sClusterDetect* clusterDet = gClusterDet;//��λ�ھ���㼣����ṹ��
        sMeasOutput* cacheTws;


        //**** 1��Ŀ�����Թ��� ****//
        numPoints = detClutProOut->detectObjNum;
        memset(clusterDet->clusterID, 0, MAX_NUM_DETECTS * sizeof(uint8_t));
        memset(visited_in, CLUSTER_POINT_UNKNOWN, numPoints * sizeof(uint8_t));

        // Init the clusterID of points are CLUSTER_POINT_LONELY
        for (n_det = 0; n_det < numPoints; n_det++)
        {
            clusterDet->clusterID[n_det] = CLUSTER_POINT_LONELY;
        }
        // scan through all the points to find its neighbors
        for (n_det = 0; n_det < numPoints; n_det++)
        {
            if (visited_in[n_det] != CLUSTER_POINT_VISITED)
            {
                visited_in[n_det] = CLUSTER_POINT_VISITED;

                neighCurrent = neighLast = neighbors;
                // scope is the local copy of visit
                memcpy(scope, visited_in, numPoints * sizeof(uint8_t));

                neighCount = detect_DBscan_findNeighbors(
                    detPointStr, n_det, neighLast, numPoints,
                    scope, &newCount);

#ifdef DEBUG_LOG_HXJ
            /*    my_printf("det[%d]: neightCount: %d\n", \
    				n_det, neighCount);*/
#endif
            /* The cluster consists of the point itself and its neighbours. */
                if (neighCount < MIN_NUM_POINTS_IN_CLUSTER - 1)
                {
                    // This point is lonely point
                    clusterDet->clusterID[n_det] = CLUSTER_POINT_LONELY;
                }
                else
                {
                    // This point belongs to a New Cluster
                    clustTarID++;                                // New cluster ID
                    clusterDet->clusterID[n_det] = clustTarID;      // This point belong to this cluster

                    // tag all the neighbors as visited_in in scope so that it will not be found again when searching neighbor's neighbor.
                    for (i_neigh = 0; i_neigh < newCount; i_neigh++)
                    {
                        i_current = neighLast[i_neigh];//�뵱ǰn_det�����ID
                        scope[i_current] = CLUSTER_POINT_VISITED;//�뵱ǰn_det�����ID���ǩ
                    }
                    neighLast += newCount;//neighLast���ƣ��˴α���������Ŀ������λ

                    while (neighCurrent != neighLast)               // neigh shall be at least minPoints in front of neighborhood pointer
                    {
                        // Explore the neighborhood
                        i_current = *neighCurrent++;               // Take point from the neighborhood
                        clusterDet->clusterID[i_current] = clustTarID; // All points from the neighborhood also belong to this cluster
                        visited_in[i_current] = CLUSTER_POINT_VISITED;

                        neighCount = detect_DBscan_findNeighbors(
                            detPointStr, i_current, neighLast, numPoints,
                            scope, &newCount);

                        if (neighCount >= MIN_NUM_POINTS_IN_CLUSTER - 1)
                        {
                            for (i_neigh = 0; i_neigh < newCount; i_neigh++)
                            {
                                i_current = neighLast[i_neigh];
                                scope[i_current] = CLUSTER_POINT_VISITED;
                            }
                            neighLast += newCount;              /* Member is a core point, and its neighborhood is added to the cluster */
                        }
                    }
                    if (clustTarID >= MAX_NUM_DETECTS)
                    {
                        return;
                    }
                }
            }
        }

        clusterDet->clusterNum = clustTarID;//�ɾ���Ŀ����

        //�µ����
        for ( n_det = 0; n_det < numPoints; n_det++)
        {
            if (clusterDet->clusterID[n_det] == CLUSTER_POINT_LONELY)
            {
                lonelyTarID = lonelyTarID + 1;               
            }
        }

        clusterDet->lonelyNum = lonelyTarID;//�µ�Ŀ��

#ifdef DEBUG_LOG_HXJ
        //my_printf("numPoints clustTarID lonelyTarID [%d %d %d]\n", numPoints, clustTarID, lonelyTarID);
#endif // DEBUG_LOG_HXJ

        //**** 2������Ŀ����о��� + ****//
        clusteringFunInBeam(clusterDet, detClutProOut, clusterObjIn);

        //**** 3����λ��Ϣ�洢 ****//
        if (detClutProOut->trackTwsTasFlag == TWS_SCAN)
        {
            numTWS = cacheBeam->numTws;
            cacheTws = (sMeasOutput*)cacheBeam->multiBeam_tws;
            if (numTWS < COCH_BEAM_NUM_TWS)
            {
                memcpy(&cacheTws[cacheBeam->numTws], clusterObjIn, sizeof(sMeasOutput));
                cacheBeam->numTws++;
            }
            else
            {
                //����8����λ��ǰ4����λֱ�����㣬��4����λ˳�Ƶ�ǰ4����λ���ӵ�5����λ��ʼ��ֵ
                for (i_beam = COCH_BEAM_NUM_TWS/2; i_beam < COCH_BEAM_NUM_TWS; i_beam++)
                {
                    memcpy(&cacheTws[i_beam - COCH_BEAM_NUM_TWS/2], &cacheTws[i_beam], sizeof(sMeasOutput));
                    memset(&cacheTws[i_beam], 0, sizeof(sMeasOutput));
                }
                cacheBeam->numTws = (COCH_BEAM_NUM_TWS/2);
                memcpy(&cacheTws[cacheBeam->numTws], clusterObjIn, sizeof(sMeasOutput));
                cacheBeam->numTws++;
            }
        }      

    }
 

////**** TWSģʽ-��λ��㼣����ģ�� ****//
//void clusteringBetweenBeam_tws(uint8_t calcAngleMode, sMeasOutput* cacheMeasBeam, sMeasOutput* clusterObjIn, sMeasOutput* measObjOut)
//{
//    uint8_t i_curObj, i_cacheObj;
//    uint8_t N_curObj = clusterObjIn->num;//��ǰ��λ�ڵ㼣����
//    uint8_t N_cacheObj = cacheMeasBeam->num; //��������λ�ھ�����Ŀ������
//    sClusterMeasure* clusterMeas = gClusterMeas;//��λ�����㼣����ṹ��
//        
//        
//    //**** 1����ǰ��λ���洢����λ���飺��ǰ��λ+���沨λ ****//
//    memset(clusterMeas, 0, sizeof(sClusterMeasure));
//    for (i_curObj = 0; i_curObj < N_curObj; i_curObj++)
//    {
//        memcpy(&clusterMeas->measurement[i_curObj], &clusterObjIn->measurement[i_curObj], sizeof(sMeasurement_SphPoint));
//    }
//    for (i_cacheObj = 0; i_cacheObj < N_cacheObj; i_cacheObj++)
//    {
//        memcpy(&clusterMeas->measurement[N_curObj+i_cacheObj], &cacheMeasBeam->measurement[i_cacheObj], sizeof(sMeasurement_SphPoint));
//    }
//
//    clusterMeas->clusterMeasNum = N_curObj + N_cacheObj;
//
//
//    //**** 2��DNScan�������鲨λ���о������ ****//
//    cluster_DBscan(clusterMeas, N_curObj);//N_curObjΪ�µ������Χ
//
//
//    //**** 3������Ŀ����о��� + �ȷ���� ****//
//    clusteringFunBetweenBeams(N_curObj, N_cacheObj, calcAngleMode, clusterMeas, measObjOut);
//
//    /*measObjOut->scanType = clusterObjIn->scanType;
//    measObjOut->tasTargetId = clusterObjIn->tasTargetId;
//    measObjOut->timestamp = clusterObjIn->timestamp;*/
//
//
//}

//**** TWSģʽ-��λ��㼣����ģ�� ****//
    void clusteringBetweenBeam_tws(uint8_t calcAngleMode, sMeasOutput* cacheMeasBeam, sMeasOutput* measObjOut)
    {
        uint8_t i_curObj, i_cacheObj;
        uint8_t N_curObj = measObjOut->num;//��ǰ��λ�����ĵ㼣����
        uint8_t N_cacheObj = cacheMeasBeam->num; //��������λ�ھ�����Ŀ������
        sClusterMeasure* clusterMeas = gClusterMeas;//��λ�����㼣����ṹ��

        //**** 1����ǰ��λ���洢����λ���飺��ǰ��λ(measObjOut)+���沨λ(cacheMeasBeam) ****//
        memset(clusterMeas, 0, sizeof(sClusterMeasure));
        for (i_curObj = 0; i_curObj < N_curObj; i_curObj++)
        {
            memcpy(&clusterMeas->measurement[i_curObj], &measObjOut->measurement[i_curObj], sizeof(sMeasurement_SphPoint));
        }
        for (i_cacheObj = 0; i_cacheObj < N_cacheObj; i_cacheObj++)
        {
            memcpy(&clusterMeas->measurement[N_curObj + i_cacheObj], &cacheMeasBeam->measurement[i_cacheObj], sizeof(sMeasurement_SphPoint));
        }

        clusterMeas->clusterMeasNum = N_curObj + N_cacheObj;

        //**** 2��DNScan�������鲨λ���о������ ****//
        cluster_DBscan(clusterMeas, N_curObj);//N_curObjΪ�µ������Χ

        //**** 3������Ŀ����о��� + �ȷ����,measObjOut�����¸�ֵ ****//
        clusteringFunBetweenBeams(N_curObj, N_cacheObj, calcAngleMode, clusterMeas, measObjOut);

    }



//**** TASģʽ-��λ�ڵ㼣����ģ�� ****//
void detChoseWithinBeam_tas(sDetectFrame* detClutProOut, sTrackingTarget* tasTrackStr, sMultiBeamMeas* cacheBeam)
{
    uint8_t n_det, Idx_maxCur, i_clusObj = 0;
    uint8_t numTAS;
    uint8_t numPoints = 0;
    uint8_t trackDetNum;
    float tmpMag, Mag_maxCur = 0;
    uint8_t clusterID[MAX_NUM_DETECTS] = { 0 };
    sDetTrackFrame* TrackDetStr = gTrackDetStr;//TASĿ��DBScan����ɸѡ�ļ���ṹ��
    sDetTrackFrame* cacheTas;

    //**** 1����TASĿ��Ե�ǰ֡�������DBScan����ɸѡ ****//
    trackDetNum = detTrack_DBscan(detClutProOut, tasTrackStr, clusterID);

    memset(TrackDetStr, 0, sizeof(sDetTrackFrame));//��λ�ڿ���TASĿ�����ļ��ṹ���ʼ��

    numPoints = detClutProOut->detectObjNum;//��λ�ڵļ�������

#ifdef DEBUG_LOG_HXJ
    //my_printf("1111111111111111trackDetNum numPoints  [%d %d]\n", trackDetNum, numPoints);
#endif

    if (trackDetNum > 0)
    {
        for (i_clusObj = 0; i_clusObj < trackDetNum; i_clusObj++)
        {
            for (n_det = 0; n_det < numPoints; n_det++)
            {
                if (clusterID[n_det] == 1)
                {
                    tmpMag = detClutProOut->detPointsList[n_det].mag * INV_ONE6FORMAT_SCALE;
                    if (tmpMag > Mag_maxCur)
                    {
                        Idx_maxCur = n_det;
                        Mag_maxCur = tmpMag;
                    }

                    memcpy(&TrackDetStr->detectFrame.detPointsList[i_clusObj], &detClutProOut->detPointsList[n_det], sizeof(sDetectPoint));
                    clusterID[n_det] = 0;
                    break;
                }
            }
        }
        TrackDetStr->detectFrame.detectObjNum = trackDetNum;//��λ�ڿ���TASĿ�����ļ�������
    }
    TrackDetStr->detectFrame.detectObjByte = detClutProOut->detectObjByte;
    TrackDetStr->detectFrame.frameID = detClutProOut->frameID;
    TrackDetStr->detectFrame.timestamp = detClutProOut->timestamp;
    TrackDetStr->detectFrame.trackTwsTasFlag = detClutProOut->trackTwsTasFlag;
    TrackDetStr->aziBeamSin[0] = (int16_t)(sinf( detClutProOut->detPointsList[Idx_maxCur].azimuth * INV_ONE6FORMAT_SCALE * DEG2RAD) * ONE15FORMAT_SCALE);
    TrackDetStr->eleBeamSin[0] = (int16_t)(sinf(detClutProOut->detPointsList[Idx_maxCur].elevation * INV_ONE6FORMAT_SCALE * DEG2RAD) * ONE15FORMAT_SCALE);


    //**** 2����λ��Ϣ�洢 ****//
    numTAS = cacheBeam->numTas;

#ifdef DEBUG_LOG_HXJ
    //my_printf("================numTAS [%d]\n", numTAS);
#endif

    cacheTas = (sDetTrackFrame*)cacheBeam->multiBeam_tas;
    if (numTAS < COCH_BEAM_NUM_TAS)
    {
        memcpy(&cacheTas[cacheBeam->numTas], TrackDetStr, sizeof(sDetTrackFrame));
        cacheBeam->numTas++;
    }
    else
    {
        //����4����λ��ֱ�����㣬��0��λ��ʼ��ֵ
        memset(cacheTas, 0, COCH_BEAM_NUM_TAS * sizeof(sDetTrackFrame));
        cacheBeam->numTas = numTAS - COCH_BEAM_NUM_TAS;
        memcpy(&cacheTas[cacheBeam->numTas], TrackDetStr, sizeof(sDetTrackFrame));
        cacheBeam->numTas++;
    }

}

//**** TASģʽ-��λ��㼣���ദ��ģ�� ****//
void detClusteringBetweenBeam_tas(sDetTrackFrame* TrackDetStr, sTrackingTarget* tasTrackStr, uint8_t* cacheBeamTarNum, sMeasOutput* measObjOut)
    {
        uint8_t n_obj, i_tas, i_beamDet;
        uint8_t i_det, add;
        uint8_t ifCalcAzi, ifCalcEle;
        uint8_t IdxBeam[COCH_BEAM_NUM_TAS] = { 0 };
        uint16_t trackDetNum;
        float tmpMag, tmpRange, tmpDoppler, sumMag = 0;
        float K, Mag, Range, Doppler;// 2^6
        float MagBeam[COCH_BEAM_NUM_TAS] = { 0.0f };
        float phiBeam[COCH_BEAM_NUM_TAS / 2] = { 0.0f };
        float thetaBeam[COCH_BEAM_NUM_TAS / 2] = { 0.0f };
        float theta, phi;
        float differSumRatio_azi = 0, differSumRatio_ele = 0;
        sDetectFrame* detInst;
        detInst = &TrackDetStr->detectFrame;

        trackDetNum = detInst->detectObjNum;//4��TAS��λ���ܵļ�����Ŀ        

        if (trackDetNum < 1)
        {
            return;
        }
        else
        { 
            //**** �㼣���������TAS����Ŀ��+�����ನλ�ڿɾ������ ****//
            n_obj = 0;
            add = 0;
            Mag = 0;
            Range = 0;
            Doppler = 0;
            sumMag = 0;

            memset(IdxBeam, CLUSTER_POINT_LONELY, COCH_BEAM_NUM_TAS * sizeof(uint8_t));
            memset(MagBeam, 0, COCH_BEAM_NUM_TAS * sizeof(float));

            //���α���4����λ����������
            sumMag = sumMag + (float)(tasTrackStr->mag * INV_ONE6FORMAT_SCALE);//��1��Ϊ����Ŀ��
            for (i_tas = 0; i_tas < COCH_BEAM_NUM_TAS; i_tas++)
            {
                for (i_beamDet = 0; i_beamDet < cacheBeamTarNum[i_tas]; i_beamDet++)
                {
                    i_det = add + i_beamDet;

                    tmpMag = (float)(detInst->detPointsList[i_det].mag * INV_ONE6FORMAT_SCALE);// *INV_ONE6FORMAT_SCALE;
                    sumMag = sumMag + tmpMag;

                    if (tmpMag > MagBeam[i_tas])
                    {
                        IdxBeam[i_tas] = i_det;
                        MagBeam[i_tas] = tmpMag;
                    }
                }
                add = add + cacheBeamTarNum[i_tas];
            }        

            //����Ŀ��range ,doppler mag����
            K = ((float)(tasTrackStr->mag * INV_ONE6FORMAT_SCALE)) / sumMag;
            Mag = Mag + ((float)(tasTrackStr->mag * INV_ONE6FORMAT_SCALE)) * K;
            Range = Range + ((float)(tasTrackStr->range * INV_ONE6FORMAT_SCALE)) * K;
            Doppler = Doppler + ((float)(tasTrackStr->velocity * INV_ONE6FORMAT_SCALE)) * K;

            for (i_det = 0; i_det < trackDetNum; i_det++)
            {
                    tmpMag = (float)(detInst->detPointsList[i_det].mag * INV_ONE6FORMAT_SCALE);// *INV_ONE6FORMAT_SCALE;
                    tmpRange = (float)(detInst->detPointsList[i_det].range * INV_ONE6FORMAT_SCALE);// * INV_ONE6FORMAT_SCALE;
                    tmpDoppler = (float)(detInst->detPointsList[i_det].velocity * INV_ONE6FORMAT_SCALE);// * INV_ONE6FORMAT_SCALE;

                    K = tmpMag / sumMag;
                    Mag = Mag + tmpMag * K;
                    Range = Range + tmpRange * K;
                    Doppler = Doppler + tmpDoppler * K;

                    measObjOut->measurement[n_obj].detectionId[i_det] = detInst->detPointsList[i_det].id;//����TAS��λ�ڸ�����Ŀ������ĵ㼣ID
            }

            //�ȷ���ǣ�2ά
            ifCalcEle = 1;
            for (i_tas = 0; i_tas < COCH_BEAM_NUM_TAS / 2; i_tas++)
            {
                if (IdxBeam[i_tas] != CLUSTER_POINT_LONELY)
                {
                    phiBeam[i_tas] = (float)(asinf(TrackDetStr->eleBeamSin[i_tas] * INV_ONE15FORMAT_SCALE));//�·�/�Ϸ���λ
                }
                else
                {
                    ifCalcEle = 0;
                    break;
                }
            }

            ifCalcAzi = 1;
            for (i_tas = COCH_BEAM_NUM_TAS / 2; i_tas < COCH_BEAM_NUM_TAS; i_tas++)
            {
                if (IdxBeam[i_tas] != CLUSTER_POINT_LONELY)
                {
                    thetaBeam[i_tas - COCH_BEAM_NUM_TAS / 2] = (float)(asinf(TrackDetStr->aziBeamSin[i_tas] * INV_ONE15FORMAT_SCALE));//��/�ҷ���λ
                }
                else
                {
                    ifCalcAzi = 0;
                    break;
                }
            }
        
            //�⸩���� 
            if ((MagBeam[0] != 0) && (MagBeam[1] != 0))
            {
                differSumRatio_ele = fabsf((MagBeam[0] - MagBeam[1]) / (MagBeam[0] + MagBeam[1]));
                if ((ifCalcEle == 1) && (differSumRatio_ele <= DIFFER_SUM_RATIO_THRESHOLD_ELE))
                {
                    phi = ((phiBeam[0] + phiBeam[1]) / 2) + (K_ELE * cosf(phiBeam[1])) * ((MagBeam[0] - MagBeam[1]) / (MagBeam[0] + MagBeam[1]));//L-R,thetaL = theta1,thetaBeam=(theta1 + theta2) / 2
                }
            }
            else
            {
                phi = (float)(tasTrackStr->elevation * INV_ONE6FORMAT_SCALE * DEG2RAD);
            }
            //�ⷽλ��
            if ((MagBeam[2] != 0) && (MagBeam[3] != 0))
            {
                differSumRatio_azi = fabsf((MagBeam[2] - MagBeam[3]) / (MagBeam[2] + MagBeam[3]));
                if ((ifCalcAzi == 1) && (differSumRatio_azi <= DIFFER_SUM_RATIO_THRESHOLD_AZI))
                {
                    theta = ((thetaBeam[0] + thetaBeam[1]) / 2) + (K_AZI * cosf(thetaBeam[1])) * ((MagBeam[2] - MagBeam[3]) / (MagBeam[2] + MagBeam[3]));//L-R,thetaL = theta1,thetaBeam=(theta1 + theta2) / 2
                }
            }
            else
            {
                theta = (float)(tasTrackStr->azimuth * INV_ONE6FORMAT_SCALE * DEG2RAD);
            }

            //TAS����Ŀ�긳ֵ
            measObjOut->num = 1;//���������TAS�����ĸ���Ŀ��
            measObjOut->measurement[n_obj].detId = n_obj;
            measObjOut->measurement[n_obj].mag = Mag;
            measObjOut->measurement[n_obj].vector.range = Range;
            measObjOut->measurement[n_obj].vector.doppler = Doppler;
            measObjOut->measurement[n_obj].vector.azimuthRad = theta;
            measObjOut->measurement[n_obj].vector.pitchRad = phi;
            measObjOut->measurement[n_obj].x = Range * cosf(phi) * cosf(theta);
            measObjOut->measurement[n_obj].y = Range * cosf(phi) * sinf(theta);
            measObjOut->measurement[n_obj].z = Range * sinf(phi);
            measObjOut->measurement[n_obj].sinAzim = sinf(theta);
            measObjOut->measurement[n_obj].cosAzim = cosf(theta);
            measObjOut->measurement[n_obj].aziBeamSin = TrackDetStr->eleBeamSin[COCH_BEAM_NUM_TAS - 1];//TAS��ǰ���֡��λ��
            measObjOut->measurement[n_obj].eleBeamSin = TrackDetStr->eleBeamSin[COCH_BEAM_NUM_TAS - 1];//TAS��ǰ���֡��λ��
            measObjOut->measurement[n_obj].detectionNum = trackDetNum;//�������������ĵ㼣����
         }
    }


//**** TWSģʽ-��λ�ڵ㼣���ദ���� ****//
void clusteringFunInBeam(sClusterDetect* clusterDet, sDetectFrame* detClutProOut, sMeasOutput* clusterObjIn)
{
    uint8_t i_det, Idx_maxCur, i_clustObj, i_clustID, i_lonelyObj;
    float tmpMag, tmpRange, tmpDoppler, Mag_maxCur;
    float K, Mag, Range, Doppler, sumMag = 0 ;// unit/2^6
    float theta, phi;
    uint8_t *clustID = clusterDet->clusterID;
    uint8_t clustNum = clusterDet->clusterNum;
    uint8_t lonelyNum = clusterDet->lonelyNum;
    uint16_t detNum = detClutProOut->detectObjNum;
    sDetectPoint* clusterDetect = detClutProOut->detPointsList;

    memset(clusterObjIn, 0, sizeof(sMeasOutput));

#ifdef DEBUG_LOG_HXJ
    //my_printf("==============clustID_before [%d %d %d %d]\n", clustID[0], clustID[1], clustID[2], clustID[3]);
	//my_printf("aziSin eleSin [%.2f %.2f]\n", theta*RAD2DEG, phi*RAD2DEG);
#endif // DEBUG_LOG_HXJ
#ifdef DEBUG_LOG_HXJ
	//my_printf("clustNum lonelyNum [%d %d]\n", clustNum, lonelyNum);
#endif // DEBUG_LOG_HXJ
    clusterObjIn->num = clustNum + lonelyNum;//�㼣������Ŀ������

    //(1)�㼣����
    for (i_clustObj = 0; i_clustObj < clustNum; i_clustObj++)
    {
        i_clustID = 0;
        Mag = 0;
        Range = 0;
        Doppler = 0;
        Mag_maxCur = 0;
        sumMag = 0;


        for (i_det = 0; i_det < detNum; i_det++)
        {
            if (clustID[i_det] == (i_clustObj + 1))
            {
                tmpMag = clusterDetect[i_det].mag * INV_ONE6FORMAT_SCALE;
                sumMag = sumMag + tmpMag;

                if (tmpMag > Mag_maxCur)
                {
                    Idx_maxCur = i_det;
                    Mag_maxCur = tmpMag;
                }

                clusterObjIn->measurement[i_clustObj].detectionId[i_clustID] = clusterDetect[i_det].id;//i_det,������Ŀ������ĵ㼣ID�������⣿��������������������������������
                i_clustID++;
            }
        }
        clusterObjIn->measurement[i_clustObj].detectionNum = i_clustID;//�������������ĵ㼣����

        //**** ����Ŀ�긳ֵ ***//      
        //��λ�ھ���Ŀ��Ǹ�ֵ��ֵ���ľ������Ƕ���Ϣ
        theta = (float)(clusterDetect[Idx_maxCur].azimuth * INV_ONE6FORMAT_SCALE * DEG2RAD);
        phi = (float)(clusterDetect[Idx_maxCur].elevation * INV_ONE6FORMAT_SCALE * DEG2RAD);

        for (i_det = 0; i_det < detNum; i_det++)
        {
            if (clustID[i_det] == (i_clustObj + 1))
            {

                tmpMag = clusterDetect[i_det].mag * INV_ONE6FORMAT_SCALE;
                tmpRange = clusterDetect[i_det].range * INV_ONE6FORMAT_SCALE;
                tmpDoppler = clusterDetect[i_det].velocity * INV_ONE6FORMAT_SCALE;

                K = tmpMag / sumMag;
                Mag = Mag + tmpMag * K;
                Range = Range + tmpRange * K;
                Doppler = Doppler + tmpDoppler * K;  

            }
        }
        clusterObjIn->measurement[i_clustObj].detId = i_clustObj;//�㼣������Ŀ��ID
        clusterObjIn->measurement[i_clustObj].mag = Mag;
        clusterObjIn->measurement[i_clustObj].vector.range = Range;
        clusterObjIn->measurement[i_clustObj].vector.doppler = Doppler;
        clusterObjIn->measurement[i_clustObj].vector.azimuthRad = theta;
        clusterObjIn->measurement[i_clustObj].vector.pitchRad = phi;
        clusterObjIn->measurement[i_clustObj].x = Range * cosf(phi) * cosf(theta);
        clusterObjIn->measurement[i_clustObj].y = Range * cosf(phi) * sinf(theta);
        clusterObjIn->measurement[i_clustObj].z = Range * sinf(phi);
        clusterObjIn->measurement[i_clustObj].sinAzim = sinf(theta);
        clusterObjIn->measurement[i_clustObj].cosAzim = cosf(theta);
        clusterObjIn->measurement[i_clustObj].aziBeamSin = (int16_t)(sinf(theta)* ONE15FORMAT_SCALE);
        clusterObjIn->measurement[i_clustObj].eleBeamSin = (int16_t)(sinf(phi) * ONE15FORMAT_SCALE);

    }

    //(2)�µ�Ŀ�����
    for (i_lonelyObj = 0; i_lonelyObj < lonelyNum; i_lonelyObj++)
    {
        for (i_det = 0; i_det < detNum; i_det++)
        {
            if (clustID[i_det] == CLUSTER_POINT_LONELY)
            {
                //��λ�ھ���Ŀ��Ǹ�ֵ��ֵ���ľ������Ƕ���Ϣ
                theta = (float)(clusterDetect[i_det].azimuth * INV_ONE6FORMAT_SCALE * DEG2RAD);
                phi = (float)(clusterDetect[i_det].elevation * INV_ONE6FORMAT_SCALE * DEG2RAD);

                clusterObjIn->measurement[clustNum + i_lonelyObj].detId = clustNum + i_lonelyObj;//�㼣������Ŀ��ID
                clusterObjIn->measurement[clustNum + i_lonelyObj].detectionId[0] = clusterDetect[i_det].id;//i_det;������Ŀ������ĵ㼣ID
                clusterObjIn->measurement[clustNum + i_lonelyObj].detectionNum = 1;//�������������ĵ㼣����
                clusterObjIn->measurement[clustNum + i_lonelyObj].mag = clusterDetect[i_det].mag * INV_ONE6FORMAT_SCALE;
                clusterObjIn->measurement[clustNum + i_lonelyObj].vector.range = clusterDetect[i_det].range * INV_ONE6FORMAT_SCALE;
                clusterObjIn->measurement[clustNum + i_lonelyObj].vector.doppler = clusterDetect[i_det].velocity * INV_ONE6FORMAT_SCALE;
                clusterObjIn->measurement[clustNum + i_lonelyObj].vector.azimuthRad = theta;
                clusterObjIn->measurement[clustNum + i_lonelyObj].vector.pitchRad = phi;
                clusterObjIn->measurement[clustNum + i_lonelyObj].x = (clusterDetect[i_det].range * INV_ONE6FORMAT_SCALE) * cosf(phi) * cosf(theta);
                clusterObjIn->measurement[clustNum + i_lonelyObj].y = (clusterDetect[i_det].range * INV_ONE6FORMAT_SCALE) * cosf(phi) * sinf(theta);
                clusterObjIn->measurement[clustNum + i_lonelyObj].z = (clusterDetect[i_det].range * INV_ONE6FORMAT_SCALE) * sinf(phi);
                clusterObjIn->measurement[clustNum + i_lonelyObj].sinAzim = sinf(theta);
                clusterObjIn->measurement[clustNum + i_lonelyObj].cosAzim = cosf(theta);
                clusterObjIn->measurement[clustNum + i_lonelyObj].aziBeamSin = (int16_t)(sinf(theta) * ONE15FORMAT_SCALE);
                clusterObjIn->measurement[clustNum + i_lonelyObj].eleBeamSin = (int16_t)(sinf(phi) * ONE15FORMAT_SCALE);
                    
                clustID[i_det] = clustNum + i_lonelyObj + 1;
//#ifdef DEBUG_LOG_HXJ
//                    my_printf("!!!!!!!!!!!!!!!!!!clustID_after [%d %d %d %d]\n", clustID[0], clustID[1], clustID[2], clustID[3]);
//                    //my_printf("aziSin eleSin [%.2f %.2f]\n", theta*RAD2DEG, phi*RAD2DEG);
//#endif // DEBUG_LOG_HXJ
                break;
            }
        }

    }

}


//**** TWSģʽ-��λ��㼣���ദ���� ****//
void clusteringFunBetweenBeams(uint8_t N_curObj, uint8_t N_cacheObj, uint8_t calcAngleMode, sClusterMeasure* clusterMeas, sMeasOutput* measObjOut)
    {
        uint8_t i_curObj, i_clustObj, i_cacheObj, i_lonelyObj;
        uint8_t Idx_maxCur, Idx_maxCache;
        uint8_t clustNum, lonelyNum, tmpDetNum = 0;
        uint8_t clusterMeasNum;//clusterMeasNum = clustNum + lonelyNum
        float tmpMag, tmpRange, tmpDoppler;
        float K, Mag, Range, Doppler;// 2^6
        float Mag_maxCur = 0, Mag_maxCache = 0, sumMag = 0, differSumRatio = 0; //maxMag
        float theta_Cur, phi_Cur, theta_Cache, phi_Cache, theta, phi;
        sMeasurement_SphPoint* inMeas;//*outMeas, ��λ�����Ŀ�껺��ṹ��

        clustNum = clusterMeas->clusterNum;
        lonelyNum = clusterMeas->lonelyNum;
        clusterMeasNum = clusterMeas->clusterMeasNum;//clusterMeasNum = N_curObj+N_cacheObj

        /* ����ṹ�� */
        inMeas = clusterMeas->measurement;

        memset(measObjOut, 0, sizeof(sMeasOutput));

        //��λ��������״̬��ϢΪ��ǰ��λ״̬��Ϣ
        measObjOut->num = clustNum + lonelyNum;//�㼣������Ŀ������


        //(1)�㼣����
        for (i_clustObj = 0; i_clustObj < clustNum; i_clustObj++)
        {
            Mag = 0;
            Range = 0;
            Doppler = 0;
            sumMag = 0;

            Idx_maxCur = 0;
            Mag_maxCur = 0;
            Idx_maxCache = N_curObj;
            Mag_maxCache = 0;

            tmpDetNum = 0;
            differSumRatio = 0;

            for (i_curObj = 0; i_curObj < N_curObj; i_curObj++)
            {
                if (clusterMeas->clusterID[i_curObj] == (i_clustObj + 1))
                {
                    tmpMag = inMeas[i_curObj].mag;// * INV_ONE6FORMAT_SCALE;
                    sumMag = sumMag + tmpMag;

                    if (tmpMag > Mag_maxCur)
                    {
                        Idx_maxCur = i_curObj;
                        Mag_maxCur = tmpMag;
                    }
                    
                    memcpy(&measObjOut->measurement[i_clustObj].detectionId[tmpDetNum], inMeas[i_curObj].detectionId, inMeas[i_curObj].detectionNum * sizeof(uint16_t));//��ǰ����һ֡������Ŀ������ĵ㼣ID
                    tmpDetNum = tmpDetNum + inMeas[i_curObj].detectionNum;//�������������ĵ㼣����
                }
            }
            measObjOut->measurement[i_clustObj].detectionNum = tmpDetNum;//�������������ĵ㼣����

            for (i_cacheObj = 0; i_cacheObj < N_cacheObj; i_cacheObj++)
            {
                if (clusterMeas->clusterID[N_curObj + i_cacheObj] == (i_clustObj + 1))
                {
                    tmpMag = inMeas[N_curObj + i_cacheObj].mag;// *INV_ONE6FORMAT_SCALE;
                    sumMag = sumMag + tmpMag;

                    if (tmpMag > Mag_maxCache)
                    {
                        Idx_maxCache = N_curObj + i_cacheObj;
                        Mag_maxCache = tmpMag;
                    }
                }
            }

            //����Ŀ��range ,doppler mag����
            for (i_curObj = 0; i_curObj < clusterMeasNum; i_curObj++)
            {
                if (clusterMeas->clusterID[i_curObj] == (i_clustObj + 1))
                {
                    tmpMag = inMeas[i_curObj].mag;// *INV_ONE6FORMAT_SCALE;
                    tmpRange = inMeas[i_curObj].vector.range;// * INV_ONE6FORMAT_SCALE;
                    tmpDoppler = inMeas[i_curObj].vector.doppler;// * INV_ONE6FORMAT_SCALE;

                    K = tmpMag / sumMag;
                    Mag = Mag + tmpMag * K;
                    Range = Range + tmpRange * K;
                    Doppler = Doppler + tmpDoppler * K;
                }
            }

            //�ȷ���ǣ�1Ϊ��ǰ��λ��2Ϊ���沨λ,��ɨ��Ϣ
            theta_Cur = (float)(asinf(inMeas[Idx_maxCur].aziBeamSin * INV_ONE15FORMAT_SCALE)); //Ϊ��ǰ��λ,���޾���㣬Idx_maxCur=0Ϊ��ǰ��λֵ
            phi_Cur = (float)(asinf(inMeas[Idx_maxCur].eleBeamSin * INV_ONE15FORMAT_SCALE));

            theta_Cache = (float)(asinf(inMeas[Idx_maxCache].aziBeamSin * INV_ONE15FORMAT_SCALE));//Ϊ���沨λ
            phi_Cache = (float)(asinf(inMeas[Idx_maxCache].eleBeamSin * INV_ONE15FORMAT_SCALE));

            //�����Ŀ���ȸ�ֵ��ǰ״̬��ýǶ�
            theta = inMeas[Idx_maxCur].vector.azimuthRad;
            phi = inMeas[Idx_maxCur].vector.pitchRad;

            //���������������Ǵ������¶�Ӧ�Ƕ�ֵ
            if ((Mag_maxCur != 0)&& (Mag_maxCache != 0))
            { 
                differSumRatio = fabsf(Mag_maxCur - Mag_maxCache) / (Mag_maxCur + Mag_maxCache);
                if ((calcAngleMode == 1) && (differSumRatio <= DIFFER_SUM_RATIO_THRESHOLD_AZI))//�ⷽλ��
                {

                    if (theta_Cur < theta_Cache)
                    {
                        theta = (theta_Cur + theta_Cache) / 2 + (K_AZI * cosf(theta_Cache)) * (Mag_maxCur - Mag_maxCache) / (Mag_maxCur + Mag_maxCache);//L-R,thetaL = theta_Cur,theta0=(theta_Cur + theta_Cache) / 2
                    }
                    else
                    {
                        theta = (theta_Cur + theta_Cache) / 2 + (K_AZI * cosf(theta_Cur)) * (Mag_maxCache - Mag_maxCur) / (Mag_maxCur + Mag_maxCache);//thetaL = theta_Cache
                    }
                }
                else if ((calcAngleMode == 2) && (differSumRatio <= DIFFER_SUM_RATIO_THRESHOLD_ELE))//�⸩����
                {
                    if (phi_Cur < phi_Cache)
                    {
                        phi = (phi_Cur + phi_Cache) / 2 + (K_ELE * cosf(phi_Cache)) * (Mag_maxCur - Mag_maxCache) / (Mag_maxCur + Mag_maxCache);//phiL = phi_Cur
                    }
                    else
                    {
                        phi = (phi_Cur + phi_Cache) / 2 + (K_ELE * cosf(phi_Cur)) * (Mag_maxCache - Mag_maxCur) / (Mag_maxCur + Mag_maxCache);//phiL = phi_Cache
                    }
                }
            }
           
            //����Ŀ�긳ֵ
            measObjOut->measurement[i_clustObj].detId = i_clustObj;
            measObjOut->measurement[i_clustObj].mag = Mag;
            measObjOut->measurement[i_clustObj].vector.range = Range;
            measObjOut->measurement[i_clustObj].vector.doppler = Doppler;
            measObjOut->measurement[i_clustObj].vector.azimuthRad = theta;
            measObjOut->measurement[i_clustObj].vector.pitchRad = phi;
            measObjOut->measurement[i_clustObj].x = Range * cosf(phi) * cosf(theta);
            measObjOut->measurement[i_clustObj].y = Range * cosf(phi) * sinf(theta);
            measObjOut->measurement[i_clustObj].z = Range * sinf(phi);
            measObjOut->measurement[i_clustObj].sinAzim = sinf(theta);
            measObjOut->measurement[i_clustObj].cosAzim = cosf(theta);
            measObjOut->measurement[i_clustObj].aziBeamSin = inMeas[Idx_maxCur].aziBeamSin;
            measObjOut->measurement[i_clustObj].eleBeamSin = inMeas[Idx_maxCur].eleBeamSin;

        }

        //(2)�µ�Ŀ�����
        for (i_lonelyObj = 0; i_lonelyObj < lonelyNum; i_lonelyObj++)
        {
            for (i_curObj = 0; i_curObj < N_curObj; i_curObj++)
            {
                if (clusterMeas->clusterID[i_curObj] == CLUSTER_POINT_LONELY)
                {
                    memcpy(&measObjOut->measurement[clustNum + i_lonelyObj], &inMeas[i_curObj], sizeof(sMeasurement_SphPoint));
                    measObjOut->measurement[clustNum + i_lonelyObj].detId = clustNum + i_lonelyObj;//�㼣������Ŀ��ID

                    clusterMeas->clusterID[i_curObj] = clustNum + i_lonelyObj + 1;
                    break;
                }
            }

        }

    }



//**** TWSģʽ-DBScan������ǰ��λ+�洢��λ���о����Ӻ��� ****//
void cluster_DBscan(sClusterMeasure* clusterMeas, uint8_t N)
    {
        uint8_t n_det, i_current, i_neigh;
        uint8_t* neighLast;
        uint8_t* neighCurrent;
        uint8_t neighCount, newCount;
        uint8_t clustTarID = 0, lonelyTarID = 0;
        uint8_t clusterMeasNum;
        uint8_t scope[MAX_NUM_DETECTS] = { 0 };
        uint8_t neighbors[MAX_NUM_DETECTS] = { 0 };
        uint8_t visited[MAX_NUM_DETECTS] = { 0 };

        clusterMeasNum = clusterMeas->clusterMeasNum;

        memset(visited, CLUSTER_POINT_UNKNOWN, clusterMeasNum * sizeof(uint8_t));
        memset(clusterMeas->clusterID, CLUSTER_POINT_LONELY, clusterMeasNum * sizeof(uint8_t));


        // scan through all the points to find its neighbors
        for (n_det = 0; n_det < clusterMeasNum; n_det++)
        {
            if (visited[n_det] != CLUSTER_POINT_VISITED)
            {
                visited[n_det] = CLUSTER_POINT_VISITED;

                neighCurrent = neighLast = neighbors;
                // scope is the local copy of visit
                memcpy(scope, visited, clusterMeasNum * sizeof(uint8_t));

                neighCount = cluster_DBscan_findNeighbors(
                    clusterMeas->measurement, n_det, neighLast, clusterMeasNum,
                    scope, &newCount);

#ifdef DEBUG_LOG_HXJ
                //my_printf("det[%d]: neightCount: %d\n", n_det, neighCount);
#endif
                /* The cluster consists of the point itself and its neighbours. */
                if (neighCount < MIN_NUM_POINTS_IN_CLUSTER - 1)
                {
                    // This point is lonely point
                    clusterMeas->clusterID[n_det] = CLUSTER_POINT_LONELY;
                }
                else
                {
                    // This point belongs to a New Cluster
                    clustTarID++;                                // New cluster ID
                    clusterMeas->clusterID[n_det] = clustTarID;      // This point belong to this cluster

                    // tag all the neighbors as visited_between in scope so that it will not be found again when searching neighbor's neighbor.
                    for (i_neigh = 0; i_neigh < newCount; i_neigh++)
                    {
                        i_current = neighLast[i_neigh];//�뵱ǰn_det�����ID
                        scope[i_current] = CLUSTER_POINT_VISITED;//�뵱ǰn_det�����ID���ǩ
                    }
                    neighLast += newCount;//neighLast���ƣ��˴α���������Ŀ������λ

                    while (neighCurrent != neighLast)               // neigh shall be at least minPoints in front of neighborhood pointer
                    {
                        // Explore the neighborhood
                        i_current = *neighCurrent++;               // Take point from the neighborhood
                        clusterMeas->clusterID[i_current] = clustTarID; // All points from the neighborhood also belong to this cluster
                        visited[i_current] = CLUSTER_POINT_VISITED;

                        neighCount = cluster_DBscan_findNeighbors(
                            clusterMeas->measurement, i_current, neighLast, clusterMeasNum,
                            scope, &newCount);

                        if (neighCount >= MIN_NUM_POINTS_IN_CLUSTER - 1)
                        {
                            for (i_neigh = 0; i_neigh < newCount; i_neigh++)
                            {
                                i_current = neighLast[i_neigh];
                                scope[i_current] = CLUSTER_POINT_VISITED;
                            }
                            neighLast += newCount;              /* Member is a core point, and its neighborhood is added to the cluster */
                        }
                    }
                    if (clustTarID >= MAX_NUM_DETECTS)
                    {
                        return;
                    }
                }
            }
        }

        clusterMeas->clusterNum = clustTarID;//�ɾ���Ŀ����

         //ֻ��TASģʽɨ���4��λ��Ŀ����йµ�������
        for (n_det = 0; n_det < N; n_det++)
        {
            if (clusterMeas->clusterID[n_det] == CLUSTER_POINT_LONELY)
            {
                lonelyTarID = lonelyTarID + 1;
            }
        }

        clusterMeas->lonelyNum = lonelyTarID;//�µ�Ŀ��

#ifdef DEBUG_LOG_HXJ
        //my_printf("numPoints clustTarID lonelyTarID [%d %d %d]\n", clusterMeasNum, clustTarID, lonelyTarID);
#endif // DEBUG_LOG_HXJ

    }

//**** TASģʽ-DBScan������ǰ��λ+����Ŀ����о����Ӻ��� ****//
uint8_t detTrack_DBscan(sDetectFrame* detClutProOut, sTrackingTarget* tasTrackStr, uint8_t *clusterID)
    {
        uint8_t n_det, i_current, i_neigh;
        uint8_t numPoints = 0;
        uint8_t clustTarID = 0;
        uint8_t* neighLast;
        uint8_t* neighCurrent;
        uint8_t neighCount, newCount;
        uint8_t visited_in[MAX_NUM_DETECTS] = { 0 };
        uint8_t scope[MAX_NUM_DETECTS] = { 0 };
        uint8_t neighbors[MAX_NUM_DETECTS] = { 0 };
        sDetectPoint* detPointStr;// 
        uint8_t trackDetNum = 0;//����TAS����Ŀ�����ļ�������

        //**** 1��Ŀ�����Թ��� ****//
        numPoints = detClutProOut->detectObjNum;
        detPointStr = detClutProOut->detPointsList;
        memset(visited_in, CLUSTER_POINT_UNKNOWN, numPoints * sizeof(uint8_t));

        // Init the clusterID of points are CLUSTER_POINT_LONELY
        for (n_det = 0; n_det < numPoints; n_det++)
        {
            clusterID[n_det] = CLUSTER_POINT_LONELY;
        }
        // scan through all the points to find its neighbors
        neighCurrent = neighLast = neighbors;
        // scope is the local copy of visit
        memcpy(scope, visited_in, numPoints * sizeof(uint8_t));

        neighCount = detTrack_DBscan_findNeighbors(
            detPointStr, tasTrackStr, neighLast, numPoints,
            scope, &newCount);

        trackDetNum += neighCount;

        /* The cluster consists of the point itself and its neighbours. */
        if (neighCount < 1)
        {
            // This point is lonely point
            return trackDetNum;
        }
        else
        {            
            // This point belongs to a New Cluster
            clustTarID = 1;                                // New cluster ID��current tracking target

            // tag all the neighbors as visited_in in scope so that it will not be found again when searching neighbor's neighbor.
            for (i_neigh = 0; i_neigh < newCount; i_neigh++)
            {
                i_current = neighLast[i_neigh];//���뵱ǰTASĿ������ID
                scope[i_current] = CLUSTER_POINT_VISITED;//�뵱ǰn_det�����ID���ǩ
            }
            neighLast += newCount;//neighLast���ƣ��˴α���������Ŀ������λ

            while (neighCurrent != neighLast)               // neigh shall be at least minPoints in front of neighborhood pointer
            {
                // Explore the neighborhood
                i_current = *neighCurrent++;               // Take point from the neighborhood
                clusterID[i_current] = clustTarID; // All points from the neighborhood also belong to this cluster
                visited_in[i_current] = CLUSTER_POINT_VISITED;

                neighCount = detect_DBscan_findNeighbors(
                    detPointStr, i_current, neighLast, numPoints,
                    scope, &newCount);

                if (neighCount >= MIN_NUM_POINTS_IN_CLUSTER - 1)
                {
                    for (i_neigh = 0; i_neigh < newCount; i_neigh++)
                    {
                        i_current = neighLast[i_neigh];
                        scope[i_current] = CLUSTER_POINT_VISITED;
                    }
                    neighLast += newCount;              /* Member is a core point, and its neighborhood is added to the cluster */
                }

                trackDetNum += neighCount;
            }

        }
        return trackDetNum;
    }


//**** TWSģʽ-��λ��DBScan�㼣�����Ӻ��� ****//
uint8_t detect_DBscan_findNeighbors(sDetectPoint* detPointStr, uint8_t n, uint8_t* neigh, uint8_t numPoints, uint8_t* visited_in, uint8_t* newCount)
    {
        uint8_t ii;
        uint8_t newCounttmp = 0;
        float tmpRange = 0, tmpVelocity = 0;
        float rangeN = detPointStr[n].range * INV_ONE6FORMAT_SCALE;
        float velocityN = detPointStr[n].velocity * INV_ONE6FORMAT_SCALE;
        float rangeThreshold, veloThreshold;

        rangeThreshold = RANGE_THRESHOLD_WITHIN;
        veloThreshold = VELOCITY_THRESHOLD_WITHIN;

        for (ii = 0; ii < numPoints; ii++)
        {
            if (visited_in[ii] == CLUSTER_POINT_VISITED)
            {
                continue;
            }  

            tmpRange = (float)(detPointStr[ii].range * INV_ONE6FORMAT_SCALE);
            tmpVelocity = (float)(detPointStr[ii].velocity * INV_ONE6FORMAT_SCALE);
            if ((fabsf(tmpRange - rangeN) <= rangeThreshold) && (fabsf(tmpVelocity - velocityN) <= veloThreshold))
            {                                      
                /* Mark this point as a neighbour in the list of
                 * neighbours. Also increment the number of neighbours
                 * for this point. */
                *neigh = ii;
                neigh++;
                newCounttmp++;
            }
        }
        *newCount = (uint16_t)newCounttmp;
        return newCounttmp;
    }

//**** TWSģʽ-��λ��DBScan�㼣�����Ӻ��� ****//
uint8_t cluster_DBscan_findNeighbors(sMeasurement_SphPoint* measPointStr, uint8_t n, uint8_t* neigh, uint8_t numPoints, uint8_t* visited_between, uint8_t* newCount)
    {
        uint8_t ii;
        uint8_t newCounttmp = 0;
        float tmpRange = 0, tmpVelocity = 0;
        float rangeN = measPointStr[n].vector.range;
        float velocityN = measPointStr[n].vector.doppler;
        float rangeThreshold, veloThreshold;

        rangeThreshold = RANGE_THRESHOLD_BETWEEN;
        veloThreshold = VELOCITY_THRESHOLD_BETWEEN;

        for (ii = 0; ii < numPoints; ii++)
        {
            if (visited_between[ii] == CLUSTER_POINT_VISITED)
            {
                continue;
            }

            tmpRange = measPointStr[ii].vector.range;
            tmpVelocity = measPointStr[ii].vector.doppler;
            if ((fabsf(tmpRange - rangeN) <= rangeThreshold) && (fabsf(tmpVelocity - velocityN) <= veloThreshold))
            {
                /* Mark this point as a neighbour in the list of
                 * neighbours. Also increment the number of neighbours
                 * for this point. */
                *neigh = ii;
                neigh++;
                newCounttmp++;
            }
        }
        *newCount = (uint16_t)newCounttmp;
        return newCounttmp;
    }


//**** TASģʽ-����Ŀ��+����DBScan�㼣�����Ӻ��� ****//
uint8_t detTrack_DBscan_findNeighbors(sDetectPoint* detPointStr, sTrackingTarget* tasTrackStr, uint8_t* neigh, uint8_t numPoints, uint8_t* visited_in, uint8_t* newCount)
    {
        uint8_t ii;
        uint8_t newCounttmp = 0;
        float tmpRange = 0, tmpVelocity = 0;
        float rangeN = tasTrackStr->range * INV_ONE6FORMAT_SCALE;
        float velocityN = tasTrackStr->velocity * INV_ONE6FORMAT_SCALE;
        float rangeThreshold, veloThreshold;

        rangeThreshold = RANGE_THRESHOLD_TAS;
        veloThreshold = VELOCITY_THRESHOLD_TAS;

        for (ii = 0; ii < numPoints; ii++)
        {
            if (visited_in[ii] == CLUSTER_POINT_VISITED)
            {
                continue;
            }

            tmpRange = (float)(detPointStr[ii].range * INV_ONE6FORMAT_SCALE);
            tmpVelocity = (float)(detPointStr[ii].velocity * INV_ONE6FORMAT_SCALE);
            if ((fabsf(tmpRange - rangeN) <= rangeThreshold) && (fabsf(tmpVelocity - velocityN) <= veloThreshold))
            {
                /* Mark this point as a neighbour in the list of
                 * neighbours. Also increment the number of neighbours
                 * for this point. */
                *neigh = ii;
                neigh++;
                newCounttmp++;
            }
        }
        *newCount = (uint16_t)newCounttmp;
        return newCounttmp;
    }