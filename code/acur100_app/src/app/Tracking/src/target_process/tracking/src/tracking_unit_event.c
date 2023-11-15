
#include <math.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

/**
*  @b Description
*  @n
*		tracking Module calls this function to run tracking unit level state machine 
*
*  @param[in]  handle
*		This is handle to GTARCK unit
*  @param[in]  num
*		This is number of associated measurments
*
*  \ingroup tracking_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/
void tracking_unitEvent(void *handle, sTracking_platformInfo* platformInfo, sTrackingParams *tracking_params)
{
    sTracking_objectUnit *inst;

	inst = (sTracking_objectUnit *)handle;

	inst->objManagement.heartBeatCount++;

	if(fabsf(inst->kalman_state.S_hat[0])<0.001f && fabsf(inst->kalman_state.S_hat[1])<0.001f)
	{
		inst->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
		my_printf("dead tracker debug %d id %d", 1, inst->uid);
#endif // DEBUG_LOG

		inst->objManagement.lifetime = 0;
		return;
	}
	if (inst->kalman_state.S_hat[0] < 0.001f)
	{
		inst->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
		my_printf("dead tracker debug %d id %d", 1, inst->uid);
#endif // DEBUG_LOG
		inst->objManagement.lifetime = 0;
		return;
	}
	// TODO: for demo
//#ifdef DEMO_ACUR
//	if (g_curTimestamp - inst->objManagement.allocationTime > 20.f&&inst->objKinematic.maxDistance < 5.f)
//	{
//		inst->objManagement.state = TRACK_STATE_FREE;
//#ifdef DEBUG_LOG_ZQ
//		my_printf("dead tracker debug %d id %d", 1, inst->uid);
//#endif // DEBUG_LOG
//		inst->objManagement.lifetime = 0;
//	}
//#endif // DEMO_ACUR
	if (tracking_params->workMode == 0)
	{
		tracking_TwsTargetStateManagement(inst);
	}
	else
	{
		if (g_scanType == TAS_SCAN)
		{
			tracking_TasTargetStateManagement(inst);
		}
		else
		{
			tracking_TwsTargetStateManagement(inst);
		}
	}
}

void putFreeStateTracker2FreeList(sTracking_moduleInstance *inst, sTracking_ListObj* activeList)
{
	sTracking_ListElem *tElem = NULL;
	sTracking_ListElem *tElemToRemove;
	uint16_t uid;
	sTracking_objectUnit *pTracker;
	tElem = tracking_listGetFirst(activeList);
	while (tElem != 0)
	{
		uid = tElem->data;
		pTracker = (sTracking_objectUnit *)inst->hTrack[uid];
		if (pTracker->objManagement.state == TRACK_STATE_FREE) {
			tElemToRemove = tElem;
			tElem = tracking_listGetNext(tElem);
			tracking_listRemoveElement(activeList, tElemToRemove);
			inst->targetNumCurrent--;
			tracking_listEnqueue(&inst->freeList, tElemToRemove);
		}
		else {
			tElem = tracking_listGetNext(tElem);
		}
	}

}
void tracking_tasManager(sTracking_moduleInstance *inst)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTrackingParams *tracking_params = inst->tracking_params;
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		if (uid == inst->tasTargetId)
		{
			tracking_unitEvent(inst->hTrack[uid], &inst->platformInfo, tracking_params);
			break;
		}
		tElem = tracking_listGetNext(tElem);
	}
	putFreeStateTracker2FreeList(inst, &inst->tasTrackList);
}

void tracking_twsManager(sTracking_moduleInstance *inst)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	sTrackingParams *tracking_params = inst->tracking_params;
	tElem = tracking_listGetFirst(&inst->twsTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		tracking_unitEvent(inst->hTrack[uid], &inst->platformInfo, tracking_params);
		tElem = tracking_listGetNext(tElem);
	}
	putFreeStateTracker2FreeList(inst, &inst->twsTrackList);
}
void setStateParams(sTracking_stateParams* stateParams, sTracking_objectUnit *inst)
{
	uint8_t active2freeThre;
	float range = inst->kalman_state.H_s.vector.range;
	float maxRangeThInClutter = 0.f;
	float maxRangeTh = 0.f;
	float minRangeTh = (inst->timestamp - inst->objManagement.allocationTime)*1.0f;
	float azimuth = inst->kalman_state.H_s.vector.azimuthRad*RAD2DEG;
	maxRangeThInClutter = minRangeTh > 30.f ? minRangeTh : 30.f;
	maxRangeTh = minRangeTh > 15.f ? minRangeTh : 15.f;
	//TODO : for demo
	if (fabsf(azimuth) > 5.0f)
	{
		stateParams->det2actThre = 20U;
		stateParams->det2freeThre = 1;
		stateParams->active2freeThre = 3;
	}
	// TODO: [210-320] is the range scope of 500kHz/2*500kHz power supply
//	if ((range > 210.f&&range < 320.f)||(range > 590.f&&range < 630.f))
//	{
//		stateParams->active2freeThre = 10;
//		stateParams->det2actThre = 100;
//		stateParams->det2freeThre = 2;
//		if (inst->objKinematic.maxDistance > maxRangeThInClutter)
//		{
//			stateParams->det2actThre = 30;
//		}
//	}
	active2freeThre = inst->objManagement.lifetime / 10;
	stateParams->active2freeThre = active2freeThre > stateParams->active2freeThre ? \
		active2freeThre : stateParams->active2freeThre;
	stateParams->active2freeThre = stateParams->active2freeThre > 20 ? 20 : stateParams->active2freeThre;
	if (fabsf(inst->kalman_state.H_s.vector.doppler) > VELOCITY_RANGE / 2.f)
	{
		stateParams->active2freeThre = 2;
		stateParams->det2freeThre = 1;
	}
	if (range < 20.f)		// TODO: near region(20m) false targets
	{
		stateParams->unseenTimeThre = 1.f;
		stateParams->active2freeThre = 3;
		stateParams->det2actThre = 30;
		stateParams->det2freeThre = 1;
	}
	else if (range < 50.f)
	{
		stateParams->det2actThre = 20;
		stateParams->unseenTimeThre = 1.f;
		stateParams->active2freeThre = 3;
		stateParams->det2freeThre = 1;
	}
	else if (range < 200.f)
	{
		stateParams->unseenTimeThre = 2.f;
	}
	else
	{
		stateParams->unseenTimeThre = 2.f;
	}

	//if (range > 50 && range < 200)
	//{
	//	stateParams->unseenTimeThre = (g_curTimestamp - inst->objManagement.allocationTime) / 10.f;
	//	stateParams->unseenTimeThre = stateParams->unseenTimeThre > 0.2f ? stateParams->unseenTimeThre : 0.2f;
	//	stateParams->unseenTimeThre = stateParams->unseenTimeThre < 2.0f ? stateParams->unseenTimeThre : 2.0f;
	//	/*stateParams->det2actThre = 10000;
	//	stateParams->det2freeThre = 10;
	//	if (g_curTimestamp - inst->objManagement.allocationTime > 3.f)
	//	{
	//		stateParams->det2actThre = 10;
	//	}*/
	//}

//	if ((range > 38 && range < 90) || (range > 0 && range < 23)) //ÒÖÖÆ½ü¾àÀëµÄ¼ÙÄ¿±ê
//	{
//		stateParams->det2actThre = 1000;
//		if (g_curTimestamp - inst->objManagement.allocationTime > 5.f)
//		{
//			stateParams->det2actThre = 10;
//		}
//	}

	//if (g_curTimestamp - inst->objManagement.allocationTime > 5.f && range > 225 && inst->objKinematic.maxDistance < 7.5f)
	//{
	//	inst->objManagement.state = TRACK_STATE_FREE;
	//}
}

void tracking_TasTargetStateManagement(sTracking_objectUnit *inst)
{
	sTracking_stateParams *tasStateParams = &inst->tracking_params->advParams.tasStateParams;
	sTracking_stateParams stateParams;
	memcpy(&stateParams, tasStateParams, sizeof(sTracking_stateParams));
	setStateParams(&stateParams, inst);

	switch (inst->objManagement.state)
	{
	case TRACK_STATE_DETECTION:
	{
		if (inst->objManagement.lifetime < stateParams.det2actThre)
		{
			if (inst->objManagement.forcastFrameNum == stateParams.det2freeThre)
			{
				inst->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
				my_printf("det 2 free id %d", inst->uid);
#endif // DEBUG_LOG

			}
		}
		else if(inst->objManagement.lifetime >= stateParams.det2actThre)
		{
			inst->objManagement.state = TRACK_STATE_NEW_ACTIVE;
		}
		break;
	}
	case TRACK_STATE_NEW_ACTIVE:
	case TRACK_STATE_ACTIVE:
	{
		if (inst->objManagement.forcastFrameNum > stateParams.active2freeThre&&\
			inst->timestamp - inst->assoProperty.lastSeenTime > stateParams.unseenTimeThre)
		{
			inst->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
			my_printf("act 2 free id %d", inst->uid);
#endif // DEBUG_LOG
		}
		else
		{
			inst->objManagement.state = TRACK_STATE_ACTIVE;
		}
		break;
	}
	default:
		break;
	}
}
void tracking_TwsTargetStateManagement(sTracking_objectUnit *inst)
{
	sTracking_stateParams *twsStateParams = &inst->tracking_params->advParams.twsStateParams;
	sTracking_stateParams stateParams;
	memcpy(&stateParams, twsStateParams, sizeof(sTracking_stateParams));
	setStateParams(&stateParams, inst);
	switch (inst->objManagement.state)
	{
	case TRACK_STATE_DETECTION:
	{
		if (inst->objManagement.lifetime < stateParams.det2actThre)
		{
			if (inst->objManagement.forcastFrameNum == stateParams.det2freeThre)
			{
				inst->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
				my_printf("det2free id %d, forcNum %d", inst->uid, inst->objManagement.forcastFrameNum);
#endif // DEBUG_LOG
			}
		}
		else if (inst->objManagement.lifetime >= stateParams.det2actThre)
		{
			inst->objManagement.state = TRACK_STATE_NEW_ACTIVE;
		}
		break;
	}
	case TRACK_STATE_NEW_ACTIVE:
	case TRACK_STATE_ACTIVE:
	{
		if (inst->objManagement.forcastFrameNum > stateParams.active2freeThre &&\
			inst->timestamp - inst->assoProperty.lastSeenTime > stateParams.unseenTimeThre)
		{
			inst->objManagement.state = TRACK_STATE_FREE;
#ifdef DEBUG_LOG_ZQ
			my_printf("act2free id %d, forcNum %d", inst->uid, inst->objManagement.forcastFrameNum);
#endif // DEBUG_LOG

		}
		else
		{
			inst->objManagement.state = TRACK_STATE_ACTIVE;
		}
		break;
	}
	default:
		break;
	}

}

//bubble ascending sort by range\doppler\distance\...
void sortTrackList(sTracking_moduleInstance *inst,sTracking_ListObj* trackList)
{
	sTracking_ListElem* p=trackList->begin;
	sTracking_ListElem* q=NULL;
	if (0==trackList->count || 0==trackList->begin)
	{
		return;
	}

	for (; p->next!=NULL; p=p->next)
	{
		for (q=p->next; q!=NULL; q = q->next)
		{				
			sTracking_objectUnit *instDataP = (sTracking_objectUnit *)(inst->hTrack[p->data]);
			sTracking_objectUnit *instDataQ = (sTracking_objectUnit *)(inst->hTrack[q->data]);
			float p_data=0, q_data=0;
			p_data = instDataP->kalman_state.H_s.vector.range*exp(-0.1*instDataP->kalman_state.H_s.vector.doppler+0);//weight coefficient to be adjusted,TODO add...
			q_data = instDataQ->kalman_state.H_s.vector.range*exp(-0.1*instDataQ->kalman_state.H_s.vector.doppler+0);

			if (q_data < p_data)//swap uid
			{
				int tmp = q->data;
				q->data = p->data;
				p->data = tmp;
			}
		}
	}
}

//by cdx //bubble sort TAS, ascending sort by fusion factors, and save the top TAS_ID_MAX_SIZE, others are moved to TWS
void tracking_TasTwsTypeManagement(sTracking_moduleInstance *inst)
{
	sTracking_ListElem *tTasElem = NULL;
	sTracking_ListElem *tTwsElem = NULL;
	sTracking_ListElem *tElemToMove;

	uint16_t uid;
	sTrackingParams *tracking_params = inst->tracking_params;
	if (0 == tracking_params->workMode)
	{
		tTasElem = tracking_listGetFirst(&inst->tasTrackList);
		if (NULL == tTasElem)
			return;
		while (tTasElem != 0)
		{
			tElemToMove = tTasElem;
			uid = tTasElem->data;
			tTasElem = tracking_listGetNext(tTasElem);
			tracking_listRemoveElement(&inst->tasTrackList, tElemToMove);
			tracking_listEnqueue(&inst->twsTrackList, tElemToMove);
		}
	}
	else
	{
		tracking_listMerge(&inst->twsTrackList, &inst->tasTrackList);

		//bubble sort, ascending sort by range\doppler\distance\...
		//sortTrackList(&inst->twsTrackList);
		sortTrackList(inst,&inst->twsTrackList);

		tracking_listSplit(&inst->twsTrackList,TAS_ID_MAX_SIZE, &inst->tasTrackList);

		//tracking_setTragetTasTwsProperty();
	}
}