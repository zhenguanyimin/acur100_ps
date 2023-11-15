
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/tracking_common.h"
#include "../include/tracking_int.h"
#ifdef JPDAF_ENABLE
#include "../include/tracking_jpdaf.h"
#endif

/**
*  @b Description
*  @n
*	   Algorithm level step funtion
*      Application shall call this function to process one frame of measurements with a given instance of the algorithm
*
*  @param[in]  handle
*      Handle to tracking module
*  @param[in]  point
*      Pointer to an array of input measurments. Each measurement has range/angle/radial velocity information
*  @param[in]  var
*      Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
*  @param[in]  mNum
*      Number of input measurements
*  @param[out]  t
*      Pointer to an array of \ref tracking_targetDesc. Application shall provide sufficient space for the expected number of targets. 
*      This function populates the descritions for each of the tracked target 
*  @param[out]  tNum
*      Pointer to a uint16_t value.
*      Function returns a number of populated target descriptos 
*  @param[out]  mIndex
*      Pointer to an array of uint8_t indices. Application shall provide sufficient space to index all measurment points.
*      This function populates target indices, indicating which tracking ID was assigned to each measurment. See Target ID defeinitions, example \ref TRACKING_ID_POINT_NOT_ASSOCIATED
*	   Shall be set to NULL when indices aren't required.
*  @param[out]  bench
*      Pointer to an array of benchmarking results. Each result is a 32bit timestamp. The array size shall be \ref TRACKING_BENCHMARK_SIZE. 
*      This function populates the array with the timestamps of free runing CPU cycles count.  
*	   Shall be set to NULL when benchmarking isn't required.
*
*  \ingroup TRACKING_ALG_EXTERNAL_FUNCTION
*
*  @retval
*      None
*/

void tracking_step(void *handle, sMeasOutput *measInfo)
{
    sTracking_moduleInstance *inst;
    inst = (sTracking_moduleInstance *)handle;
	inst->heartBeat++;

	tracking_modulePredict(inst, measInfo);

	tracking_moduleAssociate(inst, measInfo);

	tracking_moduleUpdate(inst, measInfo);

	tracking_moduleAllocate(inst, measInfo);

	tracking_moduleMerge(inst);

	tracking_moduleManager(inst);
}

void tracking_cal_FQ(sTracking_moduleInstance *inst,float dt)
{
	float varX = 0.f;
	float varY = 0.f;
	float varZ = 0.f;
	float dt2 = 0.f;
	float dt4 = 0.f;
	int i;
	float ax = 0.5f;
	float ay = 0.5f;
	float az = 1.f;
	varX = 0.f;
	varY = 0.f;
	varZ = 0.f;

	dt2 = powf(dt, 2);
	dt4 = powf(dt2, 2);
	inst->tracking_params->maxAcceleration[0] = const_max_acceleration[0];
	inst->tracking_params->maxAcceleration[1] = const_max_acceleration[1];
	inst->tracking_params->maxAcceleration[2] = const_max_acceleration[2];
	varX = powf(0.5f*inst->tracking_params->maxAcceleration[0], 2);
	varY = powf(0.5f*inst->tracking_params->maxAcceleration[1], 2);
	varZ = powf(0.5f*inst->tracking_params->maxAcceleration[2], 2);

#ifdef TRACKING_2D
	/* {1.f, 0.f, dt,  0.f,	dt2/2, 0.f,
		0.f, 1.f, 0.f, dt,	0.f,   dt2/2,
		0.f, 0.f, 1.f, 0.f,	dt,    0.f,
		0.f, 0.f, 0.f, 1.f,	0.f,   dt,
		0.f, 0.f, 0.f, 0.f,	1.f,   0.f,
		0.f, 0.f, 0.f, 0.f,	0.f,   1.f}; */
	memset(inst->tracking_params->F, 0, sizeof(float) * 36);
	inst->tracking_params->F[0] = 1.0f;
	inst->tracking_params->F[2] = dt;
	inst->tracking_params->F[4] = dt2 / 2.0f;
	inst->tracking_params->F[7] = 1.0f;
	inst->tracking_params->F[9] = dt;
	inst->tracking_params->F[11] = dt2 / 2.0f;
	inst->tracking_params->F[14] = 1.0f;
	inst->tracking_params->F[16] = dt;
	inst->tracking_params->F[21] = 1.0f;
	inst->tracking_params->F[23] = dt;
	inst->tracking_params->F[28] = 1.0f;
	inst->tracking_params->F[35] = 1.0f;

	memset(inst->tracking_params->F_x, 0, sizeof(float) * 9);
	inst->tracking_params->F_x[0] = 1.0f;
	inst->tracking_params->F_x[1] = dt;
	inst->tracking_params->F_x[2] = dt2 / 2.0f;
	inst->tracking_params->F_x[4] = 1.0f;
	inst->tracking_params->F_x[5] = dt;
	inst->tracking_params->F_x[8] = 1.0f;
	memset(inst->tracking_params->F_y, 0, sizeof(float) * 9);
	inst->tracking_params->F_y[0] = 1.0f;
	inst->tracking_params->F_y[1] = dt;
	inst->tracking_params->F_y[2] = dt2 / 2.0f;
	inst->tracking_params->F_y[4] = 1.0f;
	inst->tracking_params->F_y[5] = dt;
	inst->tracking_params->F_y[8] = 1.0f;

	//float Q6[36] = { \
	//dt4/4*varX,	0.f,        dt3/2*varX, 0.f,        dt2/2*varX,	0.f, \
	//0.f,        dt4/4*varY,	0.f,        dt3/2*varY,	0.f,        dt2/2*varY, \
	//dt3/2*varX,	0.f,        dt2*varX,	0.f,        dt*varX,    0.f, \
	//0.f,        dt3/2*varY,	0.f,        dt2*varY,	0.f,        dt*varY, \
	//dt2/2*varX,	0.f,        dt*varX,    0.f,        1.f*varX,   0.f, \
	//0.f,        dt2/2*varY, 0.f,        dt*varY,    0.f,        1.f*varY};
	//memset(inst->tracking_params->Q, 0.f, sizeof(float) * 36);
	//inst->tracking_params->Q[0] = dt4/4*varX;
	//inst->tracking_params->Q[2] = dt3/2*varX;
	//inst->tracking_params->Q[4] = dt2/2*varX;
	//inst->tracking_params->Q[7] = dt4/4*varY;
	//inst->tracking_params->Q[9] = dt3/2*varY;
	//inst->tracking_params->Q[11] = dt2/2*varY;
	//inst->tracking_params->Q[12] = dt3/2*varX;
	//inst->tracking_params->Q[14] = dt2*varX;
	//inst->tracking_params->Q[16] = dt*varX;
	//inst->tracking_params->Q[19] = dt3/2*varY;
	//inst->tracking_params->Q[21] = dt2*varY;
	//inst->tracking_params->Q[23] = dt*varY;
	//inst->tracking_params->Q[24] = dt2/2*varX;
	//inst->tracking_params->Q[26] = dt*varX;
	//inst->tracking_params->Q[28] = 1.f*varX;
	//inst->tracking_params->Q[31] = dt2/2*varY;
	//inst->tracking_params->Q[33] = dt*varY;
	//inst->tracking_params->Q[35] = 1.f*varY;

	//float Q6[36] = {
	//const_process_noise[0],	0.f,    0.f,    0.f,    0.f,    0.f,
	//0.f,    const_process_noise[0],   0.f,    0.f,	0.f,    0.f,
	//0.f,	0.f,    const_process_noise[1],	0.f,    0.f,    0.f,
	//0.f,    0.f,    0.f,    const_process_noise[1],	0.f,    0.f,
	//0.f,	0.f,    0.f,    0.f,    const_process_noise[2],   0.f,
	//0.f,    0.f,    0.f,    0.f,    0.f,    const_process_noise[2]};
	memset(inst->tracking_params->Q, 0, sizeof(float) * 36);
	inst->tracking_params->Q[0] = const_process_noise[0];
	inst->tracking_params->Q[7] = const_process_noise[1];
	inst->tracking_params->Q[14] = const_process_noise[2];
	inst->tracking_params->Q[21] = const_process_noise[3];
	inst->tracking_params->Q[28] = const_process_noise[4];
	inst->tracking_params->Q[35] = const_process_noise[5];

	memset(inst->tracking_params->Q_x, 0, sizeof(float) * 9);
	inst->tracking_params->Q_x[0] = const_process_noise[0];
	inst->tracking_params->Q_x[4] = const_process_noise[2];
	inst->tracking_params->Q_x[8] = const_process_noise[4];
	memset(inst->tracking_params->Q_y, 0, sizeof(float) * 9);
	inst->tracking_params->Q_y[0] = const_process_noise[1];
	inst->tracking_params->Q_y[4] = const_process_noise[3];
	inst->tracking_params->Q_y[8] = const_process_noise[5];

	memset(inst->tracking_params->J_x, 0, sizeof(float) * 6);
	inst->tracking_params->J_x[0] = 1.0f;
	inst->tracking_params->J_x[4] = 1.0f;
	memset(inst->tracking_params->J_y, 0, sizeof(float) * 6);
	inst->tracking_params->J_y[0] = 1.0f;
	inst->tracking_params->J_y[4] = 1.0f;
#else
	//float F9[81] = {				
	//1.f, 0.f, 0.f, dt,  0.f, 0.f, dt2/2, 0.f,   0.f,
	//0.f, 1.f, 0.f, 0.f, dt,  0.f, 0.f,   dt2/2, 0.f,
	//0.f, 0.f, 1.f, 0.f, 0.f, dt,  0.f,   0.f,   dt2/2,
	//0.f, 0.f, 0.f, 1.f, 0.f, 0.f, dt,    0.f,   0.f,
	//0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,   dt,    0.f,
	//0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f,   0.f,   dt,
	//0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f,   0.f,   0.f,
	//0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,   1.f,   0.f,
	//0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,   0.f,   1.f};
	memset(inst->tracking_params->F, 0, sizeof(float) * (SSIZE_XY*SSIZE_XY));
	inst->tracking_params->F[0] = 1.0f;
	inst->tracking_params->F[3] = dt;
	inst->tracking_params->F[6] = dt2 / 2.0f;
	inst->tracking_params->F[10] = 1.0f;
	inst->tracking_params->F[13] = dt;
	inst->tracking_params->F[16] = dt2 / 2.0f;
	inst->tracking_params->F[20] = 1.0f;
	inst->tracking_params->F[23] = dt;
	inst->tracking_params->F[26] = dt2 / 2.0f;
	inst->tracking_params->F[30] = 1.0f;
	inst->tracking_params->F[33] = dt;
	inst->tracking_params->F[40] = 1.0f;
	inst->tracking_params->F[43] = dt;
	inst->tracking_params->F[50] = 1.0f;
	inst->tracking_params->F[53] = dt;
	inst->tracking_params->F[60] = 1.0f;
	inst->tracking_params->F[70] = 1.0f;
	inst->tracking_params->F[80] = 1.0f;

	//float Q9[81] = {
	//dt4/4*varX,	0.f,        0.f,        dt3/2*varX, 0.f,        0.f,        dt2/2*varX, 0.f,        0.f,
	//0.f,        dt4/4*varY,	0.f,        0.f,        dt3/2*varY,	0.f,        0.f,        dt2/2*varY, 0.f,
	//0.f,        0.f,        dt4/4*varZ,	0.f,        0.f,        dt3/2*varZ,	0.f,        0.f,        dt2/2*varZ,
	//dt3/2*varX,	0.f,        0.f,        dt2*varX,	0.f,        0.f,        dt*varX,    0.f,        0.f,
	//0.f,        dt3/2*varY,	0.f,        0.f,        dt2*varY,	0.f,        0.f,        dt*varY,    0.f,
	//0.f,        0.f,        dt3/2*varZ,	0.f,        0.f,        dt2*varZ,	0.f,        0.f,        dt*varZ,
	//dt2/2*varX,	0.f,        0.f,        dt*varX,    0.f,        0.f,        1.f*varX,   0.f,        0.f,
	//0.f,        dt2/2*varY,	0.f,        0.f,        dt*varY,    0.f,        0.f,        1.f*varY,   0.f,
	//0.f,        0.f,        dt2/2*varZ,	0.f,        0.f,        dt*varZ,    0.f,        0.f,        1.f*varZ};
	memset(inst->tracking_params->Q, 0, sizeof(float) * (SSIZE_XY*SSIZE_XY));
	for (i = 0; i < SSIZE_XY; i++)
	{
		inst->tracking_params->Q[i*(SSIZE_XY + 1)] = const_process_noise[i];
	}
	inst->tracking_params->Q[0] = 0.25f*ax*ax*dt4;
	inst->tracking_params->Q[10] = 0.25f*ay*ay*dt4;
	inst->tracking_params->Q[20] = 0.25f*az*az*dt4;
	inst->tracking_params->Q[30] = ax * ax*dt2;
	inst->tracking_params->Q[40] = ay * ay*dt2;
	inst->tracking_params->Q[50] = az * az*dt2;
	inst->tracking_params->Q[60] = 0.001f;
	inst->tracking_params->Q[70] = 0.001f;
	inst->tracking_params->Q[80] = 0.001f;

#endif


}
void tracking_cal_dt_transMatrix(void *handle)
{
	sTracking_ListElem *tElem = NULL;
	uint16_t uid;
	float dt = 0.f;
	sTracking_objectUnit *tasTracker = NULL;
	sTracking_objectUnit *twsTracker = NULL;
	sTracking_moduleInstance *inst;
	inst = (sTracking_moduleInstance *)handle;
	if (g_scanType == TAS_SCAN)
	{
		tElem = tracking_listGetFirst(&inst->tasTrackList);
		while (tElem != 0)
		{
			uid = tElem->data;
			tasTracker = (sTracking_objectUnit *)(inst->hTrack[uid]);
			if (tasTracker->uid == inst->tasTargetId)
				break;
			tElem = tracking_listGetNext(tElem);
		}
		inst->tracking_params->dt = g_curTimestamp - tasTracker->timestamp;
		if (inst->tracking_params->dt<0.001f || inst->tracking_params->dt>2.f)
		{
			inst->tracking_params->dt = FRAME_TIME_TAS;
		}
		tasTracker->timestamp = g_curTimestamp;
		dt = inst->tracking_params->dt;
	}
	else		// TWS_SCAN
	{
		tElem = tracking_listGetFirst(&inst->twsTrackList);
		if (tElem == 0)
			return;
		twsTracker = (sTracking_objectUnit *)(inst->hTrack[tElem->data]);
		inst->tracking_params->dt = g_curTimestamp - twsTracker->timestamp;
		if (inst->tracking_params->dt<0.001f || inst->tracking_params->dt>2.f)
		{
			inst->tracking_params->dt = FRAME_TIME_TWS;
		}
		dt = inst->tracking_params->dt;
#ifdef DEBUG_LOG_ZQ
		my_printf("gCurtime trackTime dt %.3f %.3f %.3f", g_curTimestamp, twsTracker->timestamp, dt);
#endif // DEBUG_LOG

		while (tElem != 0)
		{
			uid = tElem->data;
			twsTracker = (sTracking_objectUnit *)(inst->hTrack[uid]);
			twsTracker->timestamp = g_curTimestamp;
			tElem = tracking_listGetNext(tElem);
		}
	}
	tracking_cal_FQ(inst,dt);
}


