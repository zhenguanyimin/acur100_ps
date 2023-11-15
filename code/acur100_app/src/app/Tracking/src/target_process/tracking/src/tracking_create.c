
#include <string.h>
#include <math.h>
#include <float.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"


/**
*  @b Description
*  @n
*		Algorithm level create funtion.
*       Application calls this function to create an instance of tracking algorithm with desired configuration parameters. 
*		Function returns a handle, which shall be used to execute a single frame step function, or a delete function
*
*  @param[in]  config
*		This is a pointer to the configuration structure. The structure contains all parameters that are exposed by tracking alrorithm.
*		The configuration does not need to persist.
*		Advanced configuration structure can be set to NULL to use the default one.
*		Any field within Advanced configuration can also be set to NULL to use the default values for the field.
*  @param[out] errCode
*      Error code populated on error, see \ref TRACKING_ERROR_CODE
*
*  \ingroup TRACKING_ALG_EXTERNAL_FUNCTION
*
*  @retval
*      Handle to tracking module
*/

void *tracking_create(sTrackingParams *config, int32_t *errCode, sTracking_insideScratchPad *scratchPadData)
{
	sTracking_objectUnit** unitTrackList = scratchPadData->unitTrackList;
	sTracking_moduleInstance *inst = NULL;
	uint16_t uid;
	float dt, dt2;
//    float varX, varY;
	*errCode = TRACKING_EOK;

	//if(config->maxNumPoints > TRACKING_NUM_POINTS_MAX) {
	//	*errCode = TRACKING_EINVAL;
	//	goto exit;
	//}

	//if(config->maxNumTracks > TRACKING_NUM_TRACKS_MAX) {
	//	*errCode = TRACKING_EINVAL;
	//	goto exit;
	//}

	inst = &scratchPadData->moduleInst;
	if(inst == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	inst->maxNumPoints = MAX_NUM_DETECTS;
	inst->maxNumTracks = MAX_NUM_TRAJS;
	inst->wholeSpaceScanCycleCnt = 0;
	inst->lastWholeSpaceScanCycleCnt = 0;

	inst->platformInfo.egoLineVel_long = 0.0f;
	inst->platformInfo.egoLineVel_lat = 0.0f;
	inst->platformInfo.egoOmega = 0.0f;
	inst->platformInfo.egoLineAcc_long = 0.0f;
	inst->platformInfo.egoLineAcc_lat = 0.0f;
	//inst->maxNumPoints = config->maxNumPoints;
	//inst->maxNumTracks = config->maxNumTracks;

	// init strong environment

	inst->heartBeat = 0U;

    /* default parameters */
	inst->tracking_params = config;
	if(inst->tracking_params == NULL) {
		*errCode = TRACKING_EINVAL;
	}

	inst->tracking_params->dt = config->dt;
    dt = config->dt;
//	std::cout << "current data delta_t: " << dt << std::endl;
    dt2 = powf(dt,2);

    /* initialize process variance to 1/2 of maximum target acceleration */
    memcpy(inst->tracking_params->maxAcceleration, config->maxAcceleration, sizeof(config->maxAcceleration));

//    varX = powf(0.5f*config->maxAcceleration[0],2);
//    varY = powf(0.5f*config->maxAcceleration[1],2);
#ifndef TRACKING_2D
//    varZ = powf(0.5f*config->maxAcceleration[2],2);
#endif
//	std::cout << "max acceleration x: " << config->maxAcceleration[0] << std::endl;
//	std::cout << "vaxX: " << varX << std::endl;

#ifdef TRACKING_2D
	/* {1.f, 0.f, dt,  0.f,	dt2/2, 0.f, 
		0.f, 1.f, 0.f, dt,	0.f,   dt2/2,
		0.f, 0.f, 1.f, 0.f,	dt,    0.f,
		0.f, 0.f, 0.f, 1.f,	0.f,   dt,
		0.f, 0.f, 0.f, 0.f,	1.f,   0.f,
		0.f, 0.f, 0.f, 0.f,	0.f,   1.f}; */
	memset((uint8_t*)(inst->tracking_params->F), 0, sizeof(inst->tracking_params->F));
	inst->tracking_params->F[0] = 1.0f;
	inst->tracking_params->F[2] = dt;
	inst->tracking_params->F[4] = dt2/2.0f;
	inst->tracking_params->F[7] = 1.0f;
	inst->tracking_params->F[9] = dt;
	inst->tracking_params->F[11] = dt2/2.0f;
	inst->tracking_params->F[14] = 1.0f;
	inst->tracking_params->F[16] = dt;
	inst->tracking_params->F[21] = 1.0f;
	inst->tracking_params->F[23] = dt;
	inst->tracking_params->F[28] = 1.0f;
	inst->tracking_params->F[35] = 1.0f;

	memset((uint8_t*)(inst->tracking_params->F_x), 0, sizeof(inst->tracking_params->F_x));
	inst->tracking_params->F_x[0] = 1.0f;
	inst->tracking_params->F_x[1] = dt;
	inst->tracking_params->F_x[2] = dt2/2.0f;
	inst->tracking_params->F_x[4] = 1.0f;
	inst->tracking_params->F_x[5] = dt;
	inst->tracking_params->F_x[8] = 1.0f;
	memset((uint8_t*)(inst->tracking_params->F_y), 0, sizeof(inst->tracking_params->F_y));
	inst->tracking_params->F_y[0] = 1.0f;
	inst->tracking_params->F_y[1] = dt;
	inst->tracking_params->F_y[2] = dt2/2.0f;
	inst->tracking_params->F_y[4] = 1.0f;
	inst->tracking_params->F_y[5] = dt;
	inst->tracking_params->F_y[8] = 1.0f;

	//float Q6[36] = {
	//dt4/4*varX,	0.f,        dt3/2*varX, 0.f,        dt2/2*varX,	0.f,
	//0.f,        dt4/4*varY,	0.f,        dt3/2*varY,	0.f,        dt2/2*varY,
	//dt3/2*varX,	0.f,        dt2*varX,	0.f,        dt*varX,    0.f,
	//0.f,        dt3/2*varY,	0.f,        dt2*varY,	0.f,        dt*varY,
	//dt2/2*varX,	0.f,        dt*varX,    0.f,        1.f*varX,   0.f,
	//0.f,        dt2/2*varY, 0.f,        dt*varY,    0.f,        1.f*varY};

	//float Q6[36] = {
	//const_process_noise[0],	0.f,    0.f,    0.f,    0.f,    0.f,
	//0.f,    const_process_noise[0],   0.f,    0.f,	0.f,    0.f,
	//0.f,	0.f,    const_process_noise[1],	0.f,    0.f,    0.f,
	//0.f,    0.f,    0.f,    const_process_noise[1],	0.f,    0.f,
	//0.f,	0.f,    0.f,    0.f,    const_process_noise[2],   0.f,
	//0.f,    0.f,    0.f,    0.f,    0.f,    const_process_noise[2]};
	memset((uint8_t*)(inst->tracking_params->Q), 0, sizeof(inst->tracking_params->Q));
	inst->tracking_params->Q[0] = const_process_noise[0];
	inst->tracking_params->Q[7] = const_process_noise[1];
	inst->tracking_params->Q[14] = const_process_noise[2];
	inst->tracking_params->Q[21] = const_process_noise[3];
	inst->tracking_params->Q[28] = const_process_noise[4];
	inst->tracking_params->Q[35] = const_process_noise[5];

	memset((uint8_t*)(inst->tracking_params->Q_x), 0, sizeof(inst->tracking_params->Q_x));
	inst->tracking_params->Q_x[0] = const_process_noise[0]*0.25f;
	inst->tracking_params->Q_x[4] = const_process_noise[1]*0.25f;
	inst->tracking_params->Q_x[8] = const_process_noise[2]*0.25f;
	memset((uint8_t*)(inst->tracking_params->Q_y), 0, sizeof(inst->tracking_params->Q_y));
	inst->tracking_params->Q_y[0] = const_process_noise[0];
	inst->tracking_params->Q_y[4] = const_process_noise[1];
	inst->tracking_params->Q_y[8] = const_process_noise[2];

	memset((uint8_t*)(inst->tracking_params->J_x), 0, sizeof(inst->tracking_params->J_x));
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
	memset(inst->tracking_params->F, 0, sizeof(float) * 81);
	inst->tracking_params->F[0] = 1.0f;
	inst->tracking_params->F[3] = dt;
	inst->tracking_params->F[6] = dt2/2.0f;
	inst->tracking_params->F[10] = 1.0f;
	inst->tracking_params->F[13] = dt;
	inst->tracking_params->F[16] = dt2/2.0f;
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
	memset(inst->tracking_params->Q, 0, sizeof(float) * 81);
	inst->tracking_params->Q[0] = const_process_noise[0];
	inst->tracking_params->Q[10] = const_process_noise[1];
	inst->tracking_params->Q[20] = const_process_noise[2];
	inst->tracking_params->Q[30] = const_process_noise[3];
	inst->tracking_params->Q[40] = const_process_noise[4];
	inst->tracking_params->Q[50] = const_process_noise[5];
	inst->tracking_params->Q[60] = const_process_noise[6];
	inst->tracking_params->Q[70] = const_process_noise[7];
	inst->tracking_params->Q[80] = const_process_noise[8];
#endif


//	/* configured parameters */
//    switch(config->stateVectorType) {
//        case TRACKING_STATE_VECTORS_2DA:
//            {
//				memcpy(inst->tracking_params->F, inst->tracking_params->F, sizeof(float)*36);
//				memcpy(inst->tracking_params->Q, inst->tracking_params->Q, sizeof(float)*36);
//            }
//            break;

//        case TRACKING_STATE_VECTORS_3DA:
//            {
//				//memcpy(inst->tracking_params->F, inst->tracking_params->F, sizeof(float)*81);
//				//memcpy(inst->tracking_params->Q, inst->tracking_params->Q, sizeof(float)*81);
//            }
//            break;

//        default:
//    		*errCode = TRACKING_EINVAL;
//	    	goto exit;
//    }
	inst->tracking_params->stateVectorType = config->stateVectorType;

    inst->tracking_params->maxRadialVelocity = config->maxRadialVelocity;
	inst->tracking_params->radialVelocityResolution = config->radialVelocityResolution;
	inst->tracking_params->initialRadialVelocity = config->initialRadialVelocity;

	switch(config->verbose) {
		case TRACKING_VERBOSE_NONE:
			inst->tracking_params->verbose = 0U;
			break;
		case TRACKING_VERBOSE_ERROR:
			inst->tracking_params->verbose = VERBOSE_ERROR_INFO;
			break;
		case TRACKING_VERBOSE_WARNING:
			inst->tracking_params->verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO;
			break;
		default:
		case TRACKING_VERBOSE_DEBUG:
			inst->tracking_params->verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO | VERBOSE_DEBUG_INFO | VERBOSE_UNROLL_INFO | VERBOSE_STATE_INFO;
			break;
		case TRACKING_VERBOSE_MATRIX:			
			inst->tracking_params->verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO | VERBOSE_DEBUG_INFO | VERBOSE_MATRIX_INFO;
			break;
		case TRACKING_VERBOSE_MAXIMUM:
			inst->tracking_params->verbose = VERBOSE_ERROR_INFO | VERBOSE_WARNING_INFO | VERBOSE_DEBUG_INFO | VERBOSE_MATRIX_INFO | VERBOSE_UNROLL_INFO | VERBOSE_STATE_INFO | VERBOSE_ASSOSIATION_INFO;
			break;
	}

	/* hTrack is an array of void pointers */
	inst->hTrack = (void**)(unitTrackList);
	if(inst->hTrack == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	/* scoreSheet is an array of best scores */
	memset((uint8_t*)(inst->bestScore), 0, sizeof(inst->bestScore));
	if(inst->bestScore == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	/* association array holds the ids of the best scorers */
	memset((uint8_t*)(inst->bestIndex), 0, sizeof(inst->bestIndex));
	if(inst->bestIndex == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}
	memset((uint8_t*)(inst->isUniqueIndex), 0, sizeof(inst->isUniqueIndex));
	if(inst->isUniqueIndex == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	/* allocation array holds the measurement indices of allocation set */
	memset((uint8_t*)(inst->allocIndex), 0, sizeof(inst->allocIndex));
	if(inst->allocIndex == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	/* array of tracking IDs */
	memset((uint8_t*)(inst->uidElem), 0, sizeof(inst->uidElem));
	if(inst->uidElem == NULL) {
		*errCode = TRACKING_ENOMEM;
		goto exit;
	}

	inst->targetNumTotal = 0U;
	inst->targetNumCurrent = 0U;

	tracking_listInit(&inst->freeList);
	tracking_listInit(&inst->tasTrackList);
	tracking_listInit(&inst->twsTrackList);
	/* Create unit trackers */
	for(uid=0U; uid < inst->maxNumTracks; uid++) {
		inst->uidElem[uid].data = uid;
		tracking_listEnqueue(&inst->freeList, &inst->uidElem[uid]);
        inst->tracking_params->uid = uid;
		inst->hTrack[uid] = tracking_unitCreate(inst->tracking_params, errCode, scratchPadData->unitInst);
		if(*errCode != TRACKING_EOK) {
			goto exit;
		}
	}

exit:
	if(*errCode != TRACKING_EOK) {
		inst = NULL;
	}
	return inst;
}
