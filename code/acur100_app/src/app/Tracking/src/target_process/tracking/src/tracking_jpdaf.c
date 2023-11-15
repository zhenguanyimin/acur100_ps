
#include <string.h>
#include <math.h>
#include <float.h>
#include "../../../utilities/common_struct.h"

#ifdef JPDAF_ENABLE
#include "../include/tracking_jpdaf.h"
#include "../include/tracking_int.h"

#define MAX_ASSO_DET_NUM (7U)
#define MAX_JOINT_TRACKER_NUM (2U)
#define MAX_JOINT_DETECTION_NUM (MAX_JOINT_TRACKER_NUM*MAX_ASSO_DET_NUM)

// Validation matrix for association among detections and trackers
uint8_t validation_matrix[TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX+1)]; // (dets_size, trackers_size+1)
																				// Splitted validation matrix for association among detections and trackers
// Mahalanobis distance matrix in spatial domain and velocity domain
float validation_costMatrix[TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX+1)]; // (dets_size, trackers_size+1)
// Probability matrix of association
float beta_matrix[(TRACKING_NUM_POINTS_MAX+1)*(TRACKING_NUM_TRACKS_MAX+1)]; // (dets_size, trackers_size+1)
// Probability vector of validation event
double prob_jointEventVector[NUM_SPLIT_MATRIX_MAX];
// The valid detection indices when associating
uint8_t validation_detIdx[TRACKING_NUM_POINTS_MAX];
// The flag of trackers, Wether the tracker is associated with some detection
uint8_t tracker_isAssociatedId[TRACKING_NUM_TRACKS_MAX]; // Is associated for each tracker
// The number of valid detections
uint8_t validation_detIdxNum = 0;
// The number of trackers
uint8_t tracker_Idx[TRACKING_NUM_TRACKS_MAX];
uint8_t validation_trackerIdxNum = 0;
// The actual number of splitted validation matrix
int gHyp_num = 0;

sJpdaf_param param_;
void jpdaf_dataAssoc(sTracking_moduleInstance *inst, sMeasurement_SphPoint *point, uint16_t num)
{
	// init every frame
	jpdaf_init(param_);

	// innovation matrix and its inv matrix for each tracker
	jpdaf_innovation_matrix(inst);

	// Priority assocation and generate validation matrix
	jpdaf_validation_matrix(inst, point, num);

	// calculate Joint Probabilities
	jpdaf_joint_probability(inst);
}

void jpdaf_init(sJpdaf_param in_param)
{
	in_param.pd = 1.f;
	in_param.global_assocCost = 7.81f;
	in_param.global_assocSpatialCost = 5.99f;
	memset(&param_, 0, sizeof(sJpdaf_param));
	memcpy(&param_, &in_param, sizeof(sJpdaf_param));
	validation_trackerIdxNum = param_.trackers_size;
	validation_detIdxNum = 0;
	gHyp_num = 0;
	memset(validation_matrix, 0, sizeof(validation_matrix));
	memset(validation_costMatrix, 0, sizeof(validation_costMatrix));
	memset(prob_jointEventVector, 0, sizeof(prob_jointEventVector));
	memset(beta_matrix, 0, sizeof(beta_matrix));
	memset(validation_detIdx, 0, sizeof(validation_detIdx));
	memset(tracker_isAssociatedId, 255, sizeof(tracker_isAssociatedId));
}

void jpdaf_innovation_matrix(sTracking_moduleInstance *inst)
{
	sTracking_ListElem *tElem;
	uint16_t uid;
	sTracking_platformInfo* platformInfo = &inst->platformInfo;
	sTracking_objectUnit *pTrack = NULL;
	float J[MSIZE_SPH*SSIZE_XY] = { 0.f };
	float PJ[SSIZE_XY*MSIZE_SPH] = { 0.f };
	float JPJ[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float S_inv[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float Rm[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float S[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float det = 0.f;
	float P_[SSIZE_XY*SSIZE_XY] = { 0.f };
	float Meas_var[MSIZE_SPH] = { 0.f };
	tElem = tracking_listGetFirst(&inst->tasTrackList);
#ifdef DEBUG_LOG
	//my_printf("---before---");
#endif // DEBUG_LOG
	while (tElem != 0)
	{
		uid = tElem->data;
		pTrack = (sTracking_objectUnit *)(inst->hTrack[uid]);
#ifdef DEBUG_LOG
		//my_printf("id %d x y vx vy [%.2f %.2f %.2f %.2f ]", \
			pTrack->uid, pTrack->kalman_state.S_hat[0], pTrack->kalman_state.S_hat[1],\
			pTrack->kalman_state.S_hat[2], pTrack->kalman_state.S_hat[3]);
#endif // DEBUG_LOG

		/* Compute state vector partial derivatives (Jacobian matrix) */
		tracking_computeJacobian(pTrack->tracking_params->stateVectorType, pTrack->kalman_state.S_hat, J);
		/* Compute JPJ' = J(:,1:mSize) * obj.P_apriori(1:mSize,1:mSize) * J(:,1:mSize)' */
		tracking_matrixUperTri2Matrix(pTrack->kalman_state.P_hat, P_);
		tracking_matrixComputePJT(P_, J, PJ);
		tracking_matrixMultiply(MSIZE_SPH, SSIZE_XY, MSIZE_SPH, J, PJ, JPJ);
		tracking_measurement_variance_cal(pTrack, const_tws_measurement_noise, Meas_var);
		tracking_matrixSetDiag(MSIZE_SPH, Meas_var, Rm);
		tracking_matrixAdd(MSIZE_SPH, MSIZE_SPH, JPJ, Rm, S);
		/* Compute inverse of S */
		tracking_matrixInv(S, &det, S_inv);
		memcpy(pTrack->assoProperty.gC_inv, S_inv, sizeof(S_inv));
		pTrack->assoProperty.gC_det = det;
		tElem = tracking_listGetNext(tElem);
	}
}

void jpdaf_stateUpdate(sTracking_moduleInstance *inst, sMeasurement_SphPoint *point, uint16_t num)
{
	uint8_t assocTrcIdx[TRACKING_NUM_POINTS_MAX] = { 0 }; // The index of trackers for each detection
	sTracking_ListElem *tElem;
	sTracking_ListElem *tElemToRemove;
	uint16_t uid;
	sTracking_enum_state state;
	uint8_t tracker_idx = 1;
	uint8_t det_idx = 0;
	int n = 0;
	sTracking_platformInfo* platformInfo = &inst->platformInfo;
	sTracking_objectUnit *inst_unit = NULL;
#ifdef DEBUG_LOG
	//my_printf("---after---");
#endif
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while(tElem != 0)
	{
		uid = tElem->data;
		inst_unit = (sTracking_objectUnit *)(inst->hTrack[uid]);
		
		state = jpdaf_unitUpdate(inst_unit, point, tracker_idx, false, platformInfo);
		// update the traj lenght
		if(state == TRACK_STATE_FREE) {
			tElemToRemove = tElem;
			tElem = tracking_listGetNext(tElem);
			tracking_listRemoveElement(&inst->tasTrackList, tElemToRemove);
            inst->targetNumCurrent --;
			tracking_listEnqueue(&inst->freeList, tElemToRemove);
		}
		else
		{
			tElem = tracking_listGetNext(tElem);
		}
		tracker_idx++;
		inst_unit->objManagement.isNewTracker = 0;
	}
}

// Generate validation matrix with gating
void jpdaf_validation_matrix(sTracking_moduleInstance *inst, sMeasurement_SphPoint *point, uint16_t num)
{
	uint16_t i = 0;
	sTracking_ListElem *tElem;
	uint16_t uid;
	sTracking_objectUnit *pTracker = NULL;
	sTracking_gateLimits* assocDynamicGating = NULL;
	float mah = 1000.f; // mahalanobis distance in position and velocity domain
	float mah_p = 1000.f; // mahalanobis distance in spatial domain
	uint16_t t = 1;
	uint8_t tao_meas = false;
	uint8_t numTrack = 0;
	float md_gating = param_.global_assocCost;
#ifdef PRINT_TXT
	FILE* fp1 = NULL;
	fopen_s(&fp1, "D:/JPDAF.txt", "a+");
#endif // PRINT_TXT
	tElem = tracking_listGetFirst(&inst->tasTrackList);
	while (tElem != 0)
	{
		uid = tElem->data;
		pTracker = (sTracking_objectUnit *)inst->hTrack[uid];
		tracker_Idx[numTrack] = uid;
#ifdef PRINT_TXT
		fprintf(fp1, "%d ", uid);
#endif // PRINT_TXT
#ifdef DEBUG_LOG
		//my_printf("limit [%.1f %.1f %.1f]", pTracker->assoProperty.H_limits.array[0], \
			pTracker->assoProperty.H_limits.array[1], pTracker->assoProperty.H_limits.array[2]);
#endif // DEBUG_LOG

		++numTrack;
		tElem = tracking_listGetNext(tElem);
	}
	for (i = 0; i < num; i++)
	{
		t = 1;
	    tao_meas = false;
		if (inst->bestIndex[i] == TRACKING_ID_POINT_BEHIND_THE_WALL)
		{
			continue;
		}
			
		tElem = tracking_listGetFirst(&inst->tasTrackList);
		while (tElem != 0)
		{
			uid = tElem->data;
			pTracker = (sTracking_objectUnit *)inst->hTrack[uid];
			mah = 1000;
			mah_p = 1000;
			jpdaf_gating_md(&point[i], pTracker, &mah, &mah_p);
#ifdef DEBUG_LOG
			//my_printf("trackId measId [%d %d] md %.2f", uid, i, mah);
#endif
#ifndef NANO_PROJECT
			if (pTracker->eDynamicProperty == TRACKING_DYNAMIC_STATIC && \
				fabsf(pTracker->kalman_state.H_s.vector.azimuthRad) < 20.f*DEG2RAD)
			{
				md_gating = 3.66f;
			}
#endif
			if (mah <= md_gating)
			{
				validation_matrix[validation_detIdxNum*(TRACKING_NUM_TRACKS_MAX + 1) + t] = 1;
				validation_matrix[validation_detIdxNum*(TRACKING_NUM_TRACKS_MAX + 1)] = 1;
				validation_costMatrix[validation_detIdxNum*(TRACKING_NUM_TRACKS_MAX + 1) + t] = mah;
				tao_meas = true;
				tracker_isAssociatedId[t - 1] = pTracker->uid;
				inst->bestIndex[i] = uid;

			}
#ifdef DEBUG_LOG
#ifndef STATE_DETECTION_OUTPUT
			if (pTracker->objManagement.state != TRACK_STATE_DETECTION)
			{
#endif // STATE_DETECTION_OUTPUT
				//my_printf("trackId measId [%d %d] md %.2f", uid, i, mah);
#ifndef STATE_DETECTION_OUTPUT
			}	
#endif // STATE_DETECTION_OUTPUT
#endif
#ifdef PRINT_TXT
			if (i == 0)
			{
					fprintf(fp1, "%d ", uid);
			}
#endif
			++t;	
			tElem = tracking_listGetNext(tElem);
		}
		if (tao_meas) {
			validation_detIdx[validation_detIdxNum] = i;
			validation_detIdxNum++;
#ifdef PRINT_TXT
			if (validation_detIdxNum==1)
			{
				fprintf(fp1, "\n ");
			}
			fprintf(fp1, "%d ", i);
#endif
		}
	}
	param_.trackers_size = numTrack;
	validation_trackerIdxNum = param_.trackers_size;
	validation_matrix_reduce();
#ifdef DEBUG_LOG
	//my_printf("trackNum %d", param_.trackers_size);
#endif
	
	// print data to txt file
#ifdef PRINT_TXT
	fprintf(fp1, "\n trackNum %d mesNum %d \n", param_.trackers_size, validation_detIdxNum);
	for (i = 0; i < validation_detIdxNum; i++)
	{
		for (t = 0; t < param_.trackers_size + 1; t++)
		{
			fprintf(fp1, "%d ", validation_matrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t]);
		}
		fprintf(fp1, "\n ");
	}
	for (i = 0; i < validation_detIdxNum; i++)
	{
		for (t = 0; t < param_.trackers_size + 1; t++)
		{
			fprintf(fp1, "%.2f ", validation_costMatrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t]);
		}
		fprintf(fp1, "\n ");
	}
	fclose(fp1);
#endif // PRINT_TXT
}

void jpdaf_gating_md(sMeasurement_SphPoint *measurement, sTracking_objectUnit *track, float *mah, float *mah_p)
{
	uint8_t isWithinLimits;
	sMeasurement_SphPoint u_tilda;
	int m = 0;
	memset(&u_tilda, 0, sizeof(sMeasurement_SphPoint));
	tracking_vectorSub(MEASUREMENT_VECTOR_SIZE, measurement->array, track->kalman_state.H_s.array, u_tilda.array);
	/* Any point outside the limits is outside the gate */
	isWithinLimits = true;
	for(m=0; m < MEASUREMENT_VECTOR_SIZE; m++)
	{
		if (m == 0 || m == 2)
		{
			if (fabs(u_tilda.array[m]) > track->assoProperty.H_limits.array[m])
			{
				isWithinLimits = false;
				break;
			}
		}
	}
	if (isWithinLimits == false) {
#ifdef DEBUG_LOG
		//my_printf("woLimit %d delta limit [%.1f %.1f]", m, u_tilda.array[m], track->assoProperty.H_limits.array[m]);
#endif // DEBUG_LOG

		return;
	}
	//TODO: gC_inv needs to check
	/* For the gating purposes we compute partial Mahalanobis distance, ignoring doppler */		
	tracking_computeMahalanobisPartial(u_tilda.array, track->assoProperty.gC_inv, mah_p);
	// gC_inv: The inverse of the innovation covariance matrix
	tracking_computeMahalanobis(u_tilda.array, track->assoProperty.gC_inv, mah);
}

void jpdaf_generate_hypothesis(uint8_t jointFlag, uint8_t valid_det_num, uint8_t* validation_matrix_joint, uint8_t* validation_splitMatrix)
{
	int i = 0;
	int j = 0;
	int n = 0, m = 0;
	uint16_t tCol = jointFlag + 2;
	// set the first col is 1 for each splitted validation matrix
	for (; j < NUM_SPLIT_MATRIX_MAX; j++) {
		for (; i < valid_det_num; i++) {
			validation_splitMatrix[j*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
				i*(MAX_JOINT_TRACKER_NUM + 1)] = 1;
		}
	}

	if (valid_det_num > 0)  // validation matrix is valid
	{
		for (i = 0; i < valid_det_num; i++)
		{
			for (j = 1; j < tCol; j++)
			{
				if (validation_matrix_joint[i*(MAX_JOINT_TRACKER_NUM + 1) + j] == 1)
				{
					validation_splitMatrix[gHyp_num*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
						i*(MAX_JOINT_TRACKER_NUM + 1) + 0] = 0;
					validation_splitMatrix[gHyp_num*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
						i*(MAX_JOINT_TRACKER_NUM + 1) + j] = 1;
					gHyp_num++;
					if (j == tCol - 1) continue;
					for (n = 0; n < valid_det_num; n++)
					{
						if (n != i)
						{
							for (m = j + 1; m < tCol; m++)
							{
								if (validation_matrix_joint[n*(MAX_JOINT_TRACKER_NUM + 1) + m])
								{
									validation_splitMatrix[gHyp_num*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
										i*(MAX_JOINT_TRACKER_NUM + 1) + 0] = 0;
									validation_splitMatrix[gHyp_num*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
										i*(MAX_JOINT_TRACKER_NUM + 1) + j] = 1;
									validation_splitMatrix[gHyp_num*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
										n*(MAX_JOINT_TRACKER_NUM + 1) + 0] = 0;
									validation_splitMatrix[gHyp_num*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
										n*(MAX_JOINT_TRACKER_NUM + 1) + m] = 1;
									gHyp_num++;
								}
							}
						}
					}
				}
			}
		}
	}
	gHyp_num++;
}

//void jpdaf_generate_hypothesis()
//{
//	int i = 0;
//	int j = 0;
//	// set the first col is 1 for each splitted validation matrix
//	for (; j < NUM_SPLIT_MATRIX_MAX; j++) {
//		for (; i < validation_detIdxNum; i++) {
//			validation_splitMatrix[j*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//								   i*(TRACKING_NUM_TRACKS_MAX + 1)] = 1;
//		}
//	}
//	i = 0;
//	int k = 0;
//	gHyp_num = 0;
//	int m = 0;
//	//Generating all the possible association matrices from the possible measurements
//	if (validation_detIdxNum != 0)
//	{
//		for (; i < validation_detIdxNum; ++i)
//		{
//			j = 1;
//			for (; j < validation_trackerIdxNum+1; ++j)
//			{
//				if (1==validation_matrix[i*(TRACKING_NUM_TRACKS_MAX + 1)+j]) // == 1
//				{
//					validation_splitMatrix[gHyp_num*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//										   i*(TRACKING_NUM_TRACKS_MAX + 1)+j] = 1;
//					validation_splitMatrix[gHyp_num*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//										   i*(TRACKING_NUM_TRACKS_MAX + 1)] = 0;
//					++gHyp_num;
//					if (j == validation_trackerIdxNum) continue;
//					k = 0;
//					for (; k < validation_detIdxNum; ++k)
//					{
//						if (k != i)
//						{
//							for (m = j + 1; m < validation_trackerIdxNum+1; ++m) // CHECK Q.COLS - 1
//							{
//								if (1==validation_matrix[k*(TRACKING_NUM_TRACKS_MAX + 1)+m]) // == 1
//								{
//									validation_splitMatrix[gHyp_num*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//														   i*(TRACKING_NUM_TRACKS_MAX + 1)+j] = 1;
//									validation_splitMatrix[gHyp_num*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//														   i*(TRACKING_NUM_TRACKS_MAX + 1)] = 0;
//									validation_splitMatrix[gHyp_num*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//														   k*(TRACKING_NUM_TRACKS_MAX + 1)+m] = 1;
//									validation_splitMatrix[gHyp_num*TRACKING_NUM_POINTS_MAX*(TRACKING_NUM_TRACKS_MAX + 1)+ \
//														   k*(TRACKING_NUM_TRACKS_MAX + 1)] = 0;
//									++gHyp_num;
//								}
//							}// m
//						} // if k != i
//					} // k
//				}
//			} // j
//		} // i
//	}
//#ifdef JPDAF_DEBUGLOG
//	printf("jpda gHyp_num: %d \n", gHyp_num);
//#endif
//}

void jpdaf_joint_probability(sTracking_moduleInstance *inst)
{
	uint8_t validation_matrix_joint[MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1)];
	uint8_t validation_splitMatrix[NUM_SPLIT_MATRIX_MAX*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1)];
	float validation_costMatrix_joint[MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1)];
	uint8_t tracker_pairs[MAX_JOINT_TRACKER_NUM*TRACKING_NUM_TRACKS_MAX];
	FILE* fp1 = NULL;
	uint8_t pairNum = 0;
	uint8_t i = 0;
	uint8_t valid_det_num = 0;
	uint8_t det_Idx[MAX_JOINT_DETECTION_NUM] = { 0 };
	uint8_t jointFlag = 0;
#ifdef PRINT_TXT
	FILE* fp1 = NULL;
#endif // PRINT_TXT
	memset(tracker_pairs, 0, sizeof(tracker_pairs));
	// 1.generate joint tracker pairs
	generate_joint_tracker_pairs(inst, tracker_pairs, &pairNum);
#ifdef DEBUG_LOG
	//my_printf("pairNum %d ",pairNum);
	for (i = 0; i < pairNum; i++)
	{
		int secondPair = 255;
		if (tracker_pairs[i*MAX_JOINT_TRACKER_NUM + 1] != secondPair)
		{
			secondPair = tracker_Idx[tracker_pairs[i*MAX_JOINT_TRACKER_NUM + 1]];
		}		
		//my_printf("iPair %d [%d %d] ", i, \
			tracker_Idx[tracker_pairs[i*MAX_JOINT_TRACKER_NUM]], secondPair);
	}
#endif // DEBUG_LOG

	for (i = 0; i < pairNum; i++)
	{
#ifdef DEBUG_LOG
		//my_printf("iPair %d [%d %d] ", i, \
			tracker_Idx[tracker_pairs[i*MAX_JOINT_TRACKER_NUM]], \
			tracker_Idx[tracker_pairs[i*MAX_JOINT_TRACKER_NUM + 1]]);
#endif // DEBUG_LOG
		gHyp_num = 0;
		memset(validation_matrix_joint, 0, sizeof(validation_matrix_joint));
		memset(validation_splitMatrix, 0, sizeof(validation_splitMatrix));
		memset(validation_costMatrix_joint, 0, sizeof(validation_costMatrix_joint));
		memset(prob_jointEventVector, 0, sizeof(prob_jointEventVector));
		jointFlag = tracker_pairs[i*MAX_JOINT_TRACKER_NUM + 1] == 255 ? 0 : 1;
	// 2.generate joint validation matrix and cost
		valid_det_num = generate_joint_validation_matrix_cost(i, tracker_pairs, \
			validation_matrix_joint, validation_costMatrix_joint, det_Idx);
	// 3.cal hypothesis and prob in pairs loop
		jpdaf_generate_hypothesis(jointFlag,valid_det_num,validation_matrix_joint, \
			validation_splitMatrix);
		// 1.calculate probabilities of each Joint Event
		joint_event_probability(inst, jointFlag, valid_det_num, \
			validation_splitMatrix, validation_costMatrix_joint, i, tracker_pairs, validation_matrix_joint);
		//jpdaf_event_probability(inst);
		// 2.calculate association probabilities
		association_probability( jointFlag,  valid_det_num, \
			 validation_splitMatrix, i, tracker_pairs, det_Idx);
	}
#ifdef PRINT_TXT
	fopen_s(&fp1, "D:/JPDAF.txt", "a+");
	for (i = 0; i < validation_detIdxNum + 1; i++)
	{
		int t = 0;
		for (; t < param_.trackers_size + 1; t++)
		{
			fprintf(fp1, "%.4f ", beta_matrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t]);
		}
		fprintf(fp1, "\n ");
	}
	fclose(fp1);
#endif
}


void joint_event_probability(sTracking_moduleInstance *inst, uint8_t jointFlag, uint8_t valid_det_num, \
	uint8_t* validation_splitMatrix, float* validation_costMatrix_joint,uint8_t i_pair,uint8_t*tracker_pairs,\
uint8_t* validation_matrix_joint)
{
	uint16_t i_jointEvent = 0;
	float lambda = 0.0001f;
	uint16_t phi = 0;
	uint16_t detNum = 0;
	uint8_t delta = 0, tau = 0;
	uint16_t i_meas = 0;
	sTracking_ListElem *tElem;
	uint16_t uid;
	uint8_t trackIdx;
	int i_tracker;
	float det;
	float M = 3.f;
	sTracking_objectUnit *pTracker = NULL;
	double prob_jointEventVector_sum = 0;
	double prob_tgt_det;
	double p_likelyhood;
	float md;
	double prior_prob = 0.;
	int i, t;
#ifdef PRINT_TXT
	FILE* fp1 = NULL;
	fopen_s(&fp1, "D:/JPDAF.txt", "a+");
	fprintf(fp1, "\n trackNum %d mesNum %d\n", jointFlag + 1, valid_det_num);
	for (i = 0; i < valid_det_num; i++)
	{
		for (t = 0; t < jointFlag + 2; t++)
		{
			fprintf(fp1, "%d ", validation_matrix_joint[i*(MAX_JOINT_TRACKER_NUM + 1) + t]);
		}
		fprintf(fp1, "\n ");
	}
	fclose(fp1);
#endif
	for (i_jointEvent = 0; i_jointEvent < gHyp_num; i_jointEvent++)
	{
		delta = 0;
		tau = 0;
		detNum = 0;
		prob_jointEventVector[i_jointEvent] = 1.f;
		i_meas = 0;
		
		// 1.calculate delta and tau

		// 2.
		for (i_meas = 0; i_meas < valid_det_num; i_meas++)
		{
			for (i_tracker = 0; i_tracker < jointFlag + 1; i_tracker++)
			{
				tElem = tracking_listGetFirst(&inst->tasTrackList);
				while (tElem != 0) {
					uid = tElem->data;
					pTracker = (sTracking_objectUnit *)inst->hTrack[uid];
					trackIdx = tracker_pairs[i_pair*MAX_JOINT_TRACKER_NUM + i_tracker];
					if (tracker_Idx[trackIdx] == pTracker->uid)
						break;
					tElem = tracking_listGetNext(tElem);
				}
				if (validation_splitMatrix[i_jointEvent*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
					i_meas*(MAX_JOINT_TRACKER_NUM + 1) + i_tracker + 1] == 1) {
					detNum++;
					md = validation_costMatrix_joint[i_meas*(MAX_JOINT_TRACKER_NUM + 1) + i_tracker + 1];
					// TODO: gC_det is calculate before update and is different for each measurement j
					/*letting 1/lambda be the uint volume*/
					det = sqrtf(fabsf(pTracker->assoProperty.gC_det));
					//det = sqrtf(fabsf(pTracker->assoProperty.gC_det))*lambda;
					p_likelyhood = exp(-0.5*md) / pow(2 * M_PI, M / 2.f) / det;
					prob_jointEventVector[i_jointEvent] *= p_likelyhood;
					break;
				}

			}
			
		}
		phi = valid_det_num - detNum;
#ifdef DEBUG_LOG
		//my_printf("i_jointEvent %d  %f", i_jointEvent, prob_jointEventVector[i_jointEvent]);
#endif // DEBUG_LOG
		/*letting 1/lambda be the uint volume*/ 
		prior_prob = pow(lambda, phi);
		prob_jointEventVector[i_jointEvent] *= prior_prob;

#ifdef DEBUG_LOG
		//my_printf("i_jointEvent %d  %f", i_jointEvent, prob_jointEventVector[i_jointEvent]);
#endif // DEBUG_LOG
		if (fabsf(param_.pd - 1.f) < 1E-10)
		{
			prob_tgt_det = 1.f;
		}
		else
		{
			prob_tgt_det = pow(param_.pd, detNum)*pow((1. - param_.pd), (param_.trackers_size - detNum));
		}
		prob_jointEventVector[i_jointEvent] *= prob_tgt_det;
		prob_jointEventVector_sum += prob_jointEventVector[i_jointEvent];
	}
#ifdef DEBUG_LOG
	//my_printf("gHyp_num %d probSum %f", gHyp_num, prob_jointEventVector_sum);
#endif // DEBUG_LOG
	i_jointEvent = 0;
	for (i_jointEvent = 0; i_jointEvent < gHyp_num; i_jointEvent++)
	{
		prob_jointEventVector[i_jointEvent] = prob_jointEventVector[i_jointEvent] / prob_jointEventVector_sum;
#ifdef DEBUG_LOG
		//my_printf("[%d  %f]", i_jointEvent, prob_jointEventVector[i_jointEvent]);
#endif // DEBUG_LOG
	}
}

void association_probability(uint8_t jointFlag, uint8_t valid_det_num, \
	uint8_t* validation_splitMatrix, uint8_t i_pair, uint8_t*tracker_pairs,uint8_t*det_Idx)
{
	uint16_t i = 1;
	uint16_t j = 0;
	uint8_t trackIdx;
	float beta_sumPerTracker;
	for (; i < jointFlag + 2; ++i)
	{
		trackIdx = tracker_pairs[i_pair*MAX_JOINT_TRACKER_NUM + i - 1];
		beta_sumPerTracker = 0;
		for (j = 0; j < valid_det_num; ++j)
		{
			int k = 0;
			for (; k < gHyp_num; ++k)
			{
				beta_matrix[det_Idx[j] * (TRACKING_NUM_TRACKS_MAX + 1) + trackIdx + 1] += prob_jointEventVector[k] * \
					validation_splitMatrix[k*MAX_JOINT_DETECTION_NUM*(MAX_JOINT_TRACKER_NUM + 1) + \
					j*(MAX_JOINT_TRACKER_NUM + 1) + i];
			}
			beta_sumPerTracker += beta_matrix[det_Idx[j] * (TRACKING_NUM_TRACKS_MAX + 1) + trackIdx + 1];
		}
		// beta_0: no measurement is associated with current tracker
		beta_matrix[validation_detIdxNum*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx + 1] =\
			1.0f - beta_sumPerTracker;
#ifdef DEBUG_LOG
		//my_printf("******** JPDAF tracker: %d, beta_0: %.5f \n", i, 1.0f - beta_sumPerTracker);
#endif
	}
}

/**
*  @b Description
*  @n
*		GJPDA-EKF Module calls this function to perform an update step for the tracking unit. 
*
*  @param[in]  handle
*		This is handle to TRACKING unit
*  @param[in]  point
*		This is an array of measurement points
*  @param[in]  var
*      Pointer to an array of input measurment variances. Shall be set to NULL if variances are unknown
*  @param[in]  pInd
*		This is an array of associated UIDs. After association and allocation steps, each measurment shall have a UID assigned.
*  @param[in]  num
*		Number of measurement points
*
*  \ingroup TRACKING_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

sTracking_enum_state jpdaf_unitUpdate(void *handle, sMeasurement_SphPoint *point,
                               uint8_t tracker_idx, uint8_t newTracker_flag, sTracking_platformInfo* platformInfo)
{
	sTracking_objectUnit *pTrack;
	uint16_t j_mea = 0;
	
	float K[SSIZE_XY*MSIZE_SPH] = { 0.f };
	/* Rm is Measurement error covariance matrix */
	
	float y[MSIZE_SPH] = { 0.f };
	float Ky[SSIZE_XY] = { 0.f };
	float S_j_mea[SSIZE_XY] = { 0.f };
	float S_joint[SSIZE_XY] = { 0.f };
	float S_joint_beta_jt[SSIZE_XY] = { 0.f };
	float beta_jt;
	float S_ST[SSIZE_XY*SSIZE_XY] = { 0.f };
	float S_ST_temp[SSIZE_XY*SSIZE_XY] = { 0.f };
	uint8_t assoFlag = 0; 
	float P_[SSIZE_XY*SSIZE_XY] = { 0.f };
	float J[MSIZE_SPH*SSIZE_XY] = { 0.f };
	float PJ[MSIZE_SPH*SSIZE_XY] = { 0.f };
	float JPJ[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float Rm[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float S[MSIZE_SPH*MSIZE_SPH] = { 0.f };
	float KSK[SSIZE_XY*SSIZE_XY] = { 0 };
	float KS[SSIZE_XY*MSIZE_SPH] = { 0 };
	float KSK_beta[SSIZE_XY*SSIZE_XY] = { 0 };
	float P_temp1[SSIZE_XY*SSIZE_XY] = { 0 };
	float P_temp[SSIZE_XY*SSIZE_XY] = { 0 };
	float beta_0t;
	float S_S_joint[SSIZE_XY*SSIZE_XY] = { 0 };
	float Meas_var[MSIZE_SPH] = { 0.f };
	float fVx_tmp, fVy_tmp, fYawRate;
	float detMovingFlag = 0;
	fYawRate = platformInfo->egoOmega;
	pTrack = (sTracking_objectUnit *)handle;
	memset(pTrack->assoProperty.bitIndicatorsOfDetAsso, 0, sizeof(pTrack->assoProperty.bitIndicatorsOfDetAsso));
	tracking_matrixUperTri2Matrix(pTrack->kalman_state.P_hat,P_);
	/* Compute state vector partial derivatives (Jacobian matrix) */
	tracking_computeJacobian(pTrack->tracking_params->stateVectorType, pTrack->kalman_state.S_hat, J);
	/* Compute JPJ' = J(:,1:mSize) * obj.P_apriori(1:mSize,1:mSize) * J(:,1:mSize)' */
	tracking_matrixComputePJT(P_, J, PJ);
	tracking_matrixMultiply(MSIZE_SPH, SSIZE_XY, MSIZE_SPH, J, PJ, JPJ);
	tracking_measurement_variance_cal(pTrack, const_tws_measurement_noise, Meas_var);
	tracking_matrixSetDiag(MSIZE_SPH, Meas_var, Rm);
	tracking_matrixAdd(MSIZE_SPH, MSIZE_SPH, JPJ, Rm, S);
	tracking_matrixMultiply(SSIZE_XY, MSIZE_SPH, MSIZE_SPH, PJ, pTrack->assoProperty.gC_inv, K);
	for (j_mea = 0; j_mea < validation_detIdxNum; j_mea++)
	{
		beta_jt = beta_matrix[j_mea*(TRACKING_NUM_TRACKS_MAX + 1) + tracker_idx];
		if (beta_jt > 0.f)
		{
			// 1.2 calculate x(k|k)_(j_meas): as tracker associate with jth measurement only
			tracking_matrixSub(MSIZE_SPH, 1, point[validation_detIdx[j_mea]].array, pTrack->kalman_state.H_s.array, y);
			tracking_matrixMultiply(SSIZE_XY, MSIZE_SPH, 1, K, y, Ky);
			tracking_matrixAdd(SSIZE_XY, 1, pTrack->kalman_state.S_hat, Ky, S_j_mea);

			// 1.3 update x(k|k) with x(k|k)_(j_meas) and beta(tracker_idx,j_meas)
			tracking_matrixScalarMultiply(SSIZE_XY, 1, S_j_mea, beta_jt, S_joint_beta_jt);
			tracking_matrixAdd(SSIZE_XY, 1, S_joint_beta_jt, S_joint, S_joint);

			tracking_matrixTransposeMultiply(SSIZE_XY, 1, SSIZE_XY, S_joint_beta_jt, S_j_mea, S_ST_temp);
			tracking_matrixAdd(SSIZE_XY, SSIZE_XY, S_ST_temp, S_ST, S_ST);
			assoFlag = 1;
			pTrack->assoProperty.bitIndicatorsOfDetAsso[validation_detIdx[j_mea] >> 3] |= (1 << (validation_detIdx[j_mea] & 0x7));
			if (point[validation_detIdx[j_mea]].flag_moving == 1U)
			{
				detMovingFlag = 1U;
			}
#ifdef DEBUG_LOG
			//my_printf("t j beta [%d %d %f] Ky [%.2f %.2f %.2f %.2f ]", \
				pTrack->uid, validation_detIdx[j_mea], beta_jt, Ky[0], Ky[1], Ky[2], Ky[3]);
			//my_printf("t j beta [%d %d %f] ", \
				pTrack->uid, validation_detIdx[j_mea], beta_jt);
#endif // DEBUG_LOG

		}
	}
	//1.4 update x(k|k) with x(k|k)_(j0) and beta(tracker_idx,j_0),means no association happen
	beta_0t = beta_matrix[validation_detIdxNum*(TRACKING_NUM_TRACKS_MAX + 1) + tracker_idx];

	tracking_matrixScalarMultiply(SSIZE_XY, 1, pTrack->kalman_state.S_hat, beta_0t, S_joint_beta_jt);
	tracking_matrixAdd(SSIZE_XY, 1, S_joint_beta_jt, S_joint, S_joint);
#ifdef DEBUG_LOG
	//my_printf("t 0 [%d %f]  x %.2f dx %.2f ", pTrack->uid, beta_0t, pTrack->kalman_state.S_hat[0], \
		S_joint_beta_jt[0]);
#endif // DEBUG_LOG
	tracking_matrixTransposeMultiply(SSIZE_XY, 1, SSIZE_XY, S_joint_beta_jt, pTrack->kalman_state.S_hat, S_ST_temp);
	tracking_matrixAdd(SSIZE_XY, SSIZE_XY, S_ST_temp, S_ST, S_ST);

	memcpy(pTrack->kalman_state.S_hat, S_joint, sizeof(S_joint));
	tracking_matrixTransposeMultiply(SSIZE_XY, 1, SSIZE_XY, pTrack->kalman_state.S_hat, pTrack->kalman_state.S_hat, S_S_joint);
	/* Calculate KSK'*/
	tracking_matrixMultiply(SSIZE_XY, MSIZE_SPH, MSIZE_SPH, K, S, KS);
	tracking_matrixTransposeMultiply(SSIZE_XY, MSIZE_SPH, SSIZE_XY, KS, K, KSK);
	tracking_matrixScalarMultiply(SSIZE_XY, SSIZE_XY, KSK, (beta_0t - 1), KSK_beta);
	tracking_matrixAdd(SSIZE_XY, SSIZE_XY, KSK_beta, P_, P_temp1);
	tracking_matrixAdd(SSIZE_XY, SSIZE_XY, P_temp1, S_ST, P_temp);
	tracking_matrixSub(SSIZE_XY, SSIZE_XY, P_temp, S_S_joint, P_);
	tracking_matrixMatrix2UperTri(P_, pTrack->kalman_state.P_hat);
	/*if ((pTrack->eDynamicProperty == TRACKING_DYNAMIC_STATIC || pTrack->eDynamicProperty == TRACKING_DYNAMIC_STOP) &&
		(fabsf(platformInfo->egoLineVel_long) > 1.f || fabsf(platformInfo->egoLineVel_lat) > 1.f))
	{
		fVx_tmp = -1.f*platformInfo->egoLineVel_long + fYawRate * (pTrack->kalman_state.S_hat[1] + platformInfo->radarDist_lat);
		fVy_tmp = -1.f*platformInfo->egoLineVel_lat - fYawRate * (pTrack->kalman_state.S_hat[0] + platformInfo->radarDist_long);
		//lowPassFilter, x = k*input + (1-k)*x
		myMath_lowPassFilter(&pTrack->kalman_state.S_hat[2], fVx_tmp, 0.7f);
		myMath_lowPassFilter(&pTrack->kalman_state.S_hat[3], fVy_tmp, 0.7f);
		//myMath_minMax(&inst->kalman_state.S_hat[2], -1.f*TRACKING_MIN_STATIC_VELOCITY, TRACKING_MIN_STATIC_VELOCITY);
		//myMath_minMax(&inst->kalman_state.S_hat[3], -1.f*TRACKING_MIN_STATIC_VELOCITY, TRACKING_MIN_STATIC_VELOCITY);
		pTrack->kalman_state.S_hat[4] = platformInfo->egoLineAcc_long; //0.f;
		pTrack->kalman_state.S_hat[5] = platformInfo->egoLineAcc_lat; //0.f;
	}*/

	// Trackers manager
	if (assoFlag == 1)
	{
		if (detMovingFlag == 1)
		{
			pTrack->curTarget.uFlagMoving = 1;
			//pTrack->objGeometryProperty.assocMovingCounter++;
		}
		else
		{
			pTrack->curTarget.uFlagMoving = 0;
		}
	}
	tracking_calculateTrackerAbsVelocity(pTrack, platformInfo);
	//pTrack->objGeometryProperty.historyTrajAbsLength += \
		sqrtf(fabsf(pTrack->kalman_state.fVabsX*pTrack->tracking_params->dt) \
			* fabsf(pTrack->kalman_state.fVabsX*pTrack->tracking_params->dt) \
			+ fabsf(pTrack->kalman_state.fVabsY*pTrack->tracking_params->dt) \
			* fabsf(pTrack->kalman_state.fVabsY*pTrack->tracking_params->dt));
	tracking_unitEvent(pTrack, assoFlag, platformInfo);
#ifdef DEBUG_LOG
	//my_printf("id %d x y vx vy [%.2f %.2f %.2f %.2f ] assoFlag %d detc2free ac2free [%d %d] state %d", \
		pTrack->uid, pTrack->kalman_state.S_hat[0], pTrack->kalman_state.S_hat[1], \
		pTrack->kalman_state.S_hat[2], pTrack->kalman_state.S_hat[3], assoFlag, \
		pTrack->objManagement.detect2freeCount, pTrack->objManagement.active2freeCount, pTrack->objManagement.state);
#endif // DEBUG_LOG
	return(pTrack->objManagement.state);
}
void generate_joint_tracker_pairs(sTracking_moduleInstance *inst, uint8_t* tracker_pairs, uint8_t* pairNum)
{
	sTracking_ListElem *itElem;
	sTracking_ListElem *jtElem;
	uint16_t uid;
	sTracking_objectUnit *iTracker = NULL;
	sTracking_objectUnit *jTracker = NULL;
	uint8_t visited[TRACKING_NUM_TRACKS_MAX] = { 0 };
	uint8_t pairFlag = 0;
	uint8_t i, j;
	for (i = 0; i < validation_trackerIdxNum; i++)
	{
		if (visited[i] == 1)
			continue;
		pairFlag = 0;
		itElem = tracking_listGetFirst(&inst->tasTrackList);
		while (itElem != 0) {
			uid = itElem->data;
			iTracker = (sTracking_objectUnit *)inst->hTrack[uid];
			if (iTracker->uid == tracker_Idx[i])
			{
				visited[i] = 1;
				break;
			}
			itElem = tracking_listGetNext(itElem);
		}
		for (j = i + 1; j < validation_trackerIdxNum; j++)
		{
			if (visited[j] == 1)
				continue;
			jtElem = tracking_listGetFirst(&inst->tasTrackList);
			while (jtElem != 0) {
				uid = jtElem->data;
				jTracker = (sTracking_objectUnit *)inst->hTrack[uid];
				if (jTracker->uid == tracker_Idx[j])
					break;
				jtElem = tracking_listGetNext(jtElem);
			}
			if (fabsf(iTracker->kalman_state.S_hat[0] - jTracker->kalman_state.S_hat[0]) < 3.0
				&&fabsf(iTracker->kalman_state.S_hat[1] - jTracker->kalman_state.S_hat[1]) < 3.0
				&&fabsf(iTracker->kalman_state.S_hat[2] - jTracker->kalman_state.S_hat[2]) < 2.0
				&&fabsf(iTracker->kalman_state.S_hat[3] - jTracker->kalman_state.S_hat[3]) < 2.0)
			{
				visited[j] = 1;
				visited[i] = 1;
				pairFlag = 1;
				tracker_pairs[(*pairNum)*(MAX_JOINT_TRACKER_NUM)] = i;
				tracker_pairs[(*pairNum)*(MAX_JOINT_TRACKER_NUM)+1] = j;
				(*pairNum)++;
#ifdef DEBUG_LOG
				//my_printf("pair [%d %d]", tracker_Idx[i], tracker_Idx[j]);
#endif // DEBUG_LOG

				break;
			}
		}
		if (pairFlag == 0)
		{
			visited[i] = 1;
			tracker_pairs[(*pairNum)*(MAX_JOINT_TRACKER_NUM)] = i;
			tracker_pairs[(*pairNum)*(MAX_JOINT_TRACKER_NUM)+1] = 255;
			(*pairNum)++;
		}

	}
}
uint8_t generate_joint_validation_matrix_cost(uint8_t i_pair, uint8_t* tracker_pairs, \
	uint8_t* validation_matrix_joint,float*  validation_costMatrix_joint,uint8_t*det_Idx )
{
	uint8_t valid_det_num = 0;
	int i = 0, j = 0;
	uint8_t trackIdx1 = 0;
	uint8_t trackIdx2 = 0;
	uint8_t jointFlag = 0;
	jointFlag = tracker_pairs[i_pair*MAX_JOINT_TRACKER_NUM + 1] == 255 ? 0 : 1;
	if (jointFlag)
	{
		for (j = 0; j < validation_detIdxNum; j++)
		{
			trackIdx1 = tracker_pairs[i_pair*MAX_JOINT_TRACKER_NUM + 0];
			trackIdx2 = tracker_pairs[i_pair*MAX_JOINT_TRACKER_NUM + 1];
			if (validation_matrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx1 + 1] == 1\
				|| validation_matrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx2 + 1] == 1)
			{
				validation_matrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 0] = 1;
				validation_matrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 1] = \
					validation_matrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx1 + 1];
				validation_matrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 2] = \
					validation_matrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx2 + 1];
				validation_costMatrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 1] = \
					validation_costMatrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx1 + 1];
				validation_costMatrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 2] = \
					validation_costMatrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx2 + 1];
				det_Idx[valid_det_num] = j;
				valid_det_num++;
			}
		}
	}
	else // jointFlag==0
	{
		for (j = 0; j < validation_detIdxNum; j++)
		{
			trackIdx1 = tracker_pairs[i_pair*MAX_JOINT_TRACKER_NUM + 0];
			if (validation_matrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx1 + 1] == 1)
			{
				validation_matrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1)] = 1;
				validation_matrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 1] = \
					validation_matrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx1 + 1];
				validation_costMatrix_joint[valid_det_num*(MAX_JOINT_TRACKER_NUM + 1) + 1] = \
					validation_costMatrix[j*(TRACKING_NUM_TRACKS_MAX + 1) + trackIdx1 + 1];
				det_Idx[valid_det_num] = j;
				valid_det_num++;
			}
		}
	}
	return valid_det_num;
}

void validation_matrix_reduce()
{
	uint8_t i = 0, t = 1, k = 0, n = 0;
	uint8_t assNum = 0;
	uint8_t assFlag = 0;
	float mdScore[TRACKING_NUM_POINTS_MAX] = { 0.f };
	float tmp;
	for (t = 1; t < validation_trackerIdxNum + 1; t++)
	{
		assNum = 0;
		memset(mdScore, 0, sizeof(mdScore));
		for (i = 0; i < validation_detIdxNum; i++)
		{
			assFlag = validation_matrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t];
			if (assFlag)
			{
				mdScore[assNum]= validation_costMatrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t];
				assNum++;
			}
			
		}
		if (assNum > MAX_ASSO_DET_NUM)
		{
			for (k = 0; k < MAX_ASSO_DET_NUM; k++)
			{
				for (n = 0; n < assNum - k - 1; n++)
				{
					if (mdScore[assNum - n - 1] < mdScore[assNum - n - 2])
					{
						tmp = mdScore[assNum - n - 1];
						mdScore[assNum - n - 1] = mdScore[assNum - n - 2];
						mdScore[assNum - n - 2] = tmp;
					}
				}
				
			}
			for (i = 0; i < validation_detIdxNum; i++)
			{
				if (validation_costMatrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t] > mdScore[MAX_ASSO_DET_NUM - 1])
				{
					validation_matrix[i*(TRACKING_NUM_TRACKS_MAX + 1) + t] = 0;
				}
			}
		}

	}
	
}
#endif
