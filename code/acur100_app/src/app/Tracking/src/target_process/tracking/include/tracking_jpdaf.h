
/* Data association with JPDA
   Output the Joint probability matrix
*/

#ifndef TRACKING_JPDA_H
#define TRACKING_JPDA_H

#include "../../../utilities/common_struct.h"
#include "tracking_common.h"

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif

#include "../../../utilities/myMath.h"

//#ifndef ENABLE_PORTING
//#include "../../../utilities/ringbuffer.h"
//#include "../../../utilities/slog.h"
//#endif

#ifdef JPDAF_ENABLE
#define NUM_SPLIT_MATRIX_MAX (MAX_ASSO_DET_NUM*(MAX_ASSO_DET_NUM+2)+1)

typedef struct sJpda_param {
	uint8_t dets_size;
	uint8_t trackers_size;
	float global_assocSpatialCost;
	float global_assocCost;
	float pd;
}sJpdaf_param;

extern float beta_matrix[];

void jpdaf_init(sJpdaf_param in_param);
void jpdaf_validation_matrix(sTracking_moduleInstance *inst, \
	sMeasurement_SphPoint *point, uint16_t num);
void jpdaf_innovation_matrix(sTracking_moduleInstance *inst);
void jpdaf_gating_md(sMeasurement_SphPoint *measurement, \
	sTracking_objectUnit *track, float *mah, float *mah_p);
void jpdaf_generate_hypothesis(uint8_t jointFlag, uint8_t valid_det_num, uint8_t* validation_matrix_joint, uint8_t* validation_splitMatrix);
void jpdaf_joint_probability(sTracking_moduleInstance *inst);
void joint_event_probability(sTracking_moduleInstance *inst, uint8_t jointFlag, uint8_t valid_det_num, \
	uint8_t* validation_splitMatrix, float* validation_costMatrix_joint, uint8_t i_pair, uint8_t*tracker_pairs, uint8_t* validation_matrix_joint);
void association_probability(uint8_t jointFlag, uint8_t valid_det_num, \
	uint8_t* validation_splitMatrix, uint8_t i_pair, uint8_t*tracker_pairs, uint8_t*det_Idx);
sTracking_enum_state jpdaf_unitUpdate(void *handle, sMeasurement_SphPoint *point,\
	uint8_t tracker_idx, uint8_t newTracker_flag, sTracking_platformInfo* platformInfo);
void generate_joint_tracker_pairs(sTracking_moduleInstance *inst, uint8_t* tracker_pairs, uint8_t* pairNum);
uint8_t generate_joint_validation_matrix_cost(uint8_t i_pair, uint8_t* tracker_pairs, \
	uint8_t* validation_matrix_joint, float*  validation_costMatrix_join, uint8_t*det_Idx);
void jpdaf_dataAssoc(sTracking_moduleInstance *inst, sMeasurement_SphPoint *point, uint16_t num);
void jpdaf_stateUpdate(sTracking_moduleInstance *inst, sMeasurement_SphPoint *point, uint16_t num);
void validation_matrix_reduce();
#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif

#endif

#endif /* TRACKING_CONFCOEF_H */
