//alg_init.h

#ifndef ALG_INIT_H_
#define ALG_INIT_H_

#include "../utilities/common_struct.h"


#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif

char* getTrackAlgVersion();
int measProcAlgInstance_init(sAlgObjData *algObj);
int trackingAlgInstance_init(sAlgObjData *algObj);
int dispatchAlgInstance_init(sAlgObjData *algObj);

void moduleMemCalculate(sAlgObjData *algObj);

sAlgObjData* alg_init();
void alg_process(sAlgObjData* algObj);
void module_process(sAlgObjData* algObj);
//void detectsListInput(sAlgObjData* algObj, sDetectFrame* radar_det);
void setDebugAndCtrlParam(sAlgObjData* algObj, sPlatformInfo* platformInfo, sDebugAndCtrlParam* ctrlInfo);
void setDummyFlag(int flag);
extern sPlatformInfo platformInfo_malloc[1];
extern int gAlgInitFlag;

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif


#endif // ALG_INIT_H_
