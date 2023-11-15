
#include <string.h>

#include "../include/tracking_common.h"
#include "../include/tracking_int.h"


/**
*  @b Description
*  @n
*		tracking Module calls this function to get target expected measurement matrix, H_s.
*
*  @param[in]  handle
*		This is handle to tracking unit
*  @param[in]  pH
*		This is a pointer to the measurement array H that is filled by the function
*
*  \ingroup tracking_ALG_UNIT_FUNCTION
*
*  @retval
*      None
*/

void tracking_unitGetH(void *handle, float *pH)
{
    sTracking_objectUnit *inst;

    inst = (sTracking_objectUnit *)handle;	
    memcpy(pH, inst->kalman_state.H_s.array, sizeof(inst->kalman_state.H_s.array));
}
