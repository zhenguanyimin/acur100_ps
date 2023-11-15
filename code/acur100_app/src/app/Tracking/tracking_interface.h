#ifndef TRACKING_INTERFACE_H_
#define TRACKING_INTERFACE_H_


#include "src/utilities/common_struct.h"
#include "src/alg_init/alg_init.h"
#include "../../srv/protocol/protocol_if.h"

extern int gAlgInitFlag;


void runTrackingAlg();

void trackingAlgProcess(protocol_object_list_detected_t *detect_list, \
		protocol_object_list_tracked_t *track_list, \
		protocol_beam_scheduling_t *beam_scheduling, \
		protocol_radar_platfrom_info_t *platformInfo, \
		protocol_cfg_param_t *configParmInfo, \
		protocol_radar_status_t *radar_status);

void process_dummy3D(protocol_object_list_detected_t *detect_list, \
		protocol_object_list_tracked_t *track_list, \
		protocol_beam_scheduling_t *beam_scheduling, \
		protocol_radar_platfrom_info_t *platformInfo, \
		protocol_radar_status_t *radar_status);

void trackingReport(protocol_object_list_detected_t *detect_list, \
		protocol_object_list_tracked_t *track_list, \
		protocol_beam_scheduling_t *beam_scheduling, \
		protocol_radar_status_t *radar_status);

void trans_byte_order_obj_list_detected(protocol_object_list_detected_t *detect_list);
void trans_byte_order_obj_list_tracked(protocol_object_list_tracked_t *track_list);
void trans_byte_order_beam_scheduling(protocol_beam_scheduling_t *beam_scheduling);
void trans_byte_order_radar_status(protocol_radar_status_t *radar_status);

#endif
