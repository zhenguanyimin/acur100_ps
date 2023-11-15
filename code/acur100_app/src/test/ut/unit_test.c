
#include "unit_test.h"
#include "ut_0165_drv.h"
#include "ut_chn_ctrl.h"
#include "ut_sys_ctrl.h"
#include "ut_ant_test.h"
#include "ut_protocol.h"
#include "ut_cfg_param.h"
#include "ut_datapath_test.h"
#include "ut_version.h"
#include "ut_track_test.h"

void uint_test_init(void)
{
	ut_0165_drv_init();

	ut_chn_ctrl_init();

	ut_sys_ctrl_init();

	ut_ant_test_init();

	ut_protocol_init();

	ut_cfg_param_init();

	ut_datapath_init();

	ut_version_init();

	ut_track_init();

	return;
}
