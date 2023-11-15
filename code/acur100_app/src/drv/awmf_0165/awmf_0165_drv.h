
#ifndef AWMF_0165_DRV_H
#define AWMF_0165_DRV_H

#include "awmf_0165_def.h"

typedef struct awmf_0165_chn_info {
	uint32_t chn_state;
	uint32_t phase_code;
	uint32_t amp_com_code;
	uint32_t amp_chn_code;
} awmf_0165_chn_info_t;
#define IS_0165_CHN_STATE_VALID(val) (((val) >= 0) && ((val) <= 1))
#define IS_0165_PHASE_CODE_VALID(val) (((val) >= 0) && ((val) <= 63))
#define IS_0165_AMP_COM_CODE_VALID(val) (((val) >= 0) && ((val) <= 15))
#define IS_0165_AMP_CHN_CODE_VALID(val) (((val) >= 0) && ((val) <= 15))

ret_code_t awmf_0165_drv_init(void);

ret_code_t awmf_0165_drv_open(void);
ret_code_t awmf_0165_drv_close(void);

ret_code_t awmf_0165_drv_tx_chn_set(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info);
ret_code_t awmf_0165_drv_tx_chn_get(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info);

ret_code_t awmf_0165_drv_rx_chn_set(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info);
ret_code_t awmf_0165_drv_rx_chn_get(uint32_t row_num, uint32_t col_num, awmf_0165_chn_info_t *chn_info);

ret_code_t awmf_0165_drv_close_all_chn(void);

ret_code_t awmf_0165_drv_workmode_set(uint32_t chip_id, uint32_t work_mode);
ret_code_t awmf_0165_drv_workmode_get(uint32_t chip_id, uint32_t *work_mode);

ret_code_t awmf_0165_drv_reg_set(uint32_t chip_id, uint32_t reg_addr, uint32_t data_high, uint32_t data_low);
ret_code_t awmf_0165_drv_reg_get(uint32_t chip_id, uint32_t reg_addr, uint32_t *data_high, uint32_t *data_low);

#endif /* AWMF_0165_DRV_H */
