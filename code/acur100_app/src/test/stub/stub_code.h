
#ifndef STUB_CODE_H
#define STUB_CODE_H

#include <stdint.h>

typedef int32_t ret_t;

typedef enum stub_code_ret {
	STUB_RET_OK = 0,
	STUB_RET_ERR = 1,
} stub_code_ret_t;

/*
 * Note: The parameters are not checked in the stub code.
 * */

ret_t stub_code_freq_num_set(uint32_t freq_num);
ret_t stub_code_freq_num_get(uint32_t *freq_num);

ret_t stub_code_tx_chn_set(uint32_t row_num, uint32_t col_num, uint32_t chn_state, uint32_t phase_code);
ret_t stub_code_tx_chn_get(uint32_t row_num, uint32_t col_num, uint32_t *chn_state, uint32_t *phase_code);

ret_t stub_code_rx_chn_set(uint32_t row_num, uint32_t col_num, uint32_t chn_state, uint32_t phase_code, uint32_t amp_com_code, uint32_t amp_chn_code);
ret_t stub_code_rx_chn_get(uint32_t row_num, uint32_t col_num, uint32_t *chn_state, uint32_t *phase_code, uint32_t *amp_com_code, uint32_t *amp_chn_code);

ret_t stub_code_close_all_chn(void);

ret_t stub_code_tx_ant_test(uint32_t freq_num, float az_deg, float el_deg);
ret_t stub_code_rx_ant_test(uint32_t freq_num, float az_deg, float el_deg);

#endif /* STUB_CODE_H */
