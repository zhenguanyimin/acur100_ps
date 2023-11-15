
#ifndef AWMF_0165_IO_H
#define AWMF_0165_IO_H

#include "awmf_0165_def.h"

ret_code_t awmf_0165_drv_io_init(void);
ret_code_t awmf_0165_drv_io_chain_ctrl(uint32_t chain_id, uint32_t wr_rd, uint32_t mode);
ret_code_t awmf_0165_drv_io_chip_ctrl(uint32_t chip_id, uint32_t work_mode);
ret_code_t awmf_0165_drv_io_chip_query(uint32_t chip_id, uint32_t *work_mode);

#endif /* AWMF_0165_IO_H */
