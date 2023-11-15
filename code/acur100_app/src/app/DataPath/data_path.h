
#ifndef __DATA_PATH_H__
#define __DATA_PATH_H__

#include "xil_types.h"
#include "../../hal/DataTransfer/DataTransfer.h"

typedef struct {
	u32 flag;   //1 for in using ,0 for idle
	u32 type;
	u32 ts;
	u32 length; //length of the pData
	u8 *pData;
}buffer_t;

typedef struct {
	buffer_t buff1;
	buffer_t buff2;
	volatile  u32 revLen;
	u32 usedId ;
}DataPingPong_t;

typedef void (*DataPathEvCb)(u8 * pBuf ,s32 len);

s32 data_path_init(void);

s32 data_path_start(s32 type);

s32 data_path_stop(s32 type);

void data_path_out_en(s32 type);

s32 data_path_setEvCallBack(DataPathEvCb pCb); //for dectect process ,return immediately

#endif
