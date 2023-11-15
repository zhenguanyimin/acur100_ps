
#ifndef __DATA_STRANSFER_H__
#define __DATA_STRANSFER_H__

#include "xil_types.h"
#include "xaxidma.h"
#include "xscugic.h"

#define ADC_DATA 1
#define RFFT_DATA 2
#define DFFT_DATA 3
#define RDMAP_DATA 4
#define DATA_TYPE_MAX 5

#define ADC_PACKET_LEN (2*4096) //must be keep the same with pl
#define RDM_PACKET_LEN (2*4096*32)

typedef void (*DataTransCallback)(s32 type, s32 len);

s32 data_trans_init(s32 type);//set buffer & init dma

void data_trans_deInit(s32 type);

s32 data_trans_start(s32 type,u8* pBuf ,s32 len); //for one time transfer

s32 data_trans_stop(s32 type);

s32 data_trans_set_callback(s32 type,DataTransCallback cb);

void data_trans_CacheInvalidate(u8* pBuf ,s32 len);

void data_trans_CacheFlush(u8* pBuf ,s32 len);

s32 data_trans_status(s32 type) ;

s32 data_trans_reset(s32 type) ;

#endif
