


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "netif/xadapter.h"

#include "data_path.h"
#include "../../srv/protocol/protocol_dbgdat.h"
#include "../app_init.h"
#include "../Tracking/tracking_interface.h"


#define RAW_ADC_LEN (256*1024)
#define RDMAP_DATA_LEN (256*1024)
#define DDR_BASE_ADDR (0x10000000)
#define ADC_BASE_ADDR (DDR_BASE_ADDR + 0x6400000)
#define RDM_BASE_ADDR (ADC_BASE_ADDR + 2*(RAW_ADC_LEN))

enum
{
	BUFF_STAT_START,
	BUFF_EMPTY,
	BUFF_IN_USED,
	BUFF_FULL,
	BUFF_STAT_MAX
};

static QueueHandle_t sQueueHandle ;
static s32 sFrameID = 0  ;
static s32 sDataPathOutEn = 0 ;
//for ADC
static DataPingPong_t sAdcBuf = {0} ;
static protocol_adc_data_t sAdcSendBuf = {0};

//for rdm
static DataPingPong_t sRdmBuf = {0} ;
static DataPathEvCb pDetectCb = NULL ;

extern enum ethernet_link_status eth_link_status;

void set_buffer_empty(s32 type,void *data)
{
	buffer_t *pBuff = NULL ;

	if (ADC_DATA == type)
		pBuff = &sAdcBuf.buff1;

	pBuff = (pBuff->pData == (u8*)data)?(&sAdcBuf.buff1):(&sAdcBuf.buff2);

	pBuff->flag = BUFF_EMPTY ;
}

/**
 * 1、监控pl状态是否正常
 * 2、处理pl过来的数据，上报或者调用后处理算法
 * 3、pl异常后，使其恢复。并上报状态
 */
void data_path_task( void *pvParameters )
{
	buffer_t BufMsg =  {0} ;
	BaseType_t ret = pdFALSE ;
	u32 num =0 ;

	while(1)
	{
		ret = xQueueReceive( sQueueHandle , &BufMsg , pdMS_TO_TICKS(30));
		if (pdTRUE == ret )
		{
//			for( num = 0 ;num < RAW_ADC_LEN ; num = num +8*1024)
//				data_trans_CacheInvalidate(BufMsg.pData + num , 8*1024);

			if ((ADC_DATA == BufMsg.type)&&(1==sDataPathOutEn))
			{
				memcpy(sAdcSendBuf.rawData , BufMsg.pData , RAW_ADC_LEN );

				sAdcSendBuf.rawData[0] = (uint16_t)(asinf(gBeamInfo[0].aziBeamSin * INV_ONE15FORMAT_SCALE) * RAD2DEG + 0.5 + 180);
				sAdcSendBuf.rawData[1] = (uint16_t)(asinf(gBeamInfo[0].eleBeamSin * INV_ONE15FORMAT_SCALE) * RAD2DEG + 0.5 + 180);
				runTrackingAlg();

				protocol_adcpkg_set_head(&sAdcSendBuf,sFrameID ,RAW_ADC_LEN);
				protocol_send_adc_data(&sAdcSendBuf);

			}
			else if (RDMAP_DATA == BufMsg.type)
			{

				if (pDetectCb)
				{
					pDetectCb(BufMsg.pData, RDMAP_DATA_LEN);
				}
				if (3==sDataPathOutEn)
				{
					memcpy(sAdcSendBuf.rawData , BufMsg.pData , RAW_ADC_LEN );
					protocol_adcpkg_set_head(&sAdcSendBuf,sFrameID ,RDMAP_DATA_LEN);
					protocol_send_adc_data(&sAdcSendBuf);
				}
			}
		}
	}
}

void data_path_buff_init(s32 type)
{
	if (ADC_DATA == type)
	{
		sAdcBuf.revLen = 0 ;
		sAdcBuf.buff1.flag = BUFF_EMPTY ;
		sAdcBuf.buff1.length = RAW_ADC_LEN ;
		sAdcBuf.buff1.type = ADC_DATA ;
		sAdcBuf.buff1.ts =0 ;
		sAdcBuf.buff1.pData = (u8*)ADC_BASE_ADDR ;

		sAdcBuf.buff2 = sAdcBuf.buff1;
		sAdcBuf.buff2.pData = (u8*)(sAdcBuf.buff1.pData + RAW_ADC_LEN) ;
	}else if (RDMAP_DATA == type)
	{
		sRdmBuf.revLen = 0 ;
		sRdmBuf.buff1.flag = BUFF_EMPTY ;
		sRdmBuf.buff1.length = RDMAP_DATA_LEN ;
		sRdmBuf.buff1.type = RDMAP_DATA ;
		sRdmBuf.buff1.ts =0 ;
		sRdmBuf.buff1.pData = (u8*)RDM_BASE_ADDR ;

//		sRdmBuf.buff2 = sAdcBuf.buff1; buff2.type == 1 ??!
		memcpy(&sRdmBuf.buff2,&sRdmBuf.buff1,sizeof(sRdmBuf.buff1));
		sRdmBuf.buff2.pData = (u8*)(sRdmBuf.buff1.pData + RDMAP_DATA_LEN) ;
	}
}

buffer_t * data_path_getUsedBuf(s32 type)
{
	DataPingPong_t *pPing = NULL ;
	buffer_t *pBuf = NULL ;

	if (ADC_DATA == type)
	{
		pPing = &sAdcBuf ;
	}else if (RDMAP_DATA == type)
	{
		pPing = &sRdmBuf ;
	}

	if ( ( BUFF_EMPTY == pPing->buff2.flag) &&
			( BUFF_EMPTY == pPing->buff1.flag) )
	{
		pPing->usedId = 1 ;
	}

	pBuf = ( 1 == pPing->usedId ) ? (&pPing->buff1):(&pPing->buff2);

	return pBuf ;
}

buffer_t * data_path_getFullBuf(s32 type)
{
	DataPingPong_t *pPing = NULL ;
	buffer_t *pBuf = NULL ;

	if (ADC_DATA == type)
	{
		pPing = &sAdcBuf ;
	}else if (RDMAP_DATA == type)
	{
		pPing = &sRdmBuf ;
	}
	pBuf = ( 1 == pPing->usedId ) ? (&pPing->buff2):(&pPing->buff1);

	if (BUFF_FULL!=pBuf->flag)
		pBuf = NULL ;

	return pBuf ;
}

void ADCDataCallback(s32 type, s32 len)
{
	portBASE_TYPE  taskWoken = pdFALSE;
	BaseType_t ret = pdFALSE ;
	buffer_t *pBuf = (1 == sAdcBuf.usedId) ? (&sAdcBuf.buff1):(&sAdcBuf.buff2);

	data_trans_CacheInvalidate(pBuf->pData + sAdcBuf.revLen , len);

	sAdcBuf.revLen += len ;
	if (RAW_ADC_LEN == sAdcBuf.revLen)
	{
		sAdcBuf.usedId = ( 1 == sAdcBuf.usedId ) ? 2 : 1 ;
		sAdcBuf.revLen = 0 ;
		pBuf->flag = BUFF_FULL ;
	}
	if (BUFF_FULL == pBuf->flag )
	{
		sFrameID++ ;

		ret = xQueueSendFromISR(sQueueHandle , pBuf , &taskWoken) ;
//		if (pdTRUE != ret)
//			xil_printf("error!\n");
		if (pdTRUE == taskWoken)
		{
			portYIELD_FROM_ISR(taskWoken);
		}

		pBuf = (1 == sAdcBuf.usedId) ? (&sAdcBuf.buff1):(&sAdcBuf.buff2);
		if (BUFF_EMPTY != pBuf->flag)
		{
			pBuf->flag = BUFF_EMPTY ; //reset flag
		}

	}

	data_trans_start(ADC_DATA,pBuf->pData + sAdcBuf.revLen ,ADC_PACKET_LEN);
}

void RDMAPDataCallback(s32 type, s32 len)
{
	portBASE_TYPE  taskWoken = pdFALSE;
	BaseType_t ret = pdFALSE ;
	buffer_t *pBuf = (1 == sRdmBuf.usedId) ? (&sRdmBuf.buff1):(&sRdmBuf.buff2);
//	TickType_t pre =0 ,cur =0 ;
	sRdmBuf.revLen += len ;
	if (RDMAP_DATA_LEN == sRdmBuf.revLen)
	{
		sRdmBuf.usedId = ( 1 == sRdmBuf.usedId ) ? 2 : 1 ;
		sRdmBuf.revLen = 0 ;
		pBuf->flag = BUFF_FULL ;
	}
	if (BUFF_FULL == pBuf->flag )
	{
		sFrameID++ ;

		ret = xQueueSendFromISR(sQueueHandle , pBuf , &taskWoken) ;
		if (pdTRUE == taskWoken)
		{
			portYIELD_FROM_ISR(taskWoken);
		}

		pBuf = (1 == sRdmBuf.usedId) ? (&sRdmBuf.buff1):(&sRdmBuf.buff2);
		if (BUFF_EMPTY != pBuf->flag)
		{
			pBuf->flag = BUFF_EMPTY ; //reset flag
		}

	}
//	pre = xTaskGetTickCount();
	data_trans_start(RDMAP_DATA,pBuf->pData + sRdmBuf.revLen ,RDM_PACKET_LEN);
	pBuf = (1 == sRdmBuf.usedId) ? (&sRdmBuf.buff2):(&sRdmBuf.buff1);
	data_trans_CacheInvalidate(pBuf->pData + sAdcBuf.revLen , len);
//	cur = xTaskGetTickCount();

//	s32 cost = (cur - pre) / portTICK_RATE_MS ;
}

s32 data_path_init(void)
{
	s32 status = 0;

	data_path_buff_init(ADC_DATA);
	data_path_buff_init(RDMAP_DATA);

	status = data_trans_init(ADC_DATA);
	if(!status)
		status = data_trans_set_callback(ADC_DATA,ADCDataCallback);
	if(!status)
		status = data_trans_init(RDMAP_DATA);
	if(!status)
		status = data_trans_set_callback(RDMAP_DATA,RDMAPDataCallback);

	sQueueHandle = xQueueCreate( 10 , sizeof(buffer_t) );
	if (NULL==sQueueHandle)
	{
		status = -1 ;
	}

	xTaskCreate(data_path_task, "dataPathTask", 1024 * 2, NULL, 7, NULL);

	return status ;
}

s32 data_path_start(s32 type)
{
	s32 status = 0;
	buffer_t *pBuf =NULL ;

	if (ADC_DATA == type)
	{
		pBuf = data_path_getUsedBuf(ADC_DATA);
		if(pBuf)
			status = data_trans_start(ADC_DATA,pBuf->pData,ADC_PACKET_LEN);
	} else if (RDMAP_DATA == type)
	{
		pBuf = data_path_getUsedBuf(RDMAP_DATA);
		if(pBuf)
			status = data_trans_start(RDMAP_DATA,pBuf->pData,RDM_PACKET_LEN);
	}

	return status ;
}

s32 data_path_stop(s32 type)
{
	return 0 ;
}

void data_path_out_en(s32 type)
{
	sDataPathOutEn = type ;
}

s32 data_path_setEvCallBack(DataPathEvCb pCb)
{
	pDetectCb = pCb ;
}
