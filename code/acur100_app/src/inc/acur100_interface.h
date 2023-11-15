/*
 * acur100_interface.h
 *
 *  Created on: 2022年8月12日
 *      Author: A21001
 */

#ifndef APP_ACUR100_INTERFACE_TEST_ACUR100_INTERFACE_H_
#define APP_ACUR100_INTERFACE_TEST_ACUR100_INTERFACE_H_

#include "radar_common.h"

// 结构体1字节对齐
//#ifdef __GNUC__
//#define PACKED(__Declaration__) __Declaration__ __attribute__((packed))
//#else
//#define PACKED(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
//#endif

//缩放因子
#define ONE_TENTH 0.1f             //0.1
#define ONE_HUNDREDTH 0.01         //0.01
#define INV_ONE6_SCALE 0.015625f   // 1/2^6
#define INV_ONE7_SCALE 0.0078125f  // 1/2^7
#define INV_ONE8_SCALE 0.00390625f // 1/2^8
#define MUL_ONE6_SCALE 64          //2^6
#define MUL_ONE7_SCALE 128         //2^7
#define MUL_ONE8_SCALE 256         //2^8

#define MAX_TARGET_NUM 64

#define UDP_DATA_PACKET_HEAD_FLAG		0xF0F1F2F3U
#define INFO_HEAD_SYNC 					0xA5A5A5A5U

#define TERMINAL_BASIC_INFO_TYPE 		0x0000
#define ASUR100_DETECT_INFO_TYPE 		0x1000
#define ASUR100_TRACK_INFO_TYPE 		0x1001
#define ASUR100_BEAM_INFO_TYPE 			0x2000
#define ASUR100_CONFIG_PARAM_INFO_TYPE 	0x2001
#define ASUR100_PLATFORM_INFO_TYPE 		0x3000
#define ASUR100_STATUS_FAULT_INFO_TYPE 	0x3001
#define ASUR100_ADC_INFO_TYPE 			0x4000

//---------------------------------数据包头--------------------------------------------------------
typedef struct ST_ASUR100_PacketHeader
{
	/* 包标志：0xF0F1F2F3 */
   uint32_t packetFlag;
   uint16_t totalPacketNum;
   uint16_t packetLength;
   uint16_t sendInfoCount;
   uint8_t terminalType;	//0x00反无
   uint8_t subTerminalType;
   uint16_t infoType;
   uint16_t infoPacketNum;
   uint16_t curInfoPacketOrder;
   uint16_t reserved;
}ST_ASUR100_PacketHeader_t;

//-----------------------信息包头------------------------------------------------------------------
typedef struct ST_ASUR100_InfoHeader
{
	/* 信息同步头：0xa5a5a5a5 */
   uint32_t infoSync;
   uint32_t infoLength;
   uint32_t frameID;
   uint32_t timestamp;
   uint16_t infoType;
   uint16_t terminalID;
   uint8_t terminalType;
   uint8_t subTerminalType;
   uint8_t infoVersion;
   uint8_t reserved;
}ST_ASUR100_InfoHeader_t;

//-----------------------信息尾------------------------------------------------------------------
typedef struct ST_ASUR100_InfoTail
{
   uint16_t reserved;
   /* CRC-16 Xmodem，校验数据长度从信息同步头到数据结束 */
   uint16_t crc;
}ST_ASUR100_InfoTail_t;

/*------------------------基础信息----infoType:0x0000------------------------------*/
typedef struct ST_ASUR100_BasicInfo
{
	ST_ASUR100_InfoHeader_t stInfoHeader;

	char serialNum[64];
	char hardwareVersion[64];
	char bootVersion[32];
	char psSoftwareVersion[64];
	char plSoftwareVersion[64];
	char detectionVersion[64];
	char trackVersion[64];
	char reserved[64];

	ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_BasicInfo_t;

/* -----------------------检测 --- infoType:0x1000------------------------------------*/
typedef struct ASUR100_detectPointInfo
{
   uint16_t id;
   int16_t azimuth;         //unit:deg INV_ONE6_SCALE
   int32_t range;          //unit:m INV_ONE6_SCALE
   int16_t elevation;       //unit:deg INV_ONE6_SCALE
   int16_t velocity;        //unit:m/s INV_ONE6_SCALE
   int16_t dopplerChn;      //目标多普勒号
   uint16_t mag;            //INV_ONE6_SCALE
   uint16_t objConfidence;  //目标置信度 INV_ONE6_SCALE
   uint8_t ambiguous;       //0速度无模糊 1速度存在模糊
   uint8_t classification;  //0未识别 1无人机 2单兵 3车辆 4鸟类 5直升机 其它无效
   uint8_t classfyProb;     //目标类别概率 INV_ONE6_SCALE
   uint8_t cohesionOkFlag;  //凝聚完成标记，0：未完成；1：完成
   uint8_t cohesionPntNum;  //凝聚点数
   uint8_t cohesionBeamNum; //凝聚波位个数
   uint16_t aziBeamID;      //0~29 共30个，对应 -58°：4°：58°
   uint16_t eleBeamID;      //0~3 共4个，对应 -15°：10°：15°
   uint16_t reserve[4];     //备用
}ASUR100_detectPointInfo_t;

typedef struct ST_ASUR100_detectList
{
   ST_ASUR100_InfoHeader_t stInfoHeader;

   /* 检测目标个数，最大目标个数64 */
   uint16_t detectObjNum;
   /*单个检测目标长度，其长度由检测点信息定义决定 */
   uint16_t detectObjByte;
   /* 跟踪标识，0：TWS跟踪；1：TAS跟踪 */
   uint16_t trackTwsTasFlag;
   uint16_t reserved;

   ASUR100_detectPointInfo_t detectPoint[MAX_TARGET_NUM];

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_detectList_t;

/* -----------------------跟踪 --- infoType:0x1001------------------------------------*/
typedef struct ASUR100_trackPointInfo
{
   uint16_t id;
   int16_t azimuth;          //INV_ONE6_SCALE
   uint32_t range;           //INV_ONE6_SCALE
   int16_t elevation;        //INV_ONE6_SCALE
   int16_t velocity;         //目标径向速度m/s INV_ONE6_SCALE
   int16_t dopplerChn;       //目标多普勒号
   uint16_t mag;             //目标幅度值 INV_ONE6_SCALE
   uint8_t ambiguous;        //0速度无模糊 1速度存在模糊
   uint8_t classification;   //0未识别 1无人机 2单兵 3车辆 4鸟类 5直升机 其它无效
   uint8_t classifyProb;      //目标类别概率 INV_ONE6_SCALE
   uint8_t existingProb;     //目标存在概率 INV_ONE6_SCALE
   int16_t absVel;           //目标位移速度m/s INV_ONE6_SCALE
   int16_t orientationAngle; //目标航向,目标运动方向与真北夹角（°）INV_ONE6_SCALE
   uint16_t alive;           //目标周期数
   uint16_t twsTasFlag; //跟踪标识，0：TWS跟踪；1：TAS跟踪

   int32_t x;                //x相对坐标（m）INV_ONE6_SCALE
   int32_t y;                //y相对坐标（m）INV_ONE6_SCALE
   int32_t z;                //z相对坐标（m）INV_ONE6_SCALE

   int16_t vx;               //x方向相对速度（m/s）INV_ONE6_SCALE
   int16_t vy;               //y方向相对速度（m/s）INV_ONE6_SCALE
   int16_t vz;               //z方向相对速度（m/s）INV_ONE6_SCALE
   int16_t ax;               //x方向相对加速度（m/s²）INV_ONE6_SCALE
   int16_t ay;               //y方向相对加速度（m/s²）INV_ONE6_SCALE
   int16_t az;               //z方向相对加速度（m/s²）INV_ONE6_SCALE

   uint16_t x_variance;  //x相对坐标方差 INV_ONE6_SCALE
   uint16_t y_variance;  //y相对坐标方差 INV_ONE6_SCALE
   uint16_t z_variance;  //z相对坐标方差 INV_ONE6_SCALE
   uint16_t vx_variance; //x方向相对速度方差 INV_ONE6_SCALE
   uint16_t vy_variance; //y方向相对速度方差 INV_ONE6_SCALE
   uint16_t vz_variance; //z方向相对速度方差 INV_ONE6_SCALE
   uint16_t ax_variance; //x方向相对加速度方差 INV_ONE6_SCALE
   uint16_t ay_variance; //y方向相对加速度方差 INV_ONE6_SCALE
   uint16_t az_variance; //z方向相对加速度方差 INV_ONE6_SCALE

   uint8_t stateType;   //目标航迹类型，0：暂态航迹；1：稳态航迹
   uint8_t motionType;  //运动类型 0未知 1静止 2悬停 3靠近 4远离

   uint16_t forcastFrameNum; //目标预测帧数
   uint16_t associationNum;  //目标关联的检测点个数
   uint32_t assocBit0;      //关联检测点ID
   uint32_t assocBit1;      //关联检测点ID
   uint16_t reserve[4];     //备用
}ASUR100_trackPointInfo_t;

typedef struct ST_ASUR100_trackList
{
   ST_ASUR100_InfoHeader_t stInfoHeader;

   uint16_t trackObjNum;
   uint16_t trackTwsNum;
   uint16_t trackTasNum;
   uint16_t trackObjByte;
   uint16_t trackTwsTasFlag; //0 TWS跟踪  1 TAS跟踪
   uint16_t reserved;

   ASUR100_trackPointInfo_t trackPoint[MAX_TARGET_NUM];

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_trackList_t;

/* -----------------------波控信息 --- infoType:0x2000------------------------------------*/
typedef struct ST_ASUR100_beamInfo
{
   ST_ASUR100_InfoHeader_t stInfoHeader;

   uint16_t aziBeamID;
   uint16_t eleBeamID;
   int16_t aziBeamSin;
   int16_t eleBeamSin;
   uint16_t tasBeamTotal;
   uint16_t tasBeamCntCur;

   uint16_t tasObjId;
   uint16_t tasObjFilterNum;
   uint16_t tasObjRange;
   uint16_t samplePntStart;
   uint16_t samplePntDepth;
   uint16_t beamSwitchTime;
   uint32_t wholeSpaceScanCycleCnt;

   uint16_t trackTwsTasFlag;
   uint16_t reserve[3];

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_beamInfo_t;

/* -----------------------调试配置参数信息 --- infoType:0x2001------------------------------------*/
typedef struct ST_ASUR100_configParmInfo
{
   ST_ASUR100_InfoHeader_t stInfoHeader;

   uint8_t trSwitchCtrl;
   uint8_t workMode; //工作方式，0x00：搜索模式， 0x01：跟踪模式；
   uint8_t workWaveCode;
   uint8_t workFreqCode; //0~2
   uint8_t prfPeriod;   //PRF周期，0x00：1#PRF号；0x01：2#PRF号
   uint8_t accuNum;     //0表示64 1表示32

   uint16_t noiseCoef;   //INV_ONE7_SCALE
   uint16_t clutterCoef; //INV_ONE7_SCALE
   uint16_t cfarCoef;    //INV_ONE7_SCALE

   uint16_t focusRangeMin;
   uint16_t focusRangeMax;

   int16_t clutterCurveNum;
   int16_t lobeCompCoef; //INV_ONE7_SCALE

   uint8_t cohesionVelThre;
   uint8_t cohesionRgnThre;
   uint8_t clutterMapSwitch;
   uint8_t clutterMapUpdateCoef;

   int8_t aziCalcSlope;
   int8_t aziCalcPhase;
   int8_t eleCalcSlope;
   int8_t eleCalcPhase;

   int8_t aziScanCenter;
   uint8_t aziScanScope;

   int8_t eleScanCenter;
   uint8_t eleScanScope;

   uint16_t coherentDetectSwitch;

   uint16_t reserve[3];

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_configParmInfo_t;

/* -----------------------平台姿态信息 --- infoType:0x3000------------------------------------*/
typedef struct ST_ASUR100_platformInfo
{
	ST_ASUR100_InfoHeader_t stInfoHeader;

   int32_t heading; //0~360°
   int32_t pitching;
   int32_t rolling;
   int32_t longitude;
   int32_t latitude;
   int32_t altitude;
   int32_t velocityNavi;
   uint16_t targetTimeMark[6];
   uint32_t sigProcRelativeTime;
   uint16_t reserve[4];

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_platformInfo_t;

/* -----------------------故障信息 --- infoType:0x3001------------------------------------*/
typedef struct ST_ASUR100_statusFaultInfo
{
   ST_ASUR100_InfoHeader_t stInfoHeader;

   uint16_t isFailFlag;
   uint16_t failBitData1;
   uint16_t failBitData2;
   uint16_t batteryPower;

   uint16_t reserve[4];

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_statusFaultInfo_t;

/* -----------------------ADC信息 --- infoType:0x4000------------------------------------*/
typedef struct ST_ASUR100_ADCInfo
{
   ST_ASUR100_InfoHeader_t stInfoHeader;

   uint32_t length;
   uint32_t type;
   uint8_t* rawDataAddr;

   ST_ASUR100_InfoTail_t stInfoTail;
}ST_ASUR100_ADCInfo_t;

extern void SendDummyDataTask(void);

#endif /* APP_ACUR100_INTERFACE_TEST_ACUR100_INTERFACE_H_ */
