
#ifdef __cplusplus
extern "C" {
#endif


#include <math.h>
#include "xtime_l.h"
#include "xil_printf.h"
#include "detection_cfar_2d.h"
#include "detection_dummy.h"


int32_t gRangeCfarThreshold[6] = { CFAR_RANGE_THRESHOLD_SEG_1ST, CFAR_RANGE_THRESHOLD_SEG_2ND, CFAR_RANGE_THRESHOLD_SEG_3RD, CFAR_RANGE_THRESHOLD_SEG_4TH, CFAR_RANGE_THRESHOLD_SEG_5TH, CFAR_RANGE_THRESHOLD_SEG_6TH };
int32_t gDopplerCfarThreshold[6] = { CFAR_DOPPLER_THRESHOLD_SEG_1ST, CFAR_DOPPLER_THRESHOLD_SEG_2ND, CFAR_DOPPLER_THRESHOLD_SEG_3RD, CFAR_DOPPLER_THRESHOLD_SEG_4TH, CFAR_DOPPLER_THRESHOLD_SEG_5TH, CFAR_DOPPLER_THRESHOLD_SEG_6TH };
int32_t gGlobalCfarThreshold[6] = { CFAR_GLOBAL_THRESHOLD_SEG_1ST, CFAR_GLOBAL_THRESHOLD_SEG_2ND, CFAR_GLOBAL_THRESHOLD_SEG_3RD, CFAR_GLOBAL_THRESHOLD_SEG_4TH, CFAR_GLOBAL_THRESHOLD_SEG_5TH, CFAR_GLOBAL_THRESHOLD_SEG_6TH };
int32_t gRangeBinCutIdx[6] = { RANGE_CUT_INDEX_1ST, RANGE_CUT_INDEX_2ND, RANGE_CUT_INDEX_3RD, RANGE_CUT_INDEX_4TH, RANGE_CUT_INDEX_5TH, RANGE_CUT_INDEX_6TH };


int32_t gPeakNum = 0;
int32_t gRdmDummyFrameID = 0;


static sResolutionDetect_t gResolutionDetect =
{
	.RangeResotion = 			DEFAULT_RANGE_RESO,
	.VelocityResotion = 		DEFAULT_VELOCITY_RESO,
	.MagResotion = 				DEFAULT_MAG_RESO,
};

static sCfarParameter_t gCfarParameter =
{
	.rangeBins = 				DEFAULT_RANGE_BINS,
	.dopplerBins = 				DEFAULT_DOPPLER_BINS,
    .rangeWin = 				DEFAULT_RANGE_WIN,
    .rangeGuard = 				DEFAULT_RANGE_GUARD,
    .dopplerWin = 				DEFAULT_DOPPLER_WIN,
    .dopplerGuard = 			DEFAULT_DOPPLER_GUARD,
//    .rangeThresholdLen = 		CFAR_THRESHOLD_RANGE_LEN,
//    .dopplerThresholdLen = 		CFAR_THRESHOLD_RANGE_LEN,
    .osRange = 					DEFAULT_OS_RANGE,
    .osRangeK =					(OS_K * DEFAULT_RANGE_WIN * 2),
//    .pRangeThresholdStatic = 	gRangeCfarThresholdStatic,
//    .pDopplerThresholdStatic = 	gDopplerCfarThresholdStatic,
//    .pRangeThresholdMoving = 	gRangeCfarThresholdMoving,
//    .pDopplerThresholdMoving = 	gDopplerCfarThresholdMoving,
};


void GetRdmapData(uint32_t *rdmBuff)
{
//	memcpy(rdmBuff, &rdmDummyData_xingren_33frame[DUMMY_FRAME_NUM_1_SIZE * gRdmDummyFrameID], sizeof(uint32_t) * DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS);
//	memcpy(rdmBuff, &rdmDummyData_3target_10frameDUMMY_[FRAME_NUM_1_SIZE * gRdmDummyFrameID], sizeof(uint32_t) * DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS);
//	memcpy(rdmBuff, rdmDummyData_1target, sizeof(uint32_t) * DUMMY_DEFAULT_RANGE_BINS * DUMMY_DEFAULT_DOPPLER_BINS);
//	memcpy(rdmBuff, rdmDummyData_20target, sizeof(uint32_t) * DUMMY_DEFAULT_RANGE_BINS * DUMMY_DEFAULT_DOPPLER_BINS);

	gRdmDummyFrameID++;
	if (gRdmDummyFrameID == DUMMY_FRAME_NUM_REAL)
	{
		gRdmDummyFrameID = 0;
	}
}


ret_code_t CalcHistogram(uint32_t *rdmBuff, uint16_t *histBuff)
{
	int32_t i = 0, j = 0;
	uint32_t *rdmMagAux = NULL;
	uint16_t *histAux = NULL;

    if ((rdmBuff == NULL) || (histBuff == NULL))
    {
    	return RET_INVALID_PARAM;
    }

	memset(histBuff, 0, sizeof(uint16_t) * (DEFAULT_RANGE_BINS * DEFAULT_HIST_BINS));

	for (i = 0; i < DEFAULT_RANGE_BINS; i++)
	{
		rdmMagAux = &rdmBuff[(i << DEFAULT_DOPPLER_BINS_EXP)];	// rdmBuff[i * DEFAULT_DOPPLER_BINS]
		histAux = &histBuff[(i << DEFAULT_HIST_BINS_EXP)];	// histBuff[i * DEFAULT_HIST_BINS]
		for (j = 0; j < DEFAULT_DOPPLER_BINS; j++)
		{
			histAux[(rdmMagAux[j] >> RDM_MAG_PER_HIST_BIN_EXP)]++;	//normal process, input in 256*log2, 20220916
//			histAux[((uint32_t)(256 * log2f(rdmMagAux[j])) >> RDM_MAG_PER_HIST_BIN_EXP)]++;	//test process, input in 256*log2, 20220916
		}
	}

    return RET_OK;
}


ret_code_t CalcThreshold(uint16_t *histBuff, uint16_t *histMaxIndex, uint16_t *threshBuff, int16_t nrRangeBins)
{
    uint16_t i, j;
    uint16_t maxInd, thBin;
    uint16_t maxVal, thVal;
    uint16_t *cHist;

    if ((histBuff == NULL) || (threshBuff == NULL))
    {
        return RET_INVALID_PARAM;
    }

	/* Compute separate threshold for each range bin*/
	for (i = 0; i < nrRangeBins; i++)
	{
		maxVal = 0;
		maxInd = 0;
		cHist = &histBuff[(i << DEFAULT_HIST_BINS_EXP)];	// &histBuff[i * DEFAULT_HIST_BINS];

		/* Find index of first max value in histogram, ignoring bins corresponding to value 0*/
		for (j = 2; j < DEFAULT_HIST_BINS; j++)
		{
			if (cHist[j] > maxVal)
			{
				maxVal = cHist[j];
				maxInd = j;
			}
		}

		thBin = maxInd + HIST_THRESHOLD_GUARD; // AEB:8;	/* Use a guard*/
//		thVal = (threshBuff[i] >> 1) + (thBin << (RDM_MAG_PER_HIST_BIN_EXP - 1)); /* Turn to spt_log2 format and average with previous threshold*/
		thVal = (thBin << (RDM_MAG_PER_HIST_BIN_EXP)); /* Turn to spt_log2 format and average with previous threshold*/
		threshBuff[i] = thVal;
		histMaxIndex[i] = maxInd;
    }

    return RET_OK;
}


ret_code_t CalcTransposeRdm(uint32_t *rdmBuff, uint32_t *rdmTransposeBuff, int32_t rangeBins, int32_t dopplerBins)
{
	int32_t i = 0, j = 0;
	uint32_t *rdmTransposeBuffAux = NULL;
	uint32_t *rdmBuffAux = NULL;

    if ((rdmBuff == NULL) || (rdmTransposeBuff == NULL) || (rangeBins <= 0) || (dopplerBins <= 0))
    {
        return RET_INVALID_PARAM;
    }

    for (i = 0; i < rangeBins; i++)
    {
        rdmTransposeBuffAux = &rdmTransposeBuff[i];
		rdmBuffAux = &rdmBuff[(i << DEFAULT_DOPPLER_BINS_EXP)];	// rdmBuff + i * dopplerBins;
        for (j = 0; j < dopplerBins; j++)
        {
            rdmTransposeBuffAux[(j << DEFAULT_RANGE_BINS_EXP)] = rdmBuffAux[j];
        }
    }
    return RET_OK;
}


ret_code_t CalcPeakSearchBitmap(uint32_t *rdmBuff, uint32_t *rdmTransposeBuff, uint16_t *threshBuff, \
		uint32_t *detectBitmap, uint8_t *rdmapDooplerDimPeakFlag, int32_t rangeBins, int32_t dopplerBins)
{
	int32_t i = 0, j = 0;
	uint32_t rdmMag = 0;
	uint32_t rdmMagLeft = 0;
	uint32_t rdmMagRight = 0;
	uint32_t eachRangeBinStartIndex = 0;
	uint32_t eachDooplerBinStartIndex = 0;
	uint32_t eachPeakBitmapIndex = 0;
	uint32_t *rdmMagAux = NULL;
	uint32_t *rdmTransposeMagAux = NULL;
	uint8_t *rdmapDooplerDimPeakFlagAux = NULL;

    if ((rdmBuff == NULL) || (threshBuff == NULL) || (detectBitmap == NULL))
    {
    	return RET_INVALID_PARAM;
    }
    gPeakNum = 0;
	memset(detectBitmap, 0, sizeof(uint32_t) * PACKED_BITMAP_NUM_ALL_RDM);
	memset(rdmapDooplerDimPeakFlag, 0, sizeof(uint8_t) * DEFAULT_RANGE_BINS * DEFAULT_DOPPLER_BINS);

	// 1. doppler dimension peak search
	for (i = DETECT_RANGE_START_INDEX; i < DEFAULT_RANGE_BINS; i++)
	{
		eachRangeBinStartIndex = (i << DEFAULT_DOPPLER_BINS_EXP);
		rdmMagAux = &rdmBuff[eachRangeBinStartIndex];
		rdmapDooplerDimPeakFlagAux = &rdmapDooplerDimPeakFlag[eachRangeBinStartIndex];
		for (j = 0; j < DEFAULT_DOPPLER_BINS; j++)
		{
			rdmMag = rdmMagAux[j];
			if (rdmMag < threshBuff[i])
			{
				continue;
			}
			else
			{
				if (j == 0)
				{
					rdmMagLeft = rdmMagAux[DEFAULT_DOPPLER_BINS - 1];
					rdmMagRight = rdmMagAux[j + 1];
				}
				else if (j == (DEFAULT_DOPPLER_BINS - 1))
				{
					rdmMagLeft = rdmMagAux[j - 1];
					rdmMagRight = rdmMagAux[0];
				}
				else
				{
					rdmMagLeft = rdmMagAux[j - 1];
					rdmMagRight = rdmMagAux[j + 1];
				}

				if ((rdmMag > rdmMagLeft) && (rdmMag > rdmMagRight))
				{
					rdmapDooplerDimPeakFlagAux[j] = 1;
				}
			}
		}
	}

	// 2. range dimension peak search
#if ENABLE_PS_STATIC
	for (i = 0; i < DEFAULT_DOPPLER_BINS; i++)
#else
	for (i = 1; i < (DEFAULT_DOPPLER_BINS - 1); i++)// ignore static doppler bin
#endif
	{
		eachDooplerBinStartIndex = (i << DEFAULT_RANGE_BINS_EXP);
		rdmTransposeMagAux = &rdmTransposeBuff[eachDooplerBinStartIndex];
		for (j = DETECT_RANGE_START_INDEX; j < DEFAULT_RANGE_BINS; j++)
		{
			if (rdmapDooplerDimPeakFlag[(j << DEFAULT_DOPPLER_BINS_EXP) + i] == 0)
			{
				continue;
			}
			else
			{
				rdmMag = rdmTransposeBuff[eachDooplerBinStartIndex + j];
				if (j == 0)
				{
					rdmMagLeft = 0;
					rdmMagRight = rdmTransposeMagAux[j + 1];
				}
				else if (j == (DEFAULT_RANGE_BINS - 1))
				{
					rdmMagLeft = rdmTransposeMagAux[j - 1];
					rdmMagRight = 0;
				}
				else
				{
					rdmMagLeft = rdmTransposeMagAux[j - 1];
					rdmMagRight = rdmTransposeMagAux[j + 1];
				}

				if ((rdmMag > rdmMagLeft) && (rdmMag > rdmMagRight))
				{
					eachPeakBitmapIndex = (i << PACKED_BITMAP_NUM_PER_DOPPLER_EXP) + (j >> PACKED_BITMAP_ELEMENT_SIZE_EXP);
					detectBitmap[eachPeakBitmapIndex] |= (1 << (j % PACKED_BITMAP_ELEMENT_SIZE));
					gPeakNum++;
				}
			}
		}
	}

    return RET_OK;
}


int32_t Range_OSCFAR(int32_t idx, int32_t ROW, int32_t COL, int32_t CFAR_WINDOW, int32_t CFAR_GUARD, uint32_t *RangeData, int32_t osRangeK)
{
	int32_t i = 0, j = 0;
	int32_t index_RangeData = 0;
	int32_t value_min = 65536;
	int32_t value_max = 0;
	int32_t min_index = 0;
	int32_t max_index = 0;
	int32_t min_temp = 0;
	int32_t max_temp = 0;
	int32_t winsize = CFAR_WINDOW * 2;
	int32_t needSortsz[DEFAULT_RANGE_WIN * 2] = { 0 };
	int32_t Range_NoiseFloor_OS = 0;

/* 距离维前边缘保护 */
	if (idx < (CFAR_WINDOW + CFAR_GUARD))
	{
		if (idx < (CFAR_GUARD + 1))	/* 参考单元不足 */
		{
			for (i = 0; i < winsize; i++)
			{
				index_RangeData = (idx + CFAR_GUARD + i + 1) * COL;
				needSortsz[i] = RangeData[index_RangeData];
				if (needSortsz[i] > value_max)
				{
					value_max = needSortsz[i];
					max_index = i;
				}
				else if (needSortsz[i] < value_min)
				{
					value_min = needSortsz[i];
					min_index = i;
				}
				else
				{
					/* do nothing */
				}
			}
		}
		else
		{
			for (i = 0; i < winsize; i++)
			{
				if (i < (CFAR_GUARD + CFAR_WINDOW - idx))
				{
					index_RangeData = (idx + CFAR_GUARD + CFAR_WINDOW + i + 1) * COL;
					needSortsz[i] = RangeData[index_RangeData];
				}
				else if (i < CFAR_WINDOW)
				{
					index_RangeData = (idx - CFAR_GUARD - CFAR_WINDOW + i) * COL;
					needSortsz[i] = RangeData[index_RangeData];
				}
				else
				{
					index_RangeData = (idx + CFAR_GUARD - CFAR_WINDOW + i + 1) * COL;
					needSortsz[i] = RangeData[index_RangeData];
				}

				if (needSortsz[i] > value_max)
				{
					value_max = needSortsz[i];
					max_index = i;
				}
				else if (needSortsz[i] < value_min)
				{
					value_min = needSortsz[i];
					min_index = i;
				}
				else
				{
					/* do nothing */
				}
			}
		}
	}
/* 距离维后边缘保护 */
	else if ((idx < ROW) && (idx >= (ROW - CFAR_WINDOW - CFAR_GUARD)))
	{
		if (idx >= (ROW - CFAR_GUARD - 1))	/* 参考单元不足 */
		{
			for (i = 0; i < winsize; i++)
			{
				index_RangeData = (idx - CFAR_GUARD - winsize + i) * COL;
				needSortsz[i] = RangeData[index_RangeData];
				if (needSortsz[i] > value_max)
				{
					value_max = needSortsz[i];
					max_index = i;
				}
				else if (needSortsz[i] < value_min)
				{
					value_min = needSortsz[i];
					min_index = i;
				}
				else
				{
					/* do nothing */
				}
			}
		}
		else
		{
			for (i = 0; i < winsize; i++)
			{
				if (i < (ROW - idx - CFAR_GUARD - 1))
				{
					index_RangeData = (idx + CFAR_GUARD + i + 1) * COL;
					needSortsz[i] = RangeData[index_RangeData];
				}
				else
				{
					index_RangeData = (idx - CFAR_GUARD - winsize + i) * COL;
					needSortsz[i] = RangeData[index_RangeData];
				}

				if (needSortsz[i] > value_max)
				{
					value_max = needSortsz[i];
					max_index = i;
				}
				else if (needSortsz[i] < value_min)
				{
					value_min = needSortsz[i];
					min_index = i;
				}
				else
				{
					/* do nothing */
				}
			}
		}
	}
/* 非边缘距离点 */
	else
	{
		for (i = 0; i < CFAR_WINDOW; i++)
		{
			index_RangeData = (idx - CFAR_GUARD - CFAR_WINDOW + i) * COL;
			needSortsz[i] = RangeData[index_RangeData];
			index_RangeData = (idx + CFAR_GUARD + i + 1) * COL;
			needSortsz[i + CFAR_WINDOW] = RangeData[index_RangeData];
			if (needSortsz[i] <= needSortsz[i + CFAR_WINDOW])
			{
				if (needSortsz[i + CFAR_WINDOW] > value_max)
				{
					value_max = needSortsz[i + CFAR_WINDOW];
					max_index = i + CFAR_WINDOW;
				}
				if (needSortsz[i] < value_min)
				{
					value_min = needSortsz[i];
					min_index = i;
				}
			}
			else
			{
				if (needSortsz[i] > value_max)
				{
					value_max = needSortsz[i];
					max_index = i;
				}
				if (needSortsz[i + CFAR_WINDOW] < value_min)
				{
					value_min = needSortsz[i + CFAR_WINDOW];
					min_index = i + CFAR_WINDOW;
				}
			}
		}
	}

/* OS排序 部分排序 */
	max_temp = needSortsz[0];
	needSortsz[0] = needSortsz[max_index];
	needSortsz[max_index] = max_temp;
	if (winsize == 2)
	{
		Range_NoiseFloor_OS = needSortsz[winsize - 1];
	}
	else
	{
		if (min_index == 0)
		{
			min_temp = needSortsz[winsize - 1];
			needSortsz[winsize - 1] = needSortsz[max_index];
			needSortsz[max_index] = min_temp;
		}
		else
		{
			min_temp = needSortsz[winsize - 1];
			needSortsz[winsize - 1] = needSortsz[min_index];
			needSortsz[min_index] = min_temp;
		}

		for (i = 1; i < (winsize - osRangeK + 1); i++)
		{
			for (j = (i + 1); j < (winsize - 1); j++)
			{
				if (needSortsz[i] < needSortsz[j])
				{
					min_temp = needSortsz[i];
					needSortsz[i] = needSortsz[j];
					needSortsz[j] = min_temp;
				}
			}
		}
		Range_NoiseFloor_OS = needSortsz[winsize - osRangeK];
	}

	return Range_NoiseFloor_OS;
}


int32_t Range_CACFAR(int32_t idx, int32_t ROW, int32_t COL, int32_t CFAR_WINDOW, int32_t CFAR_GUARD, uint32_t *RangeData)
{
	int32_t i = 0;
	int32_t index_RangeData = 0;
	int32_t data_sum = 0;
	int32_t i_temp = 1;
	int32_t winsize = CFAR_WINDOW * 2;
	int32_t Range_NoiseFloor_CA = 0;

/* 距离维前边缘保护 */
	if (idx < (CFAR_WINDOW + CFAR_GUARD))
	{
		if (idx < (CFAR_GUARD + 1))	/* 参考单元不足 */
		{
			for (i = 0; i < winsize; i++)
			{
				index_RangeData = (idx + CFAR_GUARD + i + 1) * COL;
				if (RangeData[index_RangeData] != 0)
				{
					data_sum += RangeData[index_RangeData];
					i_temp += 1;
				}
			}
		}
		else
		{
			for (i = 0; i < winsize; i++)
			{
				if (i < (CFAR_GUARD + CFAR_WINDOW - idx))
				{
					index_RangeData = (idx + CFAR_GUARD + CFAR_WINDOW + i + 1) * COL;
					if (RangeData[index_RangeData] != 0)
					{
						data_sum += RangeData[index_RangeData];
						i_temp += 1;
					}
				}
				else if (i < CFAR_WINDOW)
				{
					index_RangeData = (idx - CFAR_GUARD- CFAR_WINDOW + i) * COL;
					if (RangeData[index_RangeData] != 0)
					{
						data_sum += RangeData[index_RangeData];
						i_temp += 1;
					}
				}
				else
				{
					index_RangeData = (idx + CFAR_GUARD - CFAR_WINDOW + i + 1) * COL;
					if (RangeData[index_RangeData] != 0)
					{
						data_sum += RangeData[index_RangeData];
						i_temp += 1;
					}
				}
			}
		}
	}
/* 距离维后边缘保护 */
	else if ((idx < ROW) && (idx >= (ROW - CFAR_WINDOW - CFAR_GUARD)))
	{
		if (idx >= (ROW - CFAR_GUARD - 1))	/* 参考单元不足 */
		{
			for (i = 0; i < winsize; i++)
			{
				index_RangeData = (idx - CFAR_GUARD - winsize + i) * COL;
				if (RangeData[index_RangeData] != 0)
				{
					data_sum += RangeData[index_RangeData];
					i_temp += 1;
				}
			}
		}
		else
		{
			for (i = 0; i < winsize; i++)
			{
				if (i < (ROW - idx - CFAR_GUARD - 1))
				{
					index_RangeData = (idx + CFAR_GUARD + i + 1) * COL;
					if (RangeData[index_RangeData] != 0)
					{
						data_sum += RangeData[index_RangeData];
						i_temp += 1;
					}
				}
				else
				{
					index_RangeData = (idx - CFAR_GUARD - winsize + i) * COL;
					if (RangeData[index_RangeData] != 0)
					{
						data_sum += RangeData[index_RangeData];
						i_temp += 1;
					}
				}
			}
		}
	}
/* 非边缘距离点 */
	else
	{
		for (i = 0; i < CFAR_WINDOW; i++)
		{
			index_RangeData = (idx - CFAR_GUARD - CFAR_WINDOW + i) * COL;
			if (RangeData[index_RangeData] != 0)
			{
				data_sum += RangeData[index_RangeData];
				i_temp += 1;
			}
			index_RangeData = (idx + CFAR_WINDOW + i + 1) * COL;
			if (RangeData[index_RangeData] != 0)
			{
				data_sum += RangeData[index_RangeData];
				i_temp += 1;
			}
		}
	}

	Range_NoiseFloor_CA = (int32_t)(data_sum / (i_temp - 1));
	return Range_NoiseFloor_CA;
}


int32_t Doppler_CACFAR(int32_t idx, int32_t ROW, int32_t COL, int32_t CFAR_WINDOW, int32_t CFAR_GUARD, uint32_t *DopplerData, bool RemoveMax_Flag)
{
	int32_t i = 0;
	int32_t index_left = 0;
	int32_t index_right = 0;
	int32_t data_sum = 0;
	int32_t i_temp = 1;
	int32_t value_max = 0;
	int32_t Doppler_NoiseFloor_CA = 0;

	for (i = 0; i < CFAR_WINDOW; i++)
	{
		index_left = idx - CFAR_GUARD - CFAR_WINDOW + i;
		index_right = idx + CFAR_GUARD + i + 1;
		if (index_left < 0)
		{
			index_left += COL;
		}
		if (index_right > (COL - 1))
		{
			index_right -= COL;
		}

		if (DopplerData[index_left] != 0)
		{
			data_sum += DopplerData[index_left];
			i_temp += 1;
		}
		if (DopplerData[index_right] != 0)
		{
			data_sum += DopplerData[index_right];
			i_temp += 1;
		}

		if (DopplerData[index_left] >= DopplerData[index_right])
		{
			if (DopplerData[index_left] > value_max)
			{
				value_max = DopplerData[index_left];
			}
		}
		else
		{
			if (DopplerData[index_right] > value_max)
			{
				value_max = DopplerData[index_right];
			}
		}
	}

	if (RemoveMax_Flag)
	{
		Doppler_NoiseFloor_CA = (data_sum - value_max) / (i_temp - 2);
	}
	else
	{
		Doppler_NoiseFloor_CA = (data_sum) / (i_temp - 1);
	}

	return Doppler_NoiseFloor_CA;
}


//ret_code_t CfarDetection(uint32_t *cfarInput, uint32_t *detectBitmapInput, protocol_object_list_detected_t *detectList)
ret_code_t CfarDetection(uint32_t *cfarInput, uint32_t *detectBitmapInput, uint16_t *threshBuff, protocol_object_list_detected_t *detectList)

{
	uint16_t usCfarTargetNo = 0;
    uint32_t rdmSize = gCfarParameter.rangeBins * gCfarParameter.dopplerBins;
    uint32_t inputDetectBitmapSize = rdmSize / PACKED_BITMAP_ELEMENT_SIZE;// The status of every 32 bins are saved by a uint32_t variable
    uint32_t rdmapMag = 0;
    int32_t uiPeakIndexinRdm = 0;
    int32_t i_range = 0, j_doppler = 0;
	int32_t Range_NoiseFloor = 0;
	int32_t Doppler_NoiseFloor_CA = 0;
	int32_t RangeCfarThreshold = 0;
	int32_t DopplerCfarThreshold = 0;
	int32_t GlobalCfarThreshold = 0;
    int32_t peakIndex = 0;
	uint32_t aux = 0;
    int32_t i_BitmapNum = 0;
    int32_t i_index = 0, j_index = 0;
    int32_t startIndex_Doppler = 0;
    int32_t dopplerShift = 0;

    if ((cfarInput == NULL) || (detectBitmapInput == NULL) || (detectList == NULL))
    {
        return RET_INVALID_PARAM;
    }

	for (i_BitmapNum = 0; i_BitmapNum < inputDetectBitmapSize; i_BitmapNum++)
	{
		i_index = i_BitmapNum;
		j_index = 0;
		aux = detectBitmapInput[i_index];
		// Decode input peaks
		while (aux)
		{
			// Check if current index is a detected peak
			if (aux & 0x1)
			{
				peakIndex = (i_index << PACKED_BITMAP_ELEMENT_SIZE_EXP) + j_index;
				j_doppler = (peakIndex >> DEFAULT_RANGE_BINS_EXP);
				startIndex_Doppler = (j_doppler << DEFAULT_RANGE_BINS_EXP);
				i_range = peakIndex - startIndex_Doppler;
				uiPeakIndexinRdm = (i_range << DEFAULT_DOPPLER_BINS_EXP) + j_doppler;
				rdmapMag = cfarInput[uiPeakIndexinRdm];


//				if ((i_range > 0) && (i_range < RANGE_CUT_INDEX_FRONT))	/* 前2-RANGE_CUT_INDEX_FRONTbin CFAR单独考虑 */
//				{
//					RangeCfarThreshold = gRangeCfarThreshold[0];
//					DopplerCfarThreshold = gDopplerCfarThreshold[0];
//					Range_NoiseFloor = Range_OSCFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler), gCfarParameter.osRangeK);
//				}
//				else if ((i_range < gCfarParameter.rangeBins) && (i_range > (RANGE_CUT_INDEX_REAR - 1)))	/* RANGE_CUT_INDEX_REARbin后 CFAR单独考虑 */
//				{
//					RangeCfarThreshold = gRangeCfarThreshold[2];
//					DopplerCfarThreshold = gDopplerCfarThreshold[2];
//					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
//				}
//				else
//				{
//					RangeCfarThreshold = gRangeCfarThreshold[1];
//					DopplerCfarThreshold = gDopplerCfarThreshold[1];
//					if ((i_range > (RANGE_CUT_INDEX_FRONT - 1)) && (i_range < gCfarParameter.osRange))
//					{
//						Range_NoiseFloor = Range_OSCFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//								gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler), gCfarParameter.osRangeK);
//					}
//					else
//					{
//						Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//								gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
//					}
//				}


				if ((i_range >= gRangeBinCutIdx[0]) && (i_range <= gRangeBinCutIdx[1]))	/* [2,10]bin,前2-RANGE_CUT_INDEX_FRONTbin CFAR单独考虑 */
				{
					RangeCfarThreshold = gRangeCfarThreshold[0];
					DopplerCfarThreshold = gDopplerCfarThreshold[0];
					GlobalCfarThreshold = gGlobalCfarThreshold[0];
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
//					Range_NoiseFloor = Range_OSCFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler), gCfarParameter.osRangeK);
				}
				else if ((i_range > gRangeBinCutIdx[5]) && (i_range < gCfarParameter.rangeBins))	/* (430,end]bin,RANGE_CUT_INDEX_REARbin后 CFAR单独考虑 */
				{
					RangeCfarThreshold = gRangeCfarThreshold[5];
					DopplerCfarThreshold = gDopplerCfarThreshold[5];
					GlobalCfarThreshold = gGlobalCfarThreshold[5];
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
				}
				else if ((i_range > gRangeBinCutIdx[1]) && (i_range <= gRangeBinCutIdx[2]))	/* (10,32]bin,RANGE_CUT_INDEX_REARbin后 CFAR单独考虑 */
				{
					RangeCfarThreshold = gRangeCfarThreshold[1];
					DopplerCfarThreshold = gDopplerCfarThreshold[1];
					GlobalCfarThreshold = gGlobalCfarThreshold[1];
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
				}
				else if ((i_range > gRangeBinCutIdx[2]) && (i_range <= gRangeBinCutIdx[3]))	/* (32,54]bin,RANGE_CUT_INDEX_REARbin后 CFAR单独考虑 */
				{
					RangeCfarThreshold = gRangeCfarThreshold[2];
					DopplerCfarThreshold = gDopplerCfarThreshold[2];
					GlobalCfarThreshold = gGlobalCfarThreshold[2];
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
				}
				else if ((i_range > gRangeBinCutIdx[3]) && (i_range <= gRangeBinCutIdx[4])) /* (54,72]bin */
				{
					RangeCfarThreshold = gRangeCfarThreshold[3];
					DopplerCfarThreshold = gDopplerCfarThreshold[3];
					GlobalCfarThreshold = gGlobalCfarThreshold[3];
//						Range_NoiseFloor = Range_OSCFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//								gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler), gCfarParameter.osRangeK);
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));

				}
				else if ((i_range > gRangeBinCutIdx[4]) && (i_range <= gRangeBinCutIdx[5]))  /* [73,430]bin */
				{
					RangeCfarThreshold = gRangeCfarThreshold[4];
					DopplerCfarThreshold = gDopplerCfarThreshold[4];
					GlobalCfarThreshold = gGlobalCfarThreshold[4];
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
				}
				else
				{
					RangeCfarThreshold = gRangeCfarThreshold[3];
					DopplerCfarThreshold = gDopplerCfarThreshold[3];
					GlobalCfarThreshold = gGlobalCfarThreshold[3];
					Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
							gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
				}
//				else
//				{
//					RangeCfarThreshold = gRangeCfarThreshold[3];
//					DopplerCfarThreshold = gDopplerCfarThreshold[3];
//					if ((i_range > gRangeBinCutIdx[3]) && (i_range <= gRangeBinCutIdx[4])) /* (54,150]bin */
//					{
////						Range_NoiseFloor = Range_OSCFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
////								gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler), gCfarParameter.osRangeK);
//						Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//								gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
//
//					}
//					else //if ((i_range > gRangeBinCutIdx[4]) && (i_range <= gRangeBinCutIdx[5]))  /* []1,151,430]bin */
//					{
//						Range_NoiseFloor = Range_CACFAR(i_range, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
//								gCfarParameter.rangeWin, gCfarParameter.rangeGuard, (cfarInput + j_doppler));
//					}
//				}
				
				Doppler_NoiseFloor_CA = Doppler_CACFAR(j_doppler, gCfarParameter.rangeBins, gCfarParameter.dopplerBins,
						gCfarParameter.dopplerWin, gCfarParameter.dopplerGuard, (cfarInput + i_range * gCfarParameter.dopplerBins), 1);

				if (j_doppler >= (gCfarParameter.dopplerBins / 2))
				{
					dopplerShift = j_doppler - gCfarParameter.dopplerBins;
				}
				else
				{
					dopplerShift = j_doppler;
				}

				if (rdmapMag > (uint32_t)(RangeCfarThreshold + Range_NoiseFloor)
					&& (rdmapMag > (uint32_t)(DopplerCfarThreshold + Doppler_NoiseFloor_CA))
					&& (rdmapMag > (uint32_t)(GlobalCfarThreshold + (threshBuff[i_range] - ((HIST_THRESHOLD_GUARD-1)<<RDM_MAG_PER_HIST_BIN_EXP))))) //add global SNR Threshold 
				{
					detectList->detectPoint[usCfarTargetNo].id = usCfarTargetNo;
					detectList->detectPoint[usCfarTargetNo].range = (uint32_t)(i_range * gResolutionDetect.RangeResotion * MUL_ONE6_SCALE);	//i_range;
					detectList->detectPoint[usCfarTargetNo].velocity = (int16_t)(dopplerShift * gResolutionDetect.VelocityResotion * MUL_ONE6_SCALE);
					detectList->detectPoint[usCfarTargetNo].dopplerChn = (int16_t)j_doppler;
					detectList->detectPoint[usCfarTargetNo].mag = (uint16_t)(rdmapMag * gResolutionDetect.MagResotion * MUL_ONE6_SCALE); //rdmapMag;
					detectList->detectPoint[usCfarTargetNo].azimuth = (int16_t)(asinf(1.0f * gBeamInfo[0].aziBeamSin / MUL_ONE15_SCALE) * RAD2DEG * MUL_ONE6_SCALE);
					detectList->detectPoint[usCfarTargetNo].elevation = (int16_t)(asinf(1.0f * gBeamInfo[0].eleBeamSin / MUL_ONE15_SCALE) * RAD2DEG * MUL_ONE6_SCALE);
					detectList->detectPoint[usCfarTargetNo].classification = (uint8_t)((rdmapMag - Range_NoiseFloor) * gResolutionDetect.MagResotion); //rangeSNR;
					detectList->detectPoint[usCfarTargetNo].cohesionOkFlag = (uint8_t)((rdmapMag - Doppler_NoiseFloor_CA) * gResolutionDetect.MagResotion); //dopplerSNR;
					detectList->detectPoint[usCfarTargetNo].cohesionPntNum = (uint8_t)((rdmapMag - (threshBuff[i_range] - ((HIST_THRESHOLD_GUARD-1)<<RDM_MAG_PER_HIST_BIN_EXP))) * gResolutionDetect.MagResotion); //globalSNR;
					usCfarTargetNo ++;
				}

				if (usCfarTargetNo >= MAX_OUTPUT_CFAR_TARGET)
				{
					usCfarTargetNo = MAX_OUTPUT_CFAR_TARGET;
					break;
				}
			}
			// Go to the next bit in current element
			j_index += 1;
			aux >>= 1;
		}

		if (usCfarTargetNo >= MAX_OUTPUT_CFAR_TARGET)
		{
			usCfarTargetNo = MAX_OUTPUT_CFAR_TARGET;
			break;
		}
	}

	detectList->detectObjNum = usCfarTargetNo;

	return RET_OK;
}


ret_code_t DetectionAlgProcess(sDetectObjData_t *detectObjData)
{
	XTime tCur1, tCur2, tCur3, tCur4, tCur5, tCur6, tCurAll;
	XTime tEnd1, tEnd2, tEnd3, tEnd4, tEnd5, tEnd6, tEndAll;
	uint32_t tUsed1, tUsed2, tUsed3, tUsed4, tUsed5, tUsed6, tUsedAll;
	ret_code_t status = RET_OK;

	XTime_GetTime(&tCurAll);

	XTime_GetTime(&tCur1);
	GetRdmapData(detectObjData->rdmapData);
	XTime_GetTime(&tEnd1);
	tUsed1 = ((tEnd1 - tCur1) * 1000000) / (COUNTS_PER_SECOND);
//	xil_printf("1. GetRdmapData time elapsed is %d us\r\n", tUsed1);

	XTime_GetTime(&tCur2);
	if (status == RET_OK)
	{
		status = CalcHistogram(detectObjData->rdmapData, detectObjData->histBuff);
	}
	XTime_GetTime(&tEnd2);
	tUsed2 = ((tEnd2 - tCur2) * 1000000) / (COUNTS_PER_SECOND);
//	xil_printf("2. CalcHistogram time elapsed is %d us\r\n", tUsed2);

	XTime_GetTime(&tCur3);
	if (status == RET_OK)
	{
		status = CalcThreshold(detectObjData->histBuff, detectObjData->hististMaxIndex, detectObjData->threshold, gCfarParameter.rangeBins);
	}
	XTime_GetTime(&tEnd3);
	tUsed3 = ((tEnd3 - tCur3) * 1000000) / (COUNTS_PER_SECOND);
//	xil_printf("3. CalcThreshold time elapsed is %d us\r\n", tUsed3);

	XTime_GetTime(&tCur4);
	if (status == RET_OK)
	{
		status = CalcTransposeRdm(detectObjData->rdmapData, detectObjData->rdmapTransposeData, gCfarParameter.rangeBins, gCfarParameter.dopplerBins);
	}
	XTime_GetTime(&tEnd4);
	tUsed4 = ((tEnd4 - tCur4) * 1000000) / (COUNTS_PER_SECOND);
//	xil_printf("4. CalcTransposeRdm time elapsed is %d us\r\n", tUsed4);

	XTime_GetTime(&tCur5);
	if (status == RET_OK)
	{
		status = CalcPeakSearchBitmap(detectObjData->rdmapData, detectObjData->rdmapTransposeData, detectObjData->threshold, detectObjData->peakBitmap, \
				detectObjData->rdmapDooplerDimPeakFlag, gCfarParameter.rangeBins, gCfarParameter.dopplerBins);
	}
	XTime_GetTime(&tEnd5);
	tUsed5 = ((tEnd5 - tCur5) * 1000000) / (COUNTS_PER_SECOND);
//	xil_printf("5. CalcPeakSearchBitmap time elapsed is %d us\r\n", tUsed5);

	XTime_GetTime(&tCur6);
	if (status == RET_OK)
	{
		status = CfarDetection(detectObjData->rdmapData, detectObjData->peakBitmap, detectObjData->threshold, detectObjData->detectList);
	//	status = CfarDetection(detectObjData->rdmapData, detectObjData->peakBitmap, detectObjData->detectList);

	}
	XTime_GetTime(&tEnd6);
	tUsed6 = ((tEnd6 - tCur6) * 1000000) / (COUNTS_PER_SECOND);
//	xil_printf("6. CfarDetection time elapsed is %d us\r\n", tUsed6);

	XTime_GetTime(&tEndAll);
	tUsedAll = ((tEndAll - tCurAll) * 1000000) / (COUNTS_PER_SECOND);
	tUsedAll = tUsed1 + tUsed2 + tUsed3 + tUsed4 + tUsed5 + tUsed6;
	xil_printf("frameID is %d \r\n", detectObjData->detectList->stInfoHeader.frameID);
	xil_printf("DetectionAlgProcess time elapsed is %d us\r\n", tUsedAll);

	return status;
}


/*================================================================================================*/
#ifdef __cplusplus
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
