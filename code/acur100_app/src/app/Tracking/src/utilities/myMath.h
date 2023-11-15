
#ifndef MYMATH_H__
#define MYMATH_H__

#include <math.h>

#include "common_struct.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef TA_PROJECT
#define HEIGHT_SPLIT_SCALE (2.0f)
#else
#define HEIGHT_SPLIT_SCALE (10.0f)
#endif

#ifdef _WINDOWS
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <tchar.h>
#include <Windows.h>
#include <string.h>
#endif

#ifdef DEBUG_LOG
#define MAX_SIZE_LOG_STR (1024)
extern char _log_str[MAX_SIZE_LOG_STR];
extern int _log_ptr;
static int log_mutex;
#endif
//extern HANDLE my_mutex;
//extern int flag_mutex_init;
#ifdef ENABLE_OUTRESULT
extern FILE* out_fp;
extern char* g_outPath;
extern FILE* warningOut_fp;
extern char* g_warningOutPath;
extern FILE* logOut_fp;
extern char* g_logOutPath;
#endif

//extern const float cosTable[360];
//extern const float sinTable[360];
extern int dummy_frameCnt;

typedef struct {
	float x; // m
	float y; // m
	float z;
	float vx; // m/s
	float vy; // m/s
	float vz;
	float range; // m
	float doppler; // m/s
	float azimuth; // angle, deg
	float pitch;   // deg
	uint8_t uStatFlag; // 0: moving target, 1: stationary target
}sDummyRadarMeasUnit;

// Calculate Poisson Probability
float myMath_poissonProb(int lamda, int k);
// Calculate atan2f, atan(x/y)
float myMath_atan2f(float x, float y);
// convert (range, azimuth, doppler, elevation) -> (x,y,z,vx,vy,vz)
int myMath_spherical2cartesian(float *sph, float *cart);
// convert (x,y,z,vx,vy,vz) -> (range, azimuth, doppler, elevation)
int myMath_cartesian2spherical(float *cart, float *sph);
void myMath_sincosd(float theta, float * pSinVal, float * pCosVal);
// convert log2(SNR) to db
float convert_Log2SNR_to_db(int32_t inputActual, uint16_t fracBitIn);
// Descending order in float
int cmp_descending_float(const void *a, const void *b);
// Ascending order in float
int cmp_Ascending_float(const void *a, const void *b);
// Descending order in int
int cmp_descending_int(const void *a, const void *b);
// Ascending order in int
int cmp_Ascending_int(const void *a, const void *b);
// Check the value is zero, if yes, val = 0.0001
void myMath_checkZero(float *val);
// Nomalization
void myMath_nomalization(float *valArray, int k);
//lowPassFilter, x = k*input + (1-k)*x
void myMath_lowPassFilter(float *x, float input, float k);
// Set a value into [min_, max_]
void myMath_minMax(float *x, float min_, float max_);
// get max value
float myMath_getMax(float a, float b);
// get min value
float myMath_getMin(float a, float b);
// unroll a value between min_ and max_ 
void myMath_unrolling(float *x, float min_, float max_);
// Line interpolate a value in [min_, max_] based on the thresholds [thres_min, thres_max]
void myMath_lineInterpolation(float *x, float min_, float max_, float thres, float thres_min, float thres_max);
uint8_t myMath_isPointInsideBox(float *c, sBox *box);
// Small -> Big
void myMath_insertSort(sDetectPoint* allMeaDataBuff, int num);
// Small -> Big
void myMath_insertSort_float(float* allMeaDataBuff, int num);
void myMath_swap(float *a, float *b);
void myMath_swap_int16(int16_t *a, int16_t *b);
// Generate the dummy data for radar
int myMath_generateDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, float vx, float vy, float vz, uint8_t enable_stat);
void myMath_initDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, float vx, float vy, float vz);
void myMath_getNextFrameDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, float vx, float vy, float vz, uint8_t enable_stat);
int myMath_readDummyRadarData(sDummyRadarMeasUnit* dummyData, int* num, int frameID);
int myMath_generateDummyRadarData_tas(sDummyRadarMeasUnit* dummyData, int* num);

// from meter to pixel
int convert_x(double x);
int convert_y(double y);
// from pixel to meter
double inv_convert_x(int x);
double inv_convert_y(int y);

void my_setFrameNum(int frameID);

#ifdef DEBUG_LOG
void __cdecl my_printf(const char *strOutputString, ...);
#endif
#ifdef ENABLE_OUTRESULT
void my_setOutPath(char* outPath, char* logOutPath);
void my_closeOutFile();
void __cdecl my_out2File(const char *strOutputString, ...);
void __cdecl my_out2WarningFile(const char *strOutputString, ...);
#endif

#ifdef __cplusplus
}
#endif

#endif

