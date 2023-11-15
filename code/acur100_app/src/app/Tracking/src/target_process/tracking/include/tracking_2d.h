
#ifdef TRACKING_2D
#ifndef TRACKING_2D_H__
#define TRACKING_2D_H__

#include "../../../utilities/common_define.h"

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif


/**
 * @brief 
 *  TRACKING Measurement vector
 * @details
 *  The structure defines tracker measurement vector format
 */
typedef struct
{
    /**  @brief   Range, m */
	float range;
    /**  @brief   Angle, rad */
	float azimuthRad;
    /**  @brief   Radial velocity, m/s */
	float doppler;
} sMeasurement_Sph_vector;

/**
 * @brief 
 *  tracking position
 * @details
 *  The structure defines a position in cartesian space
 */
typedef struct
{
    /**  @brief   Target lateral position, m */
    float posX;
    /**  @brief   Targte horizontal position, m */
    float posY;
} sTracking_cartesian_position;

///**
// * @brief 
// *  Tracking State vector, 2DV configuration
// * @details
// *  The structure defines tracker state vector format when target position and velocity are tracked
// */
//typedef struct
//{
//    /**  @brief   Target lateral position, m */
//    float posX;
//    /**  @brief   Targte horizontal position, m */
//    float posY;
//    /**  @brief   Target lateral velocity, m/s */
//    float velX;
//    /**  @brief   Target horizontal velocity, m/s */
//    float velY;
//} sTracking_state_vector_pos_vel;

///**
// * @brief 
// *  Tracking State vector, 2DA configuration
// * @details
// *  The structure defines tracker state vector format when target position, velocity and acceleration are tracked
// */
//typedef struct
//{
//    /**  @brief   Target lateral position, m */
//    float posX;
//    /**  @brief   Targte horizontal position, m */
//    float posY;
//    /**  @brief   Target lateral velocity, m/s */
//    float velX;
//    /**  @brief   Target horizontal velocity, m/s */
//    float velY;
//    /**  @brief   Target lateral acceleration, m/s2 */
//    float accX;
//    /**  @brief   Target horizontal acceleration, m/s2 */
//    float accY;
//} sTracking_state_vector_pos_vel_acc;

typedef struct
{
    /**  @brief   Target lateral position, m */
    float p;
    /**  @brief   Target lateral velocity, m/s */
    float v;
    /**  @brief   Target lateral acceleration, m/s2 */
    float a;
} sTracking_vector_pos_vel_acc;

typedef struct
{
    /**  @brief   Target lateral position, m */
    float p;
    /**  @brief   Target lateral velocity, m/s */
    float v;
} sTracking_vector_pos_vel;

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif

#endif
#endif
