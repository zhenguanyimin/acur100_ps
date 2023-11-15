
#ifdef TRACKING_3D
#ifndef TRACKING_3D_H__
#define TRACKING_3D_H__

#include "../../../utilities/common_define.h"

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif


/**
 * @brief 
 *  TRACKING position
 * @details
 *  The structure defines a position in cartesian space
 */
typedef struct
{
    /**  @brief   Target lateral position, m */
    float posX;
    /**  @brief   Targte horizontal position, m */
    float posY;
    /**  @brief   Target vertical position, m */
    float posZ;
} sTracking_cartesian_position;

///**
// * @brief 
// *  TRACKING State vector, 3DV configuration
// * @details
// *  The structure defines tracker state vector format when target position and velocity are tracked
// */
//typedef struct
//{
//    /**  @brief   Target lateral position, m */
//    float posX;
//    /**  @brief   Targte horizontal position, m */
//    float posY;
//    /**  @brief   Target vertical position, m */
//    float posZ;
//    /**  @brief   Target lateral velocity, m/s */
//    float velX;
//    /**  @brief   Target horizontal velocity, m/s */
//    float velY;
//    /**  @brief   Target vertical velocity, m/s */
//    float velZ;
//} sTracking_vector_pos_vel;

/**
 * @brief 
 *  TRACKING State vector, 3DA configuration
 * @details
 *  The structure defines tracker state vector format when target position, velocity and acceleration are tracked
 */
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
