/*
 * Copyright (c) 2022, Autel.
 * All rights reserved.
 * @brief   CHC2442_assert
 * @file    chc2442_assert.h
 * @author  X22012
 * @date    2022年05月15日
 *
 * -History:
 *      -# author : X22012  
 *         date   : 2022年05月15日
 *         Version: V1.0
 *         details: Created
 */

#ifndef CHC2442_ASSERT_H_
#define CHC2442_ASSERT_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>

#if defined CHC2442_ASSERT

    #define FS_ASSERT(x) FsDevAssert(x)
#else
    /* Assert macro does nothing */
    #define FS_ASSERT(x) ((void)0)
#endif


#endif /* CHC2442_ASSERT_H_ */
