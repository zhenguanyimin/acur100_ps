
#ifndef TRACKING_LIST_LIB_H__
#define TRACKING_LIST_LIB_H__

#include "../../../utilities/common_define.h"

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
extern "C" {
#endif
#endif


/**
 * @brief 
 *  TRACKING ListElement
 *
 * @details
 *  The structure describes a list node which has links to the previous
 *  and next element in the list.
 */
typedef struct sTracking_ListElem_t
{
	uint32_t					data;
	struct sTracking_ListElem_t	*prev;
	struct sTracking_ListElem_t	*next;
} sTracking_ListElem;

/**
 * @brief 
 *  Tracking List Object
 *
 * @details
 *  The structure describes the list object 
 */
typedef struct
{
    uint32_t			count;
    sTracking_ListElem		*begin;
    sTracking_ListElem		*end;
} sTracking_ListObj;

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern void tracking_listInit (sTracking_ListObj *list);
extern int32_t tracking_isListEmpty (sTracking_ListObj *list);
extern void tracking_listEnqueue (sTracking_ListObj *list, sTracking_ListElem *elem);
extern sTracking_ListElem *tracking_listDequeue (sTracking_ListObj *list);
extern sTracking_ListElem* tracking_listGetFirst (sTracking_ListObj *list);
extern sTracking_ListElem* tracking_listGetNext (sTracking_ListElem *elem);
extern uint32_t tracking_listGetCount (sTracking_ListObj *list);
extern int32_t tracking_listRemoveElement (sTracking_ListObj *list, sTracking_ListElem *elem);
extern int tracking_listelemGetLength (sTracking_ListElem* list);
extern sTracking_ListElem* tracking_listGetEnd (sTracking_ListObj* list);
extern void tracking_listMerge (sTracking_ListObj *list_dst, sTracking_ListObj *list_src);
extern void tracking_listSplit(sTracking_ListObj *list_src, int split_num, sTracking_ListObj *list_ahead_part);

#ifndef ENABLE_ADTF_LOG
#ifdef __cplusplus
}
#endif
#endif

#endif /* TRACKING_LIST_LIB_H__ */

