
#include <stdint.h>

#include "../include/tracking_listlib.h"

/**************************************************************************
 ***************************** ListLib Functions **************************
 **************************************************************************/

/** 
 *  @b Description
 *  @n  
 *      The function is called to initialize the list.
 *
 *  @param[in]  list
 *      List to be initialized
 *
 *  @retval
 *      Not Applicable
 */
void tracking_listInit (sTracking_ListObj* list)
{
    /* Initialize the fields in the list object. */
    list->count = 0;
    list->begin = 0;
    list->end   = 0;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to check if the list is empty or not
 *
 *  @param[in]  list
 *      List to be checked.
 *
 *  @retval
 *      1	-	List is empty
 *  @retval
 *      0	-	List
 */
int32_t tracking_isListEmpty (sTracking_ListObj* list)
{
    if (list->count == 0)
        return 1;
    return 0;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to add an element to the end of the list.
 *
 *  @param[in]  list
 *      List from where the element is to be added
 *  @param[in]  elem
 *      Element to be added to the end of the list
 *
 *  @retval
 *      Not Applicable.
 */
void tracking_listEnqueue (sTracking_ListObj *list, sTracking_ListElem *elem)
{
	if(list->begin == 0) 
	{
		elem->prev = 0;
		elem->next = 0;

		list->begin = elem;
		list->end = elem;
	}
	else
	{
		elem->prev = list->end;
		elem->next = 0;

		list->end->next = elem;
		list->end = elem;
	}
    list->count++;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to dequeue an element from the head of the list.
 *
 *  @param[in]  list
 *      List from where the element is to be removed
 *
 *  @retval
 *      Head of the list (NULL if the list was empty)
 */
sTracking_ListElem* tracking_listDequeue (sTracking_ListObj* list)
{
	sTracking_ListElem *elem;

	if(list->begin == 0)
		return 0;
	
	elem = list->begin;
	list->begin = elem->next;
	list->count--;

	if(list->begin == 0)
		list->end = 0;

	elem->next = 0;
	elem->prev = 0;

	return elem;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to get the number of elements in the list.
 *
 *  @param[in]  list
 *      List for which the number of elements are required.
 *
 *  @retval
 *      Counter
 */
uint32_t tracking_listGetCount (sTracking_ListObj* list)
{
    return list->count;
}
/** 
 *  @b Description
 *  @n  
 *      The function is called to get the first elements in the list.
 *
 *  @param[in]  list
 *      List for which the number of elements are required.
 *
 *  @retval
 *      First Element
 */
sTracking_ListElem* tracking_listGetFirst (sTracking_ListObj* list)
{
	return list->begin;
}
/** 
 *  @b Description
 *  @n  
 *      The function is called to get the next elements in the list.
 *
 *  @param[in]  elem
 *      List for which the number of elements are required.
 *
 *  @retval
 *      Next Element
 */
sTracking_ListElem* tracking_listGetNext (sTracking_ListElem* elem)
{
	return elem->next;
}

/**
 *  @b Description
 *  @n  
 *      The function is called to remove the specific element from 
 *      the list.
 *
 *  @param[in]  list
 *      List from which the element is to be removed.
 *  @param[in]  elem
 *      The element to be removed.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t tracking_listRemoveElement (sTracking_ListObj* list, sTracking_ListElem* elem)
{
	if(elem->prev == 0)
	{
		if(list->begin != elem)
			return -1;

		if(elem->next == 0)
		{
			/* Next could be NULL only if that is the last element */
			if(list->end != elem)
				return -1;

			if(list->count != 1)
				return -1;

			/* That was the only element, list is emty now */
			list->begin = 0;
			list->end = 0;
			list->count = 0;
			return 0;
		}

		if(elem->next->prev != elem)
			return -1;

		/* That was the first element, update the list begin */
		list->begin = elem->next;
		elem->next->prev = 0;
		list->count--;
		return 0;
	}

	if(elem->prev->next != elem)
			return -1;

	if(elem->next == 0)
	{
		/* Next could be NULL only if that is the last element */
		if(list->end != elem)
			return -1;

		/* Yes, it is valid last element */
		elem->prev->next = 0;
		list->end = elem->prev;
		list->count--;
		return  0;
	}
	
	if(elem->next->prev != elem)
		return -1;

	elem->prev->next = elem->next;
	elem->next->prev = elem->prev;
	list->count--;
	return  0;
}


/** 
 *  @b Description
 *  @n  
 *      The function is called to get the length of elem list.
 *
 *  @param[in]  list
 *      List
 *
 *  @retval
 *      the length of list elem
 */
int tracking_listelemGetLength (sTracking_ListElem* list)
{
	sTracking_ListElem* q=list;
	int length=0;

	if (0==q)
	{
		return length;
	}
	else if (0==q->next)
	{
		length=1;
	}

	while (0!=q->next)
	{
		length=length+1;
		q=q->next;
	}

	return length;
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to get the end elements in the list.
 *
 *  @param[in]  elem
 *      List for which the number of elements are required.
 *
 *  @retval
 *      the end Element
 */
sTracking_ListElem* tracking_listGetEnd (sTracking_ListObj* list)
{
	return list->end;
}


/** 
 *  @b Description
 *  @n  
 *      The function is called to merge a list to the end of another list.
 *
 *  @param[in,out]  list
 *      list_to from where the list_from is to be added
 *  @param[in]  list
 *      list_from to be added to the end of list_to
 *
 *  @retval
 *      void .
 */
void tracking_listMerge (sTracking_ListObj *list_dst, sTracking_ListObj *list_src)
{
	sTracking_ListElem* t_list_from_elem=0;

	if (0==list_src || (0==list_src->begin && 0==list_src->end))
	{
		return;
	}

	t_list_from_elem = tracking_listGetFirst(list_src);
	while (t_list_from_elem != 0)
	{
		sTracking_ListElem*tElemToMove = t_list_from_elem;

		t_list_from_elem = tracking_listGetNext(t_list_from_elem);
		tracking_listRemoveElement(list_src, tElemToMove);
		tracking_listEnqueue(list_dst, tElemToMove);
	}
}

/** 
 *  @b Description
 *  @n  
 *      The function is called to split a list to two list by length.
 *
 *  @param[in,out]  list
 *      the list to be split, the first part of split
 *  @param[in]  int
 *      the ahead number elem of src, split to the first    
 *  @param[out]  list
 *      the second part of split
 *
 *  @retval
 *      Not Applicable.
 */
void tracking_listSplit(sTracking_ListObj *list_src, int split_num, sTracking_ListObj *list_ahead_part)
{
	sTracking_ListElem* t_list_from_elem = tracking_listGetFirst(list_src);

	int count=0;
	while (t_list_from_elem != 0 && count < split_num)
	{
		sTracking_ListElem*tElemToMove = t_list_from_elem;

		t_list_from_elem = tracking_listGetNext(t_list_from_elem);
		tracking_listRemoveElement(list_src, tElemToMove);
		tracking_listEnqueue(list_ahead_part, tElemToMove);

		count++;
	}
}
