
#include <string.h>
#include <math.h>
#include <float.h>
#include "../include/tracking_common.h"
#include "../include/tracking_int.h"

void tracking_pointsClustering(void *handle, \
	sMeasurement_SphPoint *point, uint16_t num, int32_t* pointClusterNum)
{
	//*pointClusterNum = tracking_DBscan(handle, point, num);
}

/*!
   \fn     tracking_DBscan
   \ret     the number of cluster.
*/
//int32_t tracking_DBscan(void* handle, sMeasurement_SphPoint* point, \
//	uint16_t num)
//{
//    uint16_t *neighLast;
//    uint16_t *neighCurrent;
//    uint16_t neighCount;
//    uint16_t newCount;
//    uint16_t n, member;
//    uint16_t numPoints;
//    uint16_t clusterId;
//    uint16_t ind;
//	sTracking_moduleInstance *inst = NULL;
//    float epsilon,epsilon2, weight;
//    float vFactor;
//	int16_t speed = 0;
//	uint16_t minNumPointsInCluster = 0;
//	uint8_t visited[MAX_NUM_DETECTS] = { 0 };
//	uint8_t scope[MAX_NUM_DETECTS] = { 0 };
//	uint16_t neighbors[MAX_NUM_DETECTS] = { 0 };
//	sTracking_platformInfo* platformInfo = NULL;
//
//	inst = (sTracking_moduleInstance*) handle;
//	platformInfo = &inst->platformInfo;
//    numPoints    = num;
//    clusterId    =    0;
//    epsilon        =    inst->tracking_params->dbscan_epsilon;
//    epsilon2    =    epsilon*epsilon;
//	minNumPointsInCluster = inst->tracking_params->dbscan_minNumPoints;
//    weight        =    inst->tracking_params->cluster_weight;
//    vFactor = inst->tracking_params->cluster_vThres;
//    
//    memset(visited, CLUSTER_POINT_UNKNOWN, numPoints*sizeof(uint8_t));
//
//	// Init the clusterId of points are CLUSTER_POINT_NOISE
//    for(n = 0; n < numPoints; n++){
//		point[n].clusterId = CLUSTER_POINT_NOISE;
//	}
//    // scan through all the points to find its neighbors
//    for(n = 0; n < numPoints; n++)
//    {
//        if(visited[n] != CLUSTER_POINT_VISITED)
//        {
//            visited[n] = CLUSTER_POINT_VISITED;
//
//            neighCurrent = neighLast = neighbors;
//            // scope is the local copy of visit
//            memcpy(scope, visited, numPoints*sizeof(uint8_t));
//
//            neighCount = tracking_DBscan_findNeighbors(
//                point, n, neighLast, numPoints, epsilon2, epsilon, weight,
//                vFactor,scope, &newCount, platformInfo);
//#ifdef DEBUG_LOG
//			//my_printf("det[%d]: neightCount: %d, weight: %.3f, epsilon: %.3f, epsilon2: %.3f \n", \
//				n, neighCount, weight, epsilon, epsilon2);
//#endif
//            /* The cluster consists of the point itself and its neighbours. */    
//            if(neighCount < minNumPointsInCluster - 1) 
//            {
//                // This point is Noise
//                point[n].clusterId = CLUSTER_POINT_NOISE;
//            }
//            else
//            {
//                // This point belongs to a New Cluster
//                clusterId++;                                // New cluster ID
//                point[n].clusterId = clusterId;      // This point belong to this cluster
//
//                // tag all the neighbors as visited in scope so that it will not be found again when searching neighbor's neighbor.
//                for (ind = 0; ind < newCount; ind ++)
//                {
//                    member = neighLast[ind];
//                    scope[member] = CLUSTER_POINT_VISITED;
//                }
//                neighLast += newCount;
//
//                while (neighCurrent != neighLast)               // neigh shall be at least minPoints in front of neighborhood pointer
//                {
//                    // Explore the neighborhood
//                    member = *neighCurrent++;               // Take point from the neighborhood
//                    point[member].clusterId = clusterId; // All points from the neighborhood also belong to this cluster
//                    visited[member] = CLUSTER_POINT_VISITED;
//
//                    neighCount = tracking_DBscan_findNeighbors(
//                                            point, member, neighLast, numPoints,
//                                            epsilon2, epsilon, weight,
//                                            vFactor, scope, &newCount, platformInfo);
//
//                    if(neighCount >= minNumPointsInCluster)
//                    {
//                        speed = (int16_t) point[n].vector.doppler;
//                        /* Check if the new point's velocity is the same velocity as the  
//                         * clusterOriginator. */
//                        if (fabsf(point[member].vector.doppler - speed) < vFactor)
//                        {
//                            for (ind = 0; ind < newCount; ind ++)
//                            {
//                                member = neighLast[ind];
//                                scope[member] = CLUSTER_POINT_VISITED;
//                            }
//                            neighLast += newCount;              /* Member is a core point, and its neighborhood is added to the cluster */
//                        }
//                    }
//                }
//
//                if (clusterId >= MAX_NUM_DETECTS)
//                {
//                    return -1;
//                }
//            }
//        }
//    }
//    return clusterId;
//}

/*!
   \fn     tracking_DBscan_findNeighbors
   \brief   The core algorithm of dBScan, where for each point, the list of 
            its neighbours are found based on a distance metric. 
   \param[in]    point
               Pointer to input data (object list).
   \param[in]    n
               The current point. 
   \param[out]   neigh,
               Pointer to the array holding the cluster's points. If the current
               point has enough neighbours, it is upgraded to a core point, and 
               its neighbours are added to the cluster. 
   \param[in]    numPoints
               Number of detected objects. 
   \param[in]    epsilon2
               square of the distance metric (in fixed point)
    \param[in]    epsilon
               the distance metric (in fixed point)
    \param[in]    weight
                the relative weight assigned to the velocity difference. 
    \param[in]    vFactor
                the velocity metric. 
    \param[in]    visited
                An array holding the state of the points. In this context, state
                means whether the point has been visited before. 
    \param[out]    newCount
                The number of neighbours for this point. 
   \ret     number of neighbours.
 */
uint16_t tracking_DBscan_findNeighbors(
                    sMeasurement_SphPoint *point,
                    uint16_t n,
                    uint16_t *neigh,
                    uint16_t numPoints,
                    float epsilon2,
                    float epsilon,
                    float weight,
                    float vFactor,
                    uint8_t *visited,
                    uint16_t *newCount,
					sTracking_platformInfo* platformInfo)
{
    float a,b,c;
    uint16_t i;
    float sum, epsilon2WithSpeed, itemp, epsilonX, epsilonY;
    uint32_t newCounttmp = 0;
    float speed = point[n].vector.doppler;
    float range = point[n].vector.range;
	float azimuth = point[n].vector.azimuthRad;
    float x = point[n].x;
    float y = point[n].y;
	float dAbsVel = 100.f;
	float sin_n = 1.0f;
	float sin_i = 1.0f;

    //itemp        =    vFactor;
    //if (fabsf(speed) < vFactor)
    //{
    //    itemp = fabsf(speed);
    //}
	//epsilonX = 1.0f;
	if(fabsf(y)<10.f){
		if(    fabsf(platformInfo->egoLineVel_lat)>5.f \
			|| fabsf(platformInfo->egoLineVel_long)>5.f ){
			epsilonX = 2.0f;
			epsilonY = 3.f*epsilon;
			itemp = 10.f;
		}
		else{
			epsilonX = 0.5f;
			epsilonY = 2.f*epsilon;
			itemp = 5.f;
		}
	}
	else{
		epsilonX = 1.0f;
		epsilonY = epsilon;
		itemp = vFactor;
	}
    epsilon2WithSpeed = (int32_t)(itemp*itemp*weight + epsilon2);
    
    for(i = 0; i < numPoints; i++)
    {
        if (visited[i] == CLUSTER_POINT_VISITED)
        {
            continue; 
        }
		if(    fabsf(azimuth)<5.f*DEG2RAD \
			|| fabsf(point[i].vector.azimuthRad)<5.f*DEG2RAD){
			dAbsVel = point[i].vector.doppler - speed;
			if (fabsf(dAbsVel) > 100.f){
				continue;
			}
		}
		else{
			sin_i = sinf(point[i].vector.azimuthRad);
			sin_n = sinf(azimuth);
			dAbsVel = point[i].vector.doppler/sin_i - speed/sin_n;
			if (fabsf(dAbsVel) > itemp){
				continue;
			}
		}
        //if (fabsf(point[i].vector.range - range) > epsilon)
        if (    fabsf(point[i].x - point[n].x) > epsilonX \
			 || fabsf(point[i].y - point[n].y) > epsilonY )
        {
            continue;
        }    
        a = point[i].x - x;
        b = point[i].y - y;
        c = point[i].vector.doppler - speed;
        sum = (a*a) + (b*b) + (int32_t)(weight*(float)(c*c));
        if (1 || sum < epsilon2WithSpeed)
        {
            /* Mark this point as a neighbour in the list of 
             * neighbours. Also increment the number of neighbours
             * for this point. */
            *neigh = i;
            neigh++;
            newCounttmp++;
        }
    }
    *newCount = (uint16_t) newCounttmp;
   return newCounttmp;
}

