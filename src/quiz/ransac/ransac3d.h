/* \author Ray Phan
 * \description Header for quiz answer to 3D RANSAC for detecting the ground
 * plane
 */
#ifndef _RANSAC_3D_H_
#define _RANSAC_3D_H_

#include <pcl/common/common.h>
#include <unordered_set>

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 int maxIterations,
                                 float distanceTol);
#endif