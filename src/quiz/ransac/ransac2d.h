#if !defined(RANSAC2D_H_)

#define RANSAC2D_H_
#include <unordered_set>
#include <pcl/io/pcd_io.h>




template <typename PointT> 
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif // RANSAC2D_H_

