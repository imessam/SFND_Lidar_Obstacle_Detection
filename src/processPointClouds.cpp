// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
  
  	// Create the filtering object
	typename pcl::VoxelGrid<PointT> sor;
  	typename pcl::CropBox<PointT> cropBox;
  	
	sor.setInputCloud (cloud);
	sor.setLeafSize (filterRes ,filterRes, filterRes);
	sor.filter (*cloud_filtered);
  
  	cropBox.setInputCloud(cloud_filtered);
  	cropBox.setMin(minPoint);
  	cropBox.setMax(maxPoint);
  	cropBox.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    // Create the filtering object
	pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr  cloud_p ( new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);


	// While 30% of the original cloud is still there
	for(int index: inliers->indices)
      	cloud_p->points.push_back(cloud->points[index]);
	
      
        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        // Create the filtering object
       	extract.setNegative (true);
        extract.filter (*cloud_f);

  
    

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.
	
  
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  
  
  	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);
  
	
	  // Segment the largest planar component from the remaining cloud
	 seg.setInputCloud (cloud);
	 seg.segment (*inliers, *coefficients);
	  if (inliers->indices.size () == 0)
	  {
	    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	  }

      
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

     // Creating the KdTree object for the search method of the extraction
     typename pcl::search::KdTree<PointT>::Ptr tree (new  pcl::search::KdTree<PointT>());
     tree->setInputCloud (cloud);
     
     std::vector<pcl::PointIndices> cluster_indices;
     typename pcl::EuclideanClusterExtraction<PointT> ec;
     ec.setClusterTolerance (clusterTolerance); // 2cm
     ec.setMinClusterSize (minSize);
     ec.setMaxClusterSize (maxSize);
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud);
     ec.extract (cluster_indices);
  
  
  
  	 int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
       
     		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());
       
			for (const auto& idx : it->indices)
					cloud_cluster->push_back ((*cloud)[idx]); //*
       
			cloud_cluster->width = cloud_cluster->size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
       		clusters.push_back(cloud_cluster);

     		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
     		j++;
     }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}