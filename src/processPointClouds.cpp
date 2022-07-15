// PCL lib Functions for processing point clouds

#include "processPointClouds.h"


// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // Create the filtering object
    typename pcl::VoxelGrid<PointT> sor;
    typename pcl::CropBox<PointT> cropBox, roof;
    typename pcl::PointIndices::Ptr roofInliers (new pcl::PointIndices());
    typename pcl::ExtractIndices<PointT> extract;
    std::vector<int> roofIndices;

    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    cropBox.setInputCloud(cloud_filtered);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.filter(*cloud_filtered);


    roof.setInputCloud(cloud_filtered);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.filter(roofIndices);

    
    for(int idx : roofIndices)
        roofInliers->indices.push_back(idx);
    
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(roofInliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
     auto startTime = std::chrono::steady_clock::now();
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>), cloud_f(new pcl::PointCloud<PointT>);

    // While 30% of the original cloud is still there
    for (int index : inliers->indices)
        cloud_p->points.push_back(cloud->points[index]);

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "seperating took  " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{


        auto startTime = std::chrono::steady_clock::now();

        std::unordered_set<int> inliers;
        typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

        if (cloud->points.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }else{

            inliers = Ransac(cloud, maxIterations, distanceThreshold);  

            for (int index = 0; index < cloud->points.size(); index++)
            {
                PointT point = cloud->points[index];
                if (inliers.count(index))
                    cloudInliers->points.push_back(point);
                else
                    cloudOutliers->points.push_back(point);
            }
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
        return segResult;
    
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

        KdTree<PointT> *tree = new KdTree<PointT>();

        for (int i = 0; i < cloud->points.size(); i++)
            tree->insert(cloud->points[i], i);

        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        //
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster(cloud, tree, 3.0);
        //
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

        return clusters;
 
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}


template <typename PointT> 
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int offset = 0, range = cloud->points.size();
	PointT p, p1, p2, p3;
	float A, B, C, D, d,eps = 1e-6;

    std::cout<<range<<std::endl;

	for (int i = 0; i < maxIterations; i++)
	{
		p1 = cloud->points[offset + (rand() % range)];
		p2 = cloud->points[offset + (rand() % range)];
		p3 = cloud->points[offset + (rand() % range)];

		A = ((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y));
		B = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z));
		C = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.x));
		D = -((A * p1.x) + (B * p1.y) + (C * p1.z));


		std::unordered_set<int> tempInliers;

		for (int j = 0; j < range; j++)
		{

			p = cloud->points[j];
			d = fabs((A * p.x) + (B * p.y) + (C * p.z) + D) / (sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2))+eps);
			if (d < distanceTol)
				tempInliers.insert(j);
		}

		if (tempInliers.size() > inliersResult.size())
		{
			inliersResult = tempInliers;
		}
	}

	return inliersResult;
}

template <typename PointT> 
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud,  KdTree<PointT> *tree, typename pcl::PointCloud<PointT>::Ptr cluster, std::unordered_set<int> &processed, int idx, float distanceTol)
{

	processed.insert(idx);
	cluster->points.push_back(cloud->points[idx]);
	std::vector<int> near = tree->search(cloud->points[idx], distanceTol);

	for (int i = 0; i < near.size(); i++)
	{
		if (processed.find(near[i]) == processed.end())
		{
			proximity(cloud, tree, cluster, processed, near[i], distanceTol);
		}
	}
}

template <typename PointT> 
std::vector<typename pcl::PointCloud<PointT>::Ptr>  ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,  KdTree<PointT> *tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::unordered_set<int> processed;

	for (int i = 0; i < cloud->points.size(); i++)
	{

		if (processed.find(i) == processed.end())
		{
			typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>());
			proximity(cloud, tree, cluster, processed, i, distanceTol);
			clusters.push_back(cluster);
		}
	}

	return clusters;
}