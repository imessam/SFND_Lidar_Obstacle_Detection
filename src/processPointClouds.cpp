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
    typename pcl::CropBox<PointT> cropBox;

    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    cropBox.setInputCloud(cloud_filtered);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.filter(*cloud_filtered);

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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, int type)
{

    if(type == 0){
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        // TODO:: Fill in this function to find inliers for the cloud.

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }


        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        return segResult;
    }

    else{

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
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, int type)
{

    if(type == 0){
        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        typename pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance); // 2cm
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {

            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>());

            for (const auto &idx : it->indices)
                cloud_cluster->push_back((*cloud)[idx]); //*

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
            j++;
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

        return clusters;
    }else{


        KdTree<PointT> *tree = new KdTree<PointT>();

        for (int i = 0; i < cloud->points.size(); i++)
            tree->insert(cloud->points[i], i);

        // int it = 0;
        // render2DTree(tree->root, viewer, window, it);

        // std::cout << "Test Search" << std::endl;
        // std::vector<int> nearby = tree->search(CreateData({{-6,7}})->points[0], 3.0);
        // for (int index : nearby)
        //     std::cout << index << ",";
        // std::cout << std::endl;

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

	// TODO: Fill in this function

	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
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