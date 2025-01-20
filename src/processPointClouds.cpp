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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr input_cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // first we need a new poinmt cloud to store
    typename pcl::PointCloud<PointT>::Ptr downsampled_cloud (new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud (input_cloud);
    voxel_filter.setLeafSize (filterRes, filterRes, filterRes);
    voxel_filter.filter (*downsampled_cloud);

    // RoI - Region of Interest/ Relevance for the car to consider
    typename pcl::PointCloud<PointT>::Ptr boxed_cloud (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> box_filter(true);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.setInputCloud(downsampled_cloud);
    box_filter.filter(*boxed_cloud);

    // Remove Car's response
    // store the points belonging to the car and make a box for the car to later subratct
    // then identify the point inside and remove those from the boxed_cloud
    std::vector<int> indices;
    pcl::CropBox<PointT> car_filter(true);
    car_filter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    car_filter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    car_filter.setInputCloud(boxed_cloud);
    car_filter.filter(indices);

    pcl::PointIndices::Ptr car_box_inliers{new pcl::PointIndices};
    for (int point: indices)
        car_box_inliers->indices.push_back(point);

    // now remove indicies from the car itself, the car_box_inliers
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(car_box_inliers);
    extract.setNegative(true);
    extract.setInputCloud(boxed_cloud);
    extract.filter(*cloud_filtered);

    // Give feedback to the user
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
         << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr ground (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT> ());
  
  // Extract the inliers -> ground plane
  pcl::ExtractIndices<PointT> extract_ground;  
  extract_ground.setInputCloud (cloud);
  extract_ground.setIndices (inliers);
  extract_ground.setNegative (false);
  extract_ground.filter (*ground);

  // Extract the inliers -> obstacles plane
  pcl::ExtractIndices<PointT> extract;  
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*obstacles);
  
  std::cerr << "PointCloud representing the planar component: " << ground->width * ground->height << " data points." << std::endl;  
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, ground);
  
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // Create a vector to store the inliers for the ground plane
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  
    // set the input cloud and segment it into 2 outputs: inliers which define the plane, and math coefficient for rendering
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

// copied the oimplementation from the quiz and mods to work with given point cloud data of type PointT
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// for etsimation how long this process takes
	auto startTime = std::chrono::steady_clock::now();

	// A set to store data
	std::unordered_set<int> inliersResult; // a set always has unique elements -> no identicals coming from rand
	srand(time(NULL));
	
	while (maxIterations--) {
        std::unordered_set<int> temp_result; // create a temporary result to store and compare the size

        //UDACITY solution
		//while (temp_result.size() < 2) // just draw 2 points from the cloud randomlyinliersResult
        //    temp_result.insert(rand()%(cloud->points.size()));

		// 3 Point form a plane
		pcl::PointT point_1 = cloud -> points.at(rand()%(cloud->points.size()));
		pcl::PointT point_2 = cloud -> points.at(rand()%(cloud->points.size()));
		pcl::PointT point_3 = cloud -> points.at(rand()%(cloud->points.size()));

		// Define helpers A,B,C
        float A, B, C, D;

        A = (point_2.y - point_1.y) * (point_3.z - point_1.z) - (point_2.z - point_1.z) * (point_3.y - point_1.y);
        B = (point_2.z - point_1.z) * (point_3.x - point_1.x) - (point_2.x - point_1.x) * (point_3.z - point_1.z);
        C = (point_2.x - point_1.x) * (point_3.y - point_1.y) - (point_2.y - point_1.y) * (point_3.x - point_1.x);
        D = -1 * (A * point_1.x + B * point_1.y + C * point_1.z);

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the two points that are sample points; if the count is larger than 0 then it is already in the set!
            if (temp_result.count(index) > 0)
                continue;

			// the point to check whether it is part of the plane
            pcl::PointT point_4 = cloud->points[index];

            float d = fabs(A * point_4.x + B * point_4.y + C * point_4.z + D) / sqrt(A * A + B * B + C * C);
            if (d <= distanceTol) {
                temp_result.insert(index);
            }
        }
        if (temp_result.size() > inliersResult.size()) {
            inliersResult = temp_result;
        }
    }

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac3D took " << elapsedTime.count() << "ms" << std::endl;
	
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Vector to store the different obstacles
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> clusterIndices;

    // Create a KdTree object for the search method of the extraction
    // remeber to use the type definitions
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // Extraction example
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (auto it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        // Create a cluster with the extracted points belonging to the same object
        for (auto pointIt = it->indices.begin(); pointIt != it->indices.end(); ++pointIt)
        {
            cluster->push_back(cloud->points[*pointIt]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        // Add the cluster to the return cluster vector
        clusters.push_back(cluster);
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
std::vector<fs::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    //std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    std::vector<fs::path> paths(fs::directory_iterator{dataPath}, fs::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}