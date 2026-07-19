// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <filesystem>


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::applyBoxCropOnCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Vector4f minPoint,
    const Eigen::Vector4f maxPoint,
    const bool isNegative)
{
    // Time cropping process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cropped_cloud{new pcl::PointCloud<PointT>()};

    pcl::CropBox<PointT> box_crop{true};
    box_crop.setMin(minPoint);
    box_crop.setMax(maxPoint);
    box_crop.setInputCloud(cloud);

    if (isNegative)  // inverted crop
    {
        std::vector<int> cropped_points_indices;
        box_crop.filter(cropped_points_indices);

        pcl::PointIndices::Ptr inliers_crop{new pcl::PointIndices};
        inliers_crop->indices = cropped_points_indices;  // Direct assignment is more efficient

        pcl::ExtractIndices<PointT> point_extractor{};
        point_extractor.setInputCloud(cloud);
        point_extractor.setIndices(inliers_crop);
        point_extractor.setNegative(isNegative);
        point_extractor.filter(*cropped_cloud);
    }
    else
    {
        box_crop.filter(*cropped_cloud);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "cropping took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped_cloud;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const float filterRes,
    const Eigen::Vector4f minPoint,
    const Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Apply voxel grid filter
    pcl::VoxelGrid<PointT> filter{};
    filter.setInputCloud(cloud);
    filter.setLeafSize(filterRes, filterRes, filterRes);

    typename pcl::PointCloud<PointT>::Ptr filtered_cloud{new pcl::PointCloud<PointT>()};
    filter.filter(*filtered_cloud);

    // Crop filterd point cloud to region of interest (RoI)
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud = this->applyBoxCropOnCloud(filtered_cloud, minPoint, maxPoint, false);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    const pcl::PointIndices::Ptr& inliers,
    const typename pcl::PointCloud<PointT>::Ptr& cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr ground_plane_cloud{new pcl::PointCloud<PointT>()};
    ground_plane_cloud->reserve(inliers->indices.size());
    
    for (int index : inliers->indices)
    {
        ground_plane_cloud->push_back(cloud->points[index]);
    }

    typename pcl::PointCloud<PointT>::Ptr obstacles_cloud{new pcl::PointCloud<PointT>()};
    pcl::ExtractIndices<PointT> point_extractor{};
    point_extractor.setInputCloud(cloud);
    point_extractor.setIndices(inliers);
    point_extractor.setNegative(true);
    point_extractor.filter(*obstacles_cloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_cloud, ground_plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const int maxIterations,
    const float distanceThreshold)
{
    std::cout << cloud->size() << "\n";

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Prepare output container
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg{};
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate planar component of input cloud!" << "\n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation (PCL) took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const int maxIterations,
    const float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Use custom RANSAC implementation for plane segmentation
    // Parameters: maxIterations, distanceThreshold, randomSeed, angleThreshold, enforceHorizontal
    // Using 0.349 radians (~20 degrees) for better tolerance with real road data
    typename pcl::PointIndices::Ptr inliers = ransac::ransacPlaneSegmentation<PointT>(
        cloud, maxIterations, distanceThreshold, 0, 0.34906585f, true);

    if (inliers->indices.empty())
    {
        std::cout << "Could not estimate planar component of input cloud!" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation (RANSAC) took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Reuse existing SeparateClouds to split into obstacles and plane
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const float clusterTolerance,
    const int minSize,
    const int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Use PCL's EuclideanClusterExtraction
    typename pcl::search::KdTree<PointT>::Ptr tree{new typename pcl::search::KdTree<PointT>};
    tree->setInputCloud(cloud);

    // Setup and parameterize
    typename pcl::EuclideanClusterExtraction<PointT> ece{};
    ece.setClusterTolerance(clusterTolerance);
    ece.setMinClusterSize(minSize);
    ece.setMaxClusterSize(maxSize);
    ece.setSearchMethod(tree);

    // Set input
    ece.setInputCloud(cloud);

    // Set output
    std::vector<pcl::PointIndices> cluster_indices;
    ece.extract(cluster_indices);

    for(auto const& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster{new typename pcl::PointCloud<PointT>};
        cloud_cluster->reserve(cluster.indices.size());
        for (auto const& idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering (PCL) took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const float clusterTolerance,
    const int minSize,
    const int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Use custom clustering implementation
    auto cluster_indices = clustering::clusterPointCloud<PointT>(cloud, clusterTolerance);
    
    // Filter clusters by size (minSize, maxSize)
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (const auto& cluster : cluster_indices)
    {
        // Skip clusters that don't meet size requirements
        if (cluster.size() < static_cast<size_t>(minSize) || cluster.size() > static_cast<size_t>(maxSize))
        {
            continue;
        }
        
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster{new typename pcl::PointCloud<PointT>};
        cloud_cluster->reserve(cluster.size());
        for (int idx : cluster)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering (Custom) took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box{
        minPoint.x,
        minPoint.y,
        minPoint.z,
        maxPoint.x,
        maxPoint.y,
        maxPoint.z
    };

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud{new pcl::PointCloud<PointT>};

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath}, std::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}