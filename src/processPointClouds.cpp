// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <filesystem>
#include <numeric>


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
bool ProcessPointClouds<PointT>::isClusterPoleLike(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& clusterIndices,
    const float poleLikeRatioThreshold)
{
    // Use Welford's online algorithm to compute mean and variance in a single pass
    // This reduces from 6 passes (3 for mean + 3 for variance) to just 1 pass
    int count = 0;
    float mean_x = 0.0f, mean_y = 0.0f, mean_z = 0.0f;
    float M2_x = 0.0f, M2_y = 0.0f, M2_z = 0.0f;
    
    for (int idx : clusterIndices)
    {
        const auto& point = (*cloud)[idx];
        ++count;
        
        // Welford's algorithm for x
        float delta_x = point.x - mean_x;
        mean_x += delta_x / count;
        M2_x += delta_x * (point.x - mean_x);
        
        // Welford's algorithm for y
        float delta_y = point.y - mean_y;
        mean_y += delta_y / count;
        M2_y += delta_y * (point.y - mean_y);
        
        // Welford's algorithm for z
        float delta_z = point.z - mean_z;
        mean_z += delta_z / count;
        M2_z += delta_z * (point.z - mean_z);
    }
    
    // Compute final variances (using population variance: divide by n)
    const float var_x = M2_x / count;
    const float var_y = M2_y / count;
    const float var_z = M2_z / count;
    
    // Combined horizontal variance (this is just a heuristic!)
    const float var_horizontal = var_x + var_y;
    
    // A cluster is pole-like if vertical variance dominates horizontal variance (my model assumption!)
    return (var_z >= (poleLikeRatioThreshold * var_horizontal));
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const float clusterTolerance,
    const int minSizeForPoleLikeClusters,
    const int minSizeForBoxLikeClusters,
    const int maxSizeForBoxLikeClusters)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Use custom clustering implementation
    auto cluster_indices = clustering::clusterPointCloud<PointT>(cloud, clusterTolerance);
    
    // Filter clusters by size (minSize, maxSize)
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (const auto& cluster : cluster_indices)
    {
        const int cluster_size = static_cast<int>(cluster.size());
        // Skip clusters that don't meet size requirements (note: poles can have slightly lower number of points and are checked for below)
        if ((cluster_size < minSizeForPoleLikeClusters) || (cluster_size > maxSizeForBoxLikeClusters))
        {
            continue;
        }

        // For small clusters, check if they have a pole-like appearance
        if ((cluster_size < minSizeForBoxLikeClusters) && (!isClusterPoleLike(cloud, cluster, 3.0f)))
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