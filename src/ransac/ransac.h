/**
 * @file ransac.h
 * @brief Header-only RANSAC algorithms for point cloud processing
 * @author Markus Hueser
 * @date 2025-07-19
 * 
 * @details Reusable RANSAC implementation for plane segmentation.
 * Designed for sensor fusion projects (LiDAR, radar, camera).
 * Template-based to support various PCL point types (XYZ, XYZI, XYZRGB, etc.).
 */

#ifndef RANSAC_H
#define RANSAC_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ransac
{

/**
 * @brief Represents a 3D plane model: Ax + By + Cz + D = 0
 * 
 * The plane equation is derived from three points using the cross product
 * of vectors formed by the points to compute the normal vector (A, B, C).
 */
struct PlaneModel
{
    float A{0.0f};  ///< X component of plane normal
    float B{0.0f};  ///< Y component of plane normal
    float C{0.0f};  ///< Z component of plane normal
    float D{0.0f};  ///< Plane offset term
    float norm{0.0f};  ///< Precomputed normalization factor: sqrt(A^2 + B^2 + C^2)
    float inv_norm{0.0f};  ///< Precomputed inverse normalization factor: 1/norm (for faster distance calculations)
    
    /**
     * @brief Compute and store normalization factor and its inverse
     */
    void computeNormalization()
    {
        norm = std::sqrt(A * A + B * B + C * C);
        inv_norm = (norm > 0.0f) ? (1.0f / norm) : 0.0f;
    }
    
    /**
     * @brief Get the precomputed normalization factor
     * @return sqrt(A^2 + B^2 + C^2)
     */
    float normalizationFactor() const
    {
        return norm;
    }
    
    /**
     * @brief Get the precomputed inverse normalization factor
     * @return 1 / sqrt(A^2 + B^2 + C^2)
     */
    float inverseNormalizationFactor() const
    {
        return inv_norm;
    }
    
    /**
     * @brief Check if plane is valid (non-zero normal)
     * @return true if normal vector is non-zero
     */
    bool isValid() const
    {
        const float epsilon = 1e-6f;
        return (norm > epsilon);
    }
    
    /**
     * @brief Check if plane is approximately horizontal (for ground plane detection)
     * @param angleThreshold Maximum angle from horizontal in radians (default ~5 degrees)
     * @return true if plane normal is within angleThreshold of vertical (i.e., plane is horizontal)
     */
    bool isHorizontal(float angleThreshold = 0.0872665f) const // ~5 degrees in radians
    {
        if (!isValid())
        {
            return false;
        }
        // For a horizontal plane, the normal should be mostly vertical (pointing up or down)
        // The angle between normal and vertical axis (0,0,1) should be small
        // cos(theta) = (normal . vertical) / (|normal| * |vertical|) = |C| / norm
        float cosAngle = std::abs(C) / norm;
        // cos(theta) > cos(angleThreshold) means theta < angleThreshold
        return cosAngle > std::cos(angleThreshold);
    }
};

/**
 * @brief Fit a plane model to three points
 * 
 * Uses vector cross product to compute the plane normal:
 * - v1 = p1 - p0
 * - v2 = p2 - p0
 * - normal = v1 × v2
 * - Plane equation: normal · (X - p0) = 0
 * 
 * @tparam PointT PCL point type (XYZ, XYZI, etc.)
 * @param p0 First point
 * @param p1 Second point
 * @param p2 Third point
 * @return PlaneModel coefficients (A, B, C, D)
 */
template<typename PointT>
PlaneModel fitPlaneToPoints(const PointT& p0, const PointT& p1, const PointT& p2)
{
    // Create vectors from p0 to p1 and p0 to p2
    Eigen::Vector3f v1 = (p1.getVector3fMap() - p0.getVector3fMap());
    Eigen::Vector3f v2 = (p2.getVector3fMap() - p0.getVector3fMap());
    
    // Compute normal vector using cross product
    Eigen::Vector3f normal = v1.cross(v2);
    
    // Extract plane coefficients: Ax + By + Cz + D = 0
    // Normal vector gives us A, B, C
    PlaneModel plane;
    plane.A = normal.x();
    plane.B = normal.y();
    plane.C = normal.z();
    
    // D = -(A*p0.x + B*p0.y + C*p0.z)
    plane.D = -((plane.A * p0.x) + (plane.B * p0.y) + (plane.C * p0.z));
    
    // Precompute normalization factor for efficiency
    plane.computeNormalization();
    
    return plane;
}

/**
 * @brief Calculate signed distance from a point to a plane
 * 
 * Distance formula: |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
 * Optimized: Uses precomputed inverse normalization factor (1/norm) for multiplication instead of division
 * 
 * @tparam PointT PCL point type
 * @param point The point to measure distance from
 * @param plane The plane model
 * @return Signed distance from point to plane
 */
template<typename PointT>
float pointToPlaneDistance(const PointT& point, const PlaneModel& plane)
{
    const float numerator = std::abs(
        (plane.A * point.x) + 
        (plane.B * point.y) + 
        (plane.C * point.z) + 
        plane.D
    );

    return numerator * plane.inverseNormalizationFactor();
}

/**
 * @brief Compute plane quality statistics for the final selected plane
 * 
 * Calculates:
 * - Mean and median inlier distances
 * - Tilt and roll angles relative to ideal ground plane (z=0)
 * - Plane normal deviation from vertical
 * 
 * @tparam PointT PCL point type
 * @param cloud Input point cloud
 * @param plane The final selected plane model
 * @param inliers Indices of inlier points
 * @return String with formatted statistics
 */
template<typename PointT>
std::string computePlaneStatistics(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const PlaneModel& plane,
    const std::vector<int>& inliers)
{
    if (inliers.empty() || !plane.isValid())
    {
        return "";
    }
    
    // Compute inlier distance statistics using STL algorithms
    std::vector<float> distances(inliers.size());
    std::transform(inliers.begin(), inliers.end(), distances.begin(),
        [&](int idx) { return pointToPlaneDistance<PointT>(cloud->points[idx], plane); });
    
    // Compute mean distance
    const float mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();

    // Compute median distance
    std::vector<float> sorted_distances = distances;
    std::sort(sorted_distances.begin(), sorted_distances.end());
    const size_t mid = sorted_distances.size() / 2;
    const float median_distance = ((sorted_distances.size() % 2) == 0)
        ? ((sorted_distances[mid - 1] + sorted_distances[mid]) / 2.0f)
        : sorted_distances[mid];
    
    // Compute plane orientation angles (in degrees)
    // For an ideal ground plane (z=0), the normal should be (0, 0, 1) or (0, 0, -1)
    // Tilt: angle between normal and vertical in the xz-plane (pitch)
    // Roll: angle between normal and vertical in the yz-plane (roll)
    
    float tilt_rad = std::atan2(std::abs(plane.A), std::abs(plane.C));
    float roll_rad = std::atan2(std::abs(plane.B), std::abs(plane.C));
    float tilt_deg = tilt_rad * (180.0f / M_PI);
    float roll_deg = roll_rad * (180.0f / M_PI);
    
    // Overall angle from vertical (0,0,1)
    float angle_from_vertical_rad = std::acos(std::abs(plane.C) / plane.norm);
    float angle_from_vertical_deg = angle_from_vertical_rad * (180.0f / M_PI);
    
    // Format output string
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "  Quality: " << inliers.size() << " inliers (mean dist: " << mean_distance
        << "m, median dist: " << median_distance << "m) | "
        << "Orientation: tilt=" << tilt_deg << "°, roll=" << roll_deg << "°, "
        << "angle from vertical=" << angle_from_vertical_deg << "°";
    
    return oss.str();
}

/**
 * @brief Output RANSAC result summary to console
 * 
 * Provides formatted output of RANSAC execution statistics including:
 * - Number of valid planes tested
 * - Number of planes skipped due to orientation constraints
 * - Best plane found (inlier count and normal vector)
 * - Plane quality and orientation statistics
 * 
 * @tparam PointT PCL point type
 * @param cloud Input point cloud
 * @param bestPlane The best plane model found
 * @param bestInliers Indices of inlier points for the best plane
 * @param valid_plane_count Number of valid (non-colinear) planes tested
 * @param horizontal_plane_count Number of planes skipped for not being horizontal
 * @param maxIterations Total RANSAC iterations attempted
 */
template<typename PointT>
void outputRansacResults(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const PlaneModel& bestPlane,
    const std::vector<int>& bestInliers,
    int valid_plane_count,
    int horizontal_plane_count,
    int maxIterations)
{
    std::cout << "RANSAC: " << valid_plane_count << "/" << maxIterations
              << " valid planes, " << horizontal_plane_count << " skipped (non-horizontal), "
              << bestInliers.size() << " best inliers found" << std::endl;
    
    if (bestInliers.empty())
    {
        std::cout << "  WARNING: No valid plane found!" << std::endl;
    }
    else
    {
        std::cout << "  Best plane normal: (" << bestPlane.A << ", " << bestPlane.B << ", " << bestPlane.C << ")" << std::endl;
        std::cout << computePlaneStatistics<PointT>(cloud, bestPlane, bestInliers) << std::endl;
    }
}

/**
 * @brief Find inliers for a plane model
 * 
 * Iterates through all points in the cloud and checks if they are within
 * the distance threshold from the plane.
 * 
 * @tparam PointT PCL point type
 * @param cloud Input point cloud
 * @param plane Plane model to test against
 * @param distanceThreshold Maximum distance for a point to be considered an inlier
 * @return Vector of inlier point indices
 */
template<typename PointT>
std::vector<int> findPlaneInliers(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const PlaneModel& plane,
    const float distanceThreshold)
{
    std::vector<int> inliers;
    
    if (!plane.isValid())
    {
        return inliers;  // Return empty if plane is invalid
    }
    
    // Use range-based for loop and reserve space for efficiency
    inliers.reserve(cloud->points.size());
    int index = 0;
    for (const auto& point : cloud->points)
    {
        float distance = pointToPlaneDistance<PointT>(point, plane);
        if (distance < distanceThreshold)
        {
            inliers.push_back(index);
        }
        ++index;
    }
    
    return inliers;
}

/**
 * @brief RANSAC-based plane segmentation
 * 
 * Implements the RANSAC algorithm for robust plane fitting in point clouds:
 * 1. Randomly sample 3 points from the cloud
 * 2. Fit a plane to the sampled points
 * 3. Count inliers (points within distanceThreshold of the plane)
 * 4. Keep the plane with the most inliers
 * 5. Repeat for maxIterations
 * 
 * @tparam PointT PCL point type (XYZ, XYZI, XYZRGB, etc.)
 * @param cloud Input point cloud to segment
 * @param maxIterations Number of RANSAC iterations (default: 100)
 * @param distanceThreshold Maximum distance for inlier classification (default: 0.2)
 * @param randomSeed Optional random seed for reproducibility (0 = use time)
 * @param angleThreshold Maximum angle from horizontal for valid ground plane (default: ~5 degrees)
 * @param enforceHorizontal If true, only consider approximately horizontal planes (default: true)
 * @return pcl::PointIndices::Ptr containing indices of inlier points (plane)
 * 
 * @note This implementation uses C++11 <random> for sampling to ensure
 *       proper random seeding across all iterations.
 * @note For flat world scenarios with sensor noise, this provides
 *       robust ground plane detection.
 */
template<typename PointT>
typename pcl::PointIndices::Ptr ransacPlaneSegmentation(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    int maxIterations = 100,
    float distanceThreshold = 0.2f,
    unsigned randomSeed = 0,
    float angleThreshold = 0.0872665f, // ~5 degrees in radians
    bool enforceHorizontal = true)
{
    // Validate input
    if (!cloud || cloud->points.empty())
    {
        return pcl::PointIndices::Ptr(new pcl::PointIndices);
    }
    
    // Setup random number generator
    std::mt19937 generator;
    if (randomSeed != 0)
    {
        generator.seed(randomSeed);
    }
    else
    {
        generator.seed(static_cast<unsigned>(std::random_device{}()));
    }
    
    // Storage for best model
    PlaneModel bestPlane{};
    std::vector<int> bestInliers{};
    
    // Storage for sampled points
    std::vector<int> sampled_indices(3);
    
    // Main RANSAC loop
    int valid_plane_count = 0;
    int horizontal_plane_count = 0;
    
    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        // Sample 3 distinct random point indices from the cloud
        const int cloud_size = static_cast<int>(cloud->points.size());
        std::uniform_int_distribution<int> distribution(0, cloud_size - 1);
        
        // Try to get 3 distinct points (max 100 attempts)
        int attempts = 0;
        const int max_attempts = 100;
        bool has_duplicates = true;
        
        while (has_duplicates && attempts < max_attempts)
        {
            // Sample 3 indices using STL generate
            std::generate(sampled_indices.begin(), sampled_indices.end(),
                [&]() { return distribution(generator); });
            
            // Check for duplicates: with only 3 elements, direct comparison is most efficient
            has_duplicates = (sampled_indices[0] == sampled_indices[1]) ||
                           (sampled_indices[0] == sampled_indices[2]) ||
                           (sampled_indices[1] == sampled_indices[2]);
            ++attempts;
        }
        
        if (has_duplicates)
        {
            // Failed to get distinct points after max attempts, skip
            continue;
        }
        
        // Fit plane to sampled points
        const PointT& p0 = cloud->points[sampled_indices[0]];
        const PointT& p1 = cloud->points[sampled_indices[1]];
        const PointT& p2 = cloud->points[sampled_indices[2]];
        
        auto const plane = fitPlaneToPoints<PointT>(p0, p1, p2);
        
        // Skip invalid planes (e.g., colinear points)
        if (!plane.isValid())
        {
            continue;
        }
        ++valid_plane_count;
        
        // Skip non-horizontal planes if enforcement is enabled
        if (enforceHorizontal && !plane.isHorizontal(angleThreshold))
        {
            horizontal_plane_count++;
            continue;
        }
        
        // Find inliers for this plane
        auto inliers = findPlaneInliers<PointT>(cloud, plane, distanceThreshold);
        
        // Update best model if this one has more inliers
        if (inliers.size() > bestInliers.size())
        {
            bestPlane = plane;
            bestInliers = std::move(inliers);
        }
    }
    
    // Output summary statistics
    outputRansacResults<PointT>(
        cloud, bestPlane, bestInliers,
        valid_plane_count, horizontal_plane_count, maxIterations);
    
    // Convert to PCL PointIndices format
    typename pcl::PointIndices::Ptr result{new pcl::PointIndices};
    result->indices = std::move(bestInliers);
    
    return result;
}

} // namespace ransac

#endif // RANSAC_H
