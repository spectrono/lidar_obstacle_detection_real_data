/**
 * @file cluster.h
 * @brief Header-only Euclidean clustering algorithms for point cloud processing
 * @author Markus Hueser
 * @date 2025-07-18
 * 
 * @details Reusable Euclidean clustering implementation using KD-Tree.
 * Template-based to support various PCL point types (XYZ, XYZI, XYZRGB, etc.).
 * 
 * This implementation is based on the quiz code from cluster.cpp and kdtree.h,
 * adapted to work with PCL point types.
 */

#ifndef CLUSTER_H
#define CLUSTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace clustering
{

/**
 * @brief Structure to represent node of kd tree for PCL points
 * 
 * @tparam PointT PCL point type (XYZ, XYZI, etc.)
 * @tparam Scalar Type for distance calculations (typically float)
 */
template<typename PointT, typename Scalar = float>
struct KdTreeNode
{
    PointT point{};
    int id{0};
    KdTreeNode* left{nullptr};
    KdTreeNode* right{nullptr};

    KdTreeNode(const PointT& arr, int setId)
        : point(arr), id(setId), left(nullptr), right(nullptr)
    {}
};

/**
 * @brief KD-Tree implementation for PCL point clouds
 * 
 * Supports efficient nearest neighbor search in k-dimensional space.
 * Specialized for 3D points (XYZ) with Euclidean distance.
 * 
 * @tparam PointT PCL point type (XYZ, XYZI, etc.)
 * @tparam Scalar Type for distance calculations (typically float)
 */
template<typename PointT, typename Scalar = float>
class KdTree
{
public:
    KdTreeNode<PointT, Scalar>* root{nullptr};

    KdTree() : root(nullptr) {}

    ~KdTree()
    {
        clear();
    }

    /**
     * @brief Clear all nodes from the tree
     */
    void clear()
    {
        clearNode(root);
        root = nullptr;
    }

    /**
     * @brief Insert a point into the KD-Tree
     * 
     * @param point The point to insert
     * @param id The index of the point in the original cloud
     */
    void insert(const PointT& point, int id)
    {
        insertAtNode(root, 0, point, id);
    }

    /**
     * @brief Find all points within a given distance of the target point
     * 
     * @param target The target point to search around
     * @param distanceTol The maximum distance for a point to be included
     * @return Vector of indices of points within distanceTol
     */
    std::vector<int> search(const PointT& target, Scalar distanceTol) const
    {
        std::vector<int> ids;
        searchFromNode(root, target, 0, distanceTol, ids);
        return ids;
    }

private:
    /**
     * @brief Recursively clear tree nodes
     */
    void clearNode(KdTreeNode<PointT, Scalar>* node)
    {
        if (node)
        {
            clearNode(node->left);
            clearNode(node->right);
            delete node;
        }
    }

    /**
     * @brief Insert a point at a specific node
     * 
     * @param node The current node
     * @param depth The current depth in the tree (determines split axis)
     * @param point The point to insert
     * @param id The index of the point
     */
    void insertAtNode(KdTreeNode<PointT, Scalar>*& node, const uint depth, const PointT& point, int id)
    {
        if (node == nullptr)
        {
            node = new KdTreeNode<PointT, Scalar>(point, id);
        }
        else
        {
            // Get current split axis (0 = x, 1 = y, 2 = z, cycles)
            const uint current_depth = depth % 3;

            // Compare points based on current axis
            const Scalar node_val = getAxisValue(node->point, current_depth);
            const Scalar point_val = getAxisValue(point, current_depth);

            if (point_val < node_val)
            {
                insertAtNode(node->left, depth + 1, point, id);
            }
            else
            {
                insertAtNode(node->right, depth + 1, point, id);
            }
        }
    }

    /**
     * @brief Recursively search for points within distanceTol of target
     * 
     * @param node The current node
     * @param target The target point
     * @param depth The current depth (determines split axis)
     * @param distanceTol The maximum distance
     * @param ids Output vector of matching point indices
     */
    void searchFromNode(KdTreeNode<PointT, Scalar>* node, const PointT& target, uint depth, Scalar distanceTol, std::vector<int>& ids) const {
        if (node != nullptr) {
            // Check if current node is within distance
            Scalar dx = node->point.x - target.x;
            Scalar dy = node->point.y - target.y;
            Scalar dz = node->point.z - target.z;
            Scalar distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (distance <= distanceTol)
            {
                ids.push_back(node->id);
            }

            // Determine which subtrees to search
            const uint current_axis = depth % 3;
            const Scalar target_val = getAxisValue(target, current_axis);
            const Scalar node_val = getAxisValue(node->point, current_axis);

            // Always search the "near" subtree
            if (current_axis == 0)
            {
                if ((target_val - distanceTol) < node_val)
                {
                    searchFromNode(node->left, target, depth + 1, distanceTol, ids);
                }
                if ((target_val + distanceTol) > node_val)
                {
                    searchFromNode(node->right, target, depth + 1, distanceTol, ids);
                }
            }
            else if (current_axis == 1)
            {
                if ((target_val - distanceTol) < node_val) {
                    searchFromNode(node->left, target, depth + 1, distanceTol, ids);
                }
                if ((target_val + distanceTol) > node_val) {
                    searchFromNode(node->right, target, depth + 1, distanceTol, ids);
                }
            }
            else
            { // current_axis == 2
                if ((target_val - distanceTol) < node_val) {
                    searchFromNode(node->left, target, depth + 1, distanceTol, ids);
                }
                if ((target_val + distanceTol) > node_val) {
                    searchFromNode(node->right, target, depth + 1, distanceTol, ids);
                }
            }
        }
    }

    /**
     * @brief Get the value of a specific axis from a point
     * 
     * @param point The point
     * @param axis The axis (0 = x, 1 = y, 2 = z)
     * @return The value at the specified axis
     */
    static Scalar getAxisValue(const PointT& point, uint axis)
    {
        switch (axis)
        {
            case 0:
            {
                return static_cast<Scalar>(point.x);
            } 
            case 1:
            {
                return static_cast<Scalar>(point.y);
            }
            case 2:
            {
                return static_cast<Scalar>(point.z);
            }
            default:
            {
                return static_cast<Scalar>(0.0);
            }
        }
    }
};

/**
 * @brief Recursively aggregate points into a cluster using proximity search
 * 
 * @tparam PointT PCL point type
 * @tparam Scalar Type for distance calculations
 * @param index Current point index
 * @param isPointProcessed Vector tracking which points have been processed
 * @param cluster Output vector of point indices in this cluster
 * @param cloud The input point cloud
 * @param tree The KD-Tree for nearest neighbor search
 * @param distanceTol Maximum distance for points to be in the same cluster
 */
template<typename PointT, typename Scalar = float>
void proximityBasedClusterAggregation(
    int index,
    std::vector<bool>& isPointProcessed,
    std::vector<int>& cluster,
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const KdTree<PointT, Scalar>& tree,
    Scalar distanceTol)
{
    isPointProcessed[index] = true;
    cluster.push_back(index);

    // Find all nearby points
    std::vector<int> nearby_points = tree.search(cloud->points[index], distanceTol);

    for (int nearby_point_index : nearby_points)
    {
        if (!isPointProcessed[nearby_point_index])
        {
            proximityBasedClusterAggregation(nearby_point_index, isPointProcessed, cluster, cloud, tree, distanceTol);
        }
    }
}

/**
 * @brief Perform Euclidean clustering on a point cloud
 * 
 * Uses a KD-Tree for efficient nearest neighbor search and recursively
 * aggregates points into clusters based on distance threshold.
 * 
 * @tparam PointT PCL point type (XYZ, XYZI, XYZRGB, etc.)
 * @tparam Scalar Type for distance calculations (typically float)
 * @param cloud The input point cloud to cluster
 * @param tree The KD-Tree built from the point cloud
 * @param distanceTol Maximum distance for points to be in the same cluster
 * @return Vector of clusters, where each cluster is a vector of point indices
 */
template<typename PointT, typename Scalar = float>
std::vector<std::vector<int>> euclideanCluster(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const KdTree<PointT, Scalar>& tree,
    Scalar distanceTol)
{
    std::vector<std::vector<int>> clusters;
    const int points_count = static_cast<int>(cloud->points.size());
    std::vector<bool> isPointProcessed(points_count, false);

    for (int index = 0; index < points_count; ++index)
    {
        if (!isPointProcessed[index])
        {
            std::vector<int> cluster{};
            cluster.reserve(points_count); // Reserve for worst case
            proximityBasedClusterAggregation(index, isPointProcessed, cluster, cloud, tree, distanceTol);
            clusters.push_back(std::move(cluster));
        }
    }

    return clusters;
}

/**
 * @brief Convenience function to perform complete Euclidean clustering
 * 
 * Builds the KD-Tree and performs clustering in one call.
 * 
 * @tparam PointT PCL point type
 * @tparam Scalar Type for distance calculations
 * @param cloud The input point cloud to cluster
 * @param distanceTol Maximum distance for points to be in the same cluster
 * @return Vector of clusters, where each cluster is a vector of point indices
 */
template<typename PointT, typename Scalar = float>
std::vector<std::vector<int>> clusterPointCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    Scalar distanceTol)
{
    KdTree<PointT, Scalar> tree{};
    
    // Build KD-Tree
    const int cloud_size = static_cast<int>(cloud->points.size());
    for (int i = 0; i < cloud_size; ++i)
    {
        tree.insert(cloud->points[i], i);
    }
    
    // Perform clustering
    return euclideanCluster<PointT, Scalar>(cloud, tree, distanceTol);
}

} // namespace clustering

#endif // CLUSTER_H
