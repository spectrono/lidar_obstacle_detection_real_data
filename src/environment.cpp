/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <utility>
#include <filesystem>


void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* point_cloud_processor_p,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display city Block     -----
    // ----------------------------------------------------

    using point_type = pcl::PointXYZI;
    using point_cloud_type = pcl::PointCloud<point_type>::Ptr;
    
    point_cloud_type filtered_cloud = point_cloud_processor_p->FilterCloud(input_cloud, 0.2, Eigen::Vector4f(-5.0f,-7.0f,-2.8f, 1.0f), Eigen::Vector4f(25.0f, 7.0f, 1.5f, 1.0f));

    // Remove roof from filtered cloud
    Eigen::Vector4f minPointRoof(-1.52f, -1.81f, -1.03f, 1.0f);
    Eigen::Vector4f maxPointRoof( 2.63f,  1.81f, -0.20f, 1.0f);
    point_cloud_type filtered_cloud_without_roof = point_cloud_processor_p->applyBoxCropOnCloud(filtered_cloud, minPointRoof, maxPointRoof, true);

    // Segment point cloud in ground plane and obstacles
    using segmentation_result_type = std::pair<point_cloud_type, point_cloud_type>;   
    segmentation_result_type plane_segmentation_result_0 = point_cloud_processor_p->SegmentPlane(filtered_cloud_without_roof, 200, 0.23f);

    Color ground_plane_color{0.0f, 1.0f, 0.0f};
    renderPointCloud(viewer, plane_segmentation_result_0.second, "Plane Cloud", ground_plane_color);

    std::vector<point_cloud_type> cloud_clusters = point_cloud_processor_p->Clustering(plane_segmentation_result_0.first, 1.0, 20, 1000);

    int cluster_id = 0;
    const std::vector<Color> obstacle_colors = {Color(1.0f, 0.0f, 0.0f), Color(1.0f, 1.0f, 0.0f), Color(0.0f, 0.5f, 1.0f), Color(0.0f, 0.0f, 1.0f)};
    const int obstacle_colors_count{static_cast<int>(obstacle_colors.size())};
    for (const auto& cluster : cloud_clusters)
    {
        std::cout << "cluster size: ";
        
        // Render cluster's points
        point_cloud_processor_p->numPoints(cluster);
        renderPointCloud(viewer, cluster, "cloud_cluster_" + std::to_string(cluster_id), obstacle_colors[cluster_id % obstacle_colors_count]);

        // Render cluster's bounding box
        Box bbox = point_cloud_processor_p->BoundingBox(cluster);
        renderBox(viewer, bbox, cluster_id);

        ++cluster_id;
    }

    // Render roof crop area
    const int roof_cluster_id{cluster_id};
    const Box roof_bbox{minPointRoof[0], minPointRoof[1], minPointRoof[2],
                       maxPointRoof[0], maxPointRoof[1], maxPointRoof[2]};
    renderBox(viewer, roof_bbox, roof_cluster_id);
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case CameraAngle::XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case CameraAngle::TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case CameraAngle::Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case CameraAngle::FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle != CameraAngle::FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = CameraAngle::XY;
    initCamera(setAngle, viewer);

    // Real data
    using point_type = pcl::PointXYZI;
    using point_cloud_type = pcl::PointCloud<point_type>::Ptr;

    ProcessPointClouds<point_type>* point_cloud_processor_p = new ProcessPointClouds<point_type>{};
    
    std::vector<std::filesystem::path> filename_stream = point_cloud_processor_p->streamPcd("../data/pcd/data_1");
    auto filename_iterator = filename_stream.begin();
    point_cloud_type input_cloud;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        point_cloud_type input_cloud = point_cloud_processor_p->loadPcd(filename_iterator->string());
        cityBlock(viewer, point_cloud_processor_p, input_cloud);

        ++filename_iterator;
        if (filename_iterator == filename_stream.end())
        {
            filename_iterator = filename_stream.begin();
        }

        viewer->spinOnce ();
    }
}