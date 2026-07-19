/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../render/render.h"
#include <unordered_set>
#include "../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../processPointClouds.cpp"
#include <pcl/filters/random_sample.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	pcl::RandomSample<pcl::PointXYZ> rs;
	rs.setInputCloud(cloud);
	rs.setSample(2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud {new pcl::PointCloud<pcl::PointXYZ>};

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i)
	{
		// Randomly sample subset (2 Points)
		rs.filter(*sampled_cloud);

		const pcl::PointXYZ& p0 = sampled_cloud->points[0];
		const pcl::PointXYZ& p1 = sampled_cloud->points[1];

		// Fit a line
		float A = p0.y - p1.y;
		float B = p1.x - p0.x;
		float C = (p0.x * p1.y) - (p1.x * p0.y);

		// Measure distance between every point and fitted line
		std::unordered_set<int> inliers;

		const float normalization_coefficient = std::sqrt( (A*A) + (B*B));

		for (int index = 0; index < cloud->points.size(); ++index)
		{
			if (inliers.count(index) > 0)
			{
				continue;
			}

    		const pcl::PointXYZ& p_index = cloud->points[index];

			// If distance is smaller than threshold count it as inlier
			float distance = std::abs((A * p_index.x) + (B * p_index.y) + C) / normalization_coefficient;
			if (distance < distanceTol)
			{
				inliers.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	return inliersResult;
}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	pcl::RandomSample<pcl::PointXYZ> rs;
	rs.setInputCloud(cloud);
	rs.setSample(3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud {new pcl::PointCloud<pcl::PointXYZ>};

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i)
	{
		// Randomly sample subset (3 Points)
		rs.filter(*sampled_cloud);

		const pcl::PointXYZ& p0 = sampled_cloud->points[0];
		const pcl::PointXYZ& p1 = sampled_cloud->points[1];
		const pcl::PointXYZ& p2 = sampled_cloud->points[2];

		// Fit a plane
		const Eigen::Vector3f v1 = p1.getVector3fMap() - p0.getVector3fMap();
		const Eigen::Vector3f v2 = p2.getVector3fMap() - p0.getVector3fMap();

		const Eigen::Vector3f v1xv2 = v1.cross(v2);

		const float A = v1xv2.x();
		const float B = v1xv2.y();
		const float C = v1xv2.z();
		const float D = -((A * p0.x) + (B * p0.y) + (C * p0.z));

		// Measure distance between every point and fitted line
		std::unordered_set<int> inliers;

		const float normalization_coefficient = std::sqrt( (A*A) + (B*B) + (C*C));

		for (int index = 0; index < cloud->points.size(); ++index)
		{
			if (inliers.count(index) > 0)
			{
				continue;
			}

    		const pcl::PointXYZ& p_index = cloud->points[index];

			// If distance is smaller than threshold count it as inlier
			float distance = std::abs((A * p_index.x) + (B * p_index.y) + (C * p_index.z) + D) / normalization_coefficient;
			if (distance < distanceTol)
			{
				inliers.insert(index);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 10000, 0.9);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
