/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
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
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	std::unordered_set<int> inliersResult;

	std::size_t total_points = cloud->points.size();

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> uni(0, total_points - 1);

	// TODO: Fill in this function

	for (int i = 1; i <= maxIterations; i++)
	{
		std::unordered_set<int> inliersTemp;
		int rand1 = uni(rng);
		int rand2 = uni(rng);

		if (rand1 == rand2 && rand1 != total_points - 1)
		{
			rand2++;
		}
		std::cerr << "Rands : " << rand1 << " " << rand2 << "\n";
		const pcl::PointXYZ &random_point1 = cloud->points.at(rand1);
		const pcl::PointXYZ &random_point2 = cloud->points.at(rand2);

		double x1 = random_point1.x;
		double y1 = random_point1.y;

		double x2 = random_point2.x;
		double y2 = random_point2.y;

		double A = y1 - y2;
		double B = x2 - x1;
		double C = (x1 * y2) - (x2 * y1);

		for (int i = 0; i < total_points; i++)
		{
			const auto &point = cloud->points.at(i);
			double distance = fabs(((A * point.x) + (B * point.y) + C)) / (sqrt((A * A) + (B * B)));

			std::cerr << "Distance : " << distance << "\n";
			if (distance <= distanceTol)
			{
				inliersTemp.insert(i);
			}
		}

		if (inliersTemp.size() > inliersResult.size())
		{
			inliersResult = inliersTemp;
		}
	}

	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	std::size_t total_points = cloud->points.size();

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> uni(0, total_points - 1);

	for (int i = 1; i <= maxIterations; i++)
	{
		std::unordered_set<int> inliersTemp;
		int rand1 = uni(rng);
		int rand2 = uni(rng);
		int rand3 = uni(rng);

		const auto &point1 = cloud->points.at(rand1);
		const auto &point2 = cloud->points.at(rand2);
		const auto &point3 = cloud->points.at(rand3);

		double x1 = point1.x;
		double y1 = point1.y;
		double z1 = point1.z;

		double x2 = point2.x;
		double y2 = point2.y;
		double z2 = point2.z;

		double x3 = point3.x;
		double y3 = point3.y;
		double z3 = point3.z;

		double A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		double B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		double C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		double D = -1 * ((A * x1) + (B * y1) + (C * z1));

		for (int i = 0; i < total_points; i++)
		{
			const auto &point = cloud->points.at(i);
			double x = point.x;
			double y = point.y;
			double z = point.z;

			double distance = fabs((A * x) + (B * y) + (C * z) + D) / sqrt((A * A) + (B * B) + (C * C));

			if (distance <= distanceTol)
			{
				inliersTemp.insert(i);
			}
		}

		if (inliersTemp.size() > inliersResult.size())
		{
			inliersResult = inliersTemp;
		}
	}

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// DONE: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
