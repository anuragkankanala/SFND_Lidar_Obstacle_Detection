/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "customProcessPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // DONE:: Create lidar sensor
    std::unique_ptr<Lidar> lidar(new Lidar(cars, 0.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr = lidar->scan();

    // DONE:: Create point processor
    //renderRays(viewer, lidar->position, point_cloud_ptr); //lidar rays
    //renderPointCloud(viewer, point_cloud_ptr, "point_cloud"); //original unsegmented lidar cloud

    std::unique_ptr<ProcessPointClouds<pcl::PointXYZ>> point_cloud_processor(new ProcessPointClouds<pcl::PointXYZ>());

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud = point_cloud_processor->SegmentPlane(point_cloud_ptr, 100, 0.2);

    //renderPointCloud(viewer, segment_cloud.first, "Obstacle Cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segment_cloud.second, "Plane Cloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_cloud_processor->Clustering(segment_cloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1.0, 0.0, 0.0), Color(1.0, 1.0, 0.0), Color(0.0, 0.0, 1.0)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_cloud_processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        auto box = point_cloud_processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, colors[clusterId % colors.size()], 1.0);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointCloudProcessorPtr, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    std::shared_ptr<CustomProcessPointClouds<pcl::PointXYZI>> customPointCloudProcessorPtr(new CustomProcessPointClouds<pcl::PointXYZI>());

    Eigen::Vector4f cropBoxMin(-10, -5.0, -2.0, 1.0);
    Eigen::Vector4f cropBoxMax(35.0, 10.0, 5.0, 1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudProcessorPtr->FilterCloud(inputCloud, 0.3, cropBoxMin, cropBoxMax);
    //renderPointCloud(viewer, filteredCloud, "inputCloud");

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
    // segment_cloud = pointCloudProcessorPtr->SegmentPlane(filteredCloud, 100, 0.2);

    //Use custom Segment Plane algorithm.
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
        segment_cloud = customPointCloudProcessorPtr->SegmentPlane(filteredCloud, 30, 0.3);

    //Render ground plane
    renderPointCloud(viewer, segment_cloud.second, "Plane Cloud", Color(0, 1, 0));

    //Detect clusters
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointCloudProcessorPtr->Clustering(segment_cloud.first, 0.5, 10, 500);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = customPointCloudProcessorPtr->Clustering(segment_cloud.first, 0.3, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //pointCloudProcessorPtr->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[(clusterId % colors.size())]);

        auto box = pointCloudProcessorPtr->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointCloudProcessorPtr(new ProcessPointClouds<pcl::PointXYZI>());

    std::vector<boost::filesystem::path> stream = pointCloudProcessorPtr->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointCloudProcessorPtr->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointCloudProcessorPtr, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}