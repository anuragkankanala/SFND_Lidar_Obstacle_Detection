// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
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

    //Convert raw input cloud to voxels
    typename pcl::PointCloud<PointT>::Ptr voxelizedCloud(new typename pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize(filterRes, filterRes, filterRes);
    voxel_grid_filter.filter(*voxelizedCloud);

    //Apply CropBox with given points
    typename pcl::PointCloud<PointT>::Ptr cropBoxedCloud(new typename pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(voxelizedCloud);
    cropBoxFilter.filter(*cropBoxedCloud);

    //Identify roof points
    std::vector<int> roofCloudIndices;
    pcl::CropBox<PointT> roofBoxFilter(true);
    roofBoxFilter.setMin(Eigen::Vector4f(-1.5, -1.5, -1.0, 1.0));
    roofBoxFilter.setMax(Eigen::Vector4f(2.5, 1.5, -0.5, 1.0));
    roofBoxFilter.setInputCloud(cropBoxedCloud);
    roofBoxFilter.filter(roofCloudIndices);

    pcl::PointIndices ::Ptr roofPoints(new pcl::PointIndices);
    for (int index : roofCloudIndices)
    {
        roofPoints->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cropBoxedCloud);
    extract.setIndices(roofPoints);
    extract.setNegative(true);
    extract.filter(*cropBoxedCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropBoxedCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // DONE: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

    for (auto index : inliers->indices)
    {
        plane_cloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // DONE:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "No planar model found \n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // DONE:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
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

//Custom Segment Plane using RansacPlane.
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inlier_indices = RansacPlane(cloud, maxIterations, distanceThreshold);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    for (int index : inlier_indices)
    {
        inliers->indices.push_back(index);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    return segResult;
}

//Custom Ransac3D.
template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

template <typename PointT>
void ProcessPointClouds<PointT>::Proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed_indices, std::shared_ptr<KdTree> tree, float distanceTol)
{
    processed_indices[index] = true;
    cluster.push_back(index);

    std::vector<float> point{cloud->points[index].x, cloud->points[index].y};
    std::vector<int> nearest_points = tree->search(point, distanceTol);
    for (auto i : nearest_points)
    {
        if (!processed_indices[i])
        {
            Proximity(i, cloud, cluster, processed_indices, tree, distanceTol);
        }
    }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, std::shared_ptr<KdTree> tree, float distanceTol)
{

    // DONE: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed_indices(cloud->points.size(), false);

    int index = 0;

    while (index < cloud->points.size())
    {
        if (!processed_indices[index])
        {
            std::vector<int> cluster;
            Proximity(index, cloud, cluster, processed_indices, tree, distanceTol);
            clusters.push_back(cluster);
        }
        index++;
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    std::shared_ptr<KdTree> tree(new KdTree());

    //Populate KDTree
    for (int i = 0; i < cloud->points.size(); i++)
    {
        tree->insert({cloud->points[i].x, cloud->points[i].y}, i);
    }

    std::vector<std::vector<int>> clusters_indices_list = euclideanCluster(cloud, tree, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (auto &cluster_indices : clusters_indices_list)
    {
        if (cluster_indices.size() >= minSize && cluster_indices.size() <= maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            for (int index : cluster_indices)
            {
                cluster->points.push_back(cloud->points[index]);
            }
            clusters.push_back(cluster);
        }
    }

    return clusters;
}