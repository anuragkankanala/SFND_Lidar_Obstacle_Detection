#include "processPointClouds.h"

template <typename PointT>
struct CustomProcessPointClouds
{
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    //Custom implementation of SegmentPlane using Ransac
    std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    //Custom implementation of euclidean clustering using kdtree
    void Proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed_indices, std::shared_ptr<KdTree> tree, float distanceTol);

    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, std::shared_ptr<KdTree> tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
};

//Custom Segment Plane using RansacPlane.
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
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
std::unordered_set<int> CustomProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
void CustomProcessPointClouds<PointT>::Proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed_indices, std::shared_ptr<KdTree> tree, float distanceTol)
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
std::vector<std::vector<int>> CustomProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, std::shared_ptr<KdTree> tree, float distanceTol)
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
std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
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

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
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
