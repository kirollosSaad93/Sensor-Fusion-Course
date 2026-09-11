// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr reduced(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(filterRes, filterRes, filterRes);
    voxel.filter(*reduced);

    typename pcl::PointCloud<PointT>::Ptr region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(reduced);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*region);

    pcl::CropBox<PointT> roof;
    roof.setInputCloud(region);
    roof.setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.0f, 1));
    roof.setMax(Eigen::Vector4f(2.6f, 1.7f, -0.4f, 1));
    roof.setNegative(true);
    cloud.reset(new pcl::PointCloud<PointT>);
    roof.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
    std::vector<bool> ground(cloud->size(), false);
    if (inliers)
        for (int id : inliers->indices)
            ground[id] = true;
    for (std::size_t i = 0; i < cloud->size(); ++i)
        if (ground[i]) plane->push_back(cloud->points[i]);
        else obstacles->push_back(cloud->points[i]);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    inliers.reset(new pcl::PointIndices);
    if (cloud->size() >= 3 && distanceThreshold > 0)
    {
        srand(42);
        std::vector<int> candidate;
        candidate.reserve(cloud->size());
        for (int iteration = 0; iteration < maxIterations; ++iteration)
        {
            const int i = rand() % cloud->size();
            int j = rand() % cloud->size();
            while (j == i) j = rand() % cloud->size();
            int k = rand() % cloud->size();
            while (k == i || k == j) k = rand() % cloud->size();
            const PointT& p = cloud->points[i];
            const PointT& q = cloud->points[j];
            const PointT& r = cloud->points[k];
            const double ux = q.x-p.x, uy = q.y-p.y, uz = q.z-p.z;
            const double vx = r.x-p.x, vy = r.y-p.y, vz = r.z-p.z;
            double a = uy*vz-uz*vy, b = uz*vx-ux*vz, c = ux*vy-uy*vx;
            const double norm = std::sqrt(a*a+b*b+c*c);
            if (norm < 1e-9) continue; // Degenerate sample: no unique plane.
            a /= norm;
            b /= norm;
            c /= norm;
            const double d = -(a*p.x+b*p.y+c*p.z);
            candidate.clear();
            for (std::size_t id = 0; id < cloud->size(); ++id)
            {
                const PointT& point = cloud->points[id];
                if (std::abs(a*point.x+b*point.y+c*point.z+d) <= distanceThreshold)
                    candidate.push_back(static_cast<int>(id));
            }
            if (candidate.size() > inliers->indices.size())
                inliers->indices = candidate;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Keep the custom 3D KD-Tree inside this implementation; no header changes.
    struct KdTree
    {
        struct Node { int id, left, right, axis; };
        const pcl::PointCloud<PointT>& points;
        std::vector<Node> nodes;
        int root;

        float coordinate(int id, int axis) const
        {
            return axis == 0 ? points[id].x : (axis == 1 ? points[id].y : points[id].z);
        }

        int build(std::vector<int>& ids, int first, int last, int depth)
        {
            if (first == last) return -1;
            const int axis = depth % 3;
            const int middle = first + (last-first)/2;
            std::nth_element(ids.begin()+first, ids.begin()+middle, ids.begin()+last,
                [&](int a, int b) { return coordinate(a, axis) < coordinate(b, axis); });
            const int node = static_cast<int>(nodes.size());
            nodes.push_back({ids[middle], -1, -1, axis});
            const int left = build(ids, first, middle, depth+1);
            const int right = build(ids, middle+1, last, depth+1);
            nodes[node].left = left;
            nodes[node].right = right;
            return node;
        }

        explicit KdTree(const pcl::PointCloud<PointT>& cloud) : points(cloud)
        {
            std::vector<int> ids(points.size());
            for (std::size_t i = 0; i < ids.size(); ++i) ids[i] = static_cast<int>(i);
            nodes.reserve(points.size());
            root = build(ids, 0, static_cast<int>(ids.size()), 0);
        }

        void search(int nodeId, int target, float radius, float squaredRadius,
                    std::vector<int>& neighbors) const
        {
            if (nodeId < 0) return;
            const Node& node = nodes[nodeId];
            const PointT& p = points[node.id];
            const PointT& q = points[target];
            const float dx = p.x-q.x, dy = p.y-q.y, dz = p.z-q.z;
            if (dx*dx+dy*dy+dz*dz <= squaredRadius) neighbors.push_back(node.id);
            const float delta = coordinate(target, node.axis)-coordinate(node.id, node.axis);
            if (delta <= radius) search(node.left, target, radius, squaredRadius, neighbors);
            if (delta >= -radius) search(node.right, target, radius, squaredRadius, neighbors);
        }
    };

    if (clusterTolerance > 0 && minSize > 0 && maxSize >= minSize)
    {
        KdTree tree(*cloud);
        std::vector<bool> visited(cloud->size(), false);
        std::vector<int> component, neighbors;
        const float squaredTolerance = clusterTolerance*clusterTolerance;
        for (std::size_t seed = 0; seed < cloud->size(); ++seed)
        {
            if (visited[seed]) continue;
            component.clear();
            component.push_back(static_cast<int>(seed));
            visited[seed] = true;
            // Iterative flood fill avoids a deep call stack for large components.
            for (std::size_t next = 0; next < component.size(); ++next)
            {
                neighbors.clear();
                tree.search(tree.root, component[next], clusterTolerance, squaredTolerance, neighbors);
                for (int neighbor : neighbors)
                    if (!visited[neighbor])
                    {
                        visited[neighbor] = true;
                        component.push_back(neighbor);
                    }
            }
            // Visit the entire component before applying size limits.
            if (component.size() < static_cast<std::size_t>(minSize) ||
                component.size() > static_cast<std::size_t>(maxSize)) continue;
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            for (int id : component) cluster->push_back(cloud->points[id]);
            clusters.push_back(cluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}