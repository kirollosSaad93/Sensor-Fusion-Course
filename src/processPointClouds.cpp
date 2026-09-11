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

    if (filterRes <= 0) throw std::invalid_argument("filter resolution must be positive");
    typename pcl::PointCloud<PointT>::Ptr reduced(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(filterRes,filterRes,filterRes);
    voxel.filter(*reduced);
    typename pcl::PointCloud<PointT>::Ptr region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(reduced); crop.setMin(minPoint); crop.setMax(maxPoint);
    crop.filter(*region);
    pcl::CropBox<PointT> roof;
    roof.setInputCloud(region);
    roof.setMin(Eigen::Vector4f(-1.5f,-1.7f,-1.0f,1));
    roof.setMax(Eigen::Vector4f(2.6f,1.7f,-0.4f,1));
    roof.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    roof.filter(*filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
    std::vector<bool> ground(cloud->size(),false);
    for (int id : inliers->indices) ground.at(id)=true;
    obstacles->reserve(cloud->size()); plane->reserve(inliers->indices.size());
    for (std::size_t i=0;i<cloud->size();++i)
        if (ground[i]) plane->push_back(cloud->points[i]);
        else obstacles->push_back(cloud->points[i]);
    return {obstacles,plane};
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    if (maxIterations <= 0 || distanceThreshold <= 0)
        throw std::invalid_argument("RANSAC requires positive iterations and tolerance");
    if (cloud->size() >= 3) {
        // Repeatable sampling avoids changes when replaying the same frame.
        std::mt19937 random(42);
        std::uniform_int_distribution<int> sample(0,static_cast<int>(cloud->size())-1);
        std::vector<int> candidate; candidate.reserve(cloud->size());
        for (int iteration=0;iteration<maxIterations;++iteration) {
            const int i=sample(random);
            int j=sample(random); while(j==i) j=sample(random);
            int k=sample(random); while(k==i || k==j) k=sample(random);
            const auto& p=cloud->points[i]; const auto& q=cloud->points[j];
            const auto& r=cloud->points[k];
            const double ux=q.x-p.x, uy=q.y-p.y, uz=q.z-p.z;
            const double vx=r.x-p.x, vy=r.y-p.y, vz=r.z-p.z;
            double a=uy*vz-uz*vy, b=uz*vx-ux*vz, c=ux*vy-uy*vx;
            const double norm=std::sqrt(a*a+b*b+c*c);
            if (norm < 1e-9) continue; // duplicate or collinear samples
            a/=norm; b/=norm; c/=norm;
            // The road is approximately horizontal; reject building facades.
            if (std::abs(c) < 0.9) continue;
            const double d=-(a*p.x+b*p.y+c*p.z);
            candidate.clear();
            for (std::size_t id=0;id<cloud->size();++id) {
                const auto& point=cloud->points[id];
                if (std::abs(a*point.x+b*point.y+c*point.z+d) <= distanceThreshold)
                    candidate.push_back(static_cast<int>(id));
            }
            if (candidate.size() > inliers->indices.size()) inliers->indices=candidate;
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

    if (clusterTolerance <= 0 || minSize < 1 || maxSize < minSize)
        throw std::invalid_argument("invalid cluster parameters");
    std::vector<std::array<float,3>> points; points.reserve(cloud->size());
    for (const auto& p : cloud->points) points.push_back({p.x,p.y,p.z});
    KdTree3D tree(points);
    std::vector<bool> visited(points.size(),false);
    std::vector<int> component;
    for (std::size_t seed=0;seed<points.size();++seed) {
        if (visited[seed]) continue;
        component.clear(); component.push_back(static_cast<int>(seed)); visited[seed]=true;
        // Iterative flood fill avoids stack overflow on large connected regions.
        for (std::size_t next=0;next<component.size();++next)
            for (int neighbor : tree.search(points[component[next]],clusterTolerance))
                if (!visited[neighbor]) { visited[neighbor]=true; component.push_back(neighbor); }
        // Fully visit oversized components before rejecting them.
        if (component.size() < static_cast<std::size_t>(minSize) ||
            component.size() > static_cast<std::size_t>(maxSize)) continue;
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        cluster->reserve(component.size());
        for (int id : component) cluster->push_back(cloud->points[id]);
        clusters.push_back(cluster);
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
        throw std::runtime_error("Could not read PCD: " + file);
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths;
    for (const auto& entry : boost::filesystem::directory_iterator(dataPath))
        if (boost::filesystem::is_regular_file(entry.path()) && entry.path().extension()==".pcd")
            paths.push_back(entry.path());

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}