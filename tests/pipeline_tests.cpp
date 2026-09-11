#include "../src/processPointClouds.cpp"
#include <set>

void require(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
int main() {
    try {
        std::mt19937 random(7);
        std::uniform_real_distribution<float> coordinate(-10,10);
        std::vector<std::array<float,3>> points;
        for (int i=0;i<500;++i) points.push_back({coordinate(random),coordinate(random),coordinate(random)});
        points.push_back({0,0,0}); points.push_back({0,0,1}); points.push_back({0,0,1});
        KdTree3D tree(points);
        for (int query=0;query<100;++query) {
            const std::array<float,3> target=query==0 ? std::array<float,3>{0,0,0} : points[query];
            const float radius=query==0 ? 1.0f : 2.5f;
            const auto found=tree.search(target,radius);
            std::set<int> expected;
            for (std::size_t i=0;i<points.size();++i) {
                float squared=0;
                for (int axis=0;axis<3;++axis) { float d=points[i][axis]-target[axis]; squared+=d*d; }
                if (squared<=radius*radius) expected.insert(static_cast<int>(i));
            }
            require(std::set<int>(found.begin(),found.end())==expected,"KD-tree differs from brute force");
        }
        require(KdTree3D({}).search({0,0,0},1).empty(),"empty tree");
        ProcessPointClouds<pcl::PointXYZ> processor;
        Cloud::Ptr cloud(new Cloud);
        for(int x=-5;x<=5;++x) for(int y=-5;y<=5;++y)
            cloud->push_back(pcl::PointXYZ(x,y,0.05f*x));
        for(int i=0;i<20;++i) cloud->push_back(pcl::PointXYZ(i*0.1f,0,3));
        const auto separated=processor.SegmentPlane(cloud,150,0.02f);
        require(separated.second->size()==121 && separated.first->size()==20,"RANSAC plane membership");
        Cloud::Ptr empty(new Cloud);
        require(processor.SegmentPlane(empty,10,0.1f).first->empty(),"empty segmentation");
        Cloud::Ptr line(new Cloud);
        for(int i=0;i<10;++i) line->push_back(pcl::PointXYZ(i,0,0));
        require(processor.SegmentPlane(line,10,0.1f).second->empty(),"degenerate samples");
        Cloud::Ptr groups(new Cloud);
        for(int i=0;i<5;++i) groups->push_back(pcl::PointXYZ(i*0.2f,0,0));
        for(int i=0;i<3;++i) groups->push_back(pcl::PointXYZ(i*0.2f,0,2));
        groups->push_back(pcl::PointXYZ(20,20,20));
        const auto clusters=processor.Clustering(groups,0.25f,2,10);
        require(clusters.size()==2 && clusters[0]->size()==5 && clusters[1]->size()==3,"3D flood fill");
        const auto limited=processor.Clustering(groups,0.25f,2,4);
        require(limited.size()==1 && limited[0]->size()==3,"oversized component must not fragment");
        const auto box=processor.BoundingBox(clusters[0]);
        require(box.x_min==0 && std::abs(box.x_max-0.8f)<1e-6f && box.z_max==0,"bounding box");
        std::cout << "All algorithm tests passed\n";
    } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
