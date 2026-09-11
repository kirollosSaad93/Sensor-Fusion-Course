#include "render/render.h"
#include "processPointClouds.cpp"


int main(int argc, char** argv) {
    try {
        bool headless=false;
        std::string dataPath=DEFAULT_DATA_PATH;
        for (int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if (arg=="--headless") headless=true;
            else if (arg=="--help") {
                std::cout << "Usage: environment [--headless] [PCD directory]\n";
                return 0;
            } else dataPath=arg;
        }
        ProcessPointClouds<pcl::PointXYZI> processor;
        const auto files=processor.streamPcd(dataPath);
        if (files.empty()) throw std::runtime_error("No PCD files in " + dataPath);
        pcl::visualization::PCLVisualizer::Ptr viewer;
        if (!headless) {
            viewer.reset(new pcl::visualization::PCLVisualizer("Obstacle detection"));
            viewer->setBackgroundColor(0,0,0);
            viewer->initCameraParameters();
            viewer->setCameraPosition(-16,-16,16,1,1,0);
        }
        std::size_t frame=0;
        do {
            const auto cloud=processor.loadPcd(files[frame].string());
            if (cloud->empty()) throw std::runtime_error("Empty input frame");
            const auto filtered=processor.FilterCloud(cloud,0.2f,
                Eigen::Vector4f(-10,-6,-3,1),Eigen::Vector4f(30,7,2,1));
            const auto segmented=processor.SegmentPlane(filtered,150,0.2f);
            const auto clusters=processor.Clustering(segmented.first,0.45f,10,1500);
            if (segmented.second->empty() || clusters.empty())
                throw std::runtime_error("No road or obstacles detected in " + files[frame].string());
            if (viewer) {
                viewer->removeAllPointClouds(); viewer->removeAllShapes();
                renderPointCloud(viewer,segmented.second,"road",Color(0,1,0));
                renderPointCloud(viewer,segmented.first,"obstacles",Color(1,0,0));
            }
            std::cout << "FRAME " << frame << " road=" << segmented.second->size()
                      << " obstacles=" << segmented.first->size() << " boxes=" << clusters.size() << '\n';
            int id=0;
            for (const auto& cluster : clusters) {
                const Box box=processor.BoundingBox(cluster);
                std::cout << "BOX " << id << ' ' << cluster->size() << ' '
                    << box.x_min << ' ' << box.y_min << ' ' << box.z_min << ' '
                    << box.x_max << ' ' << box.y_max << ' ' << box.z_max << '\n';
                if (viewer) renderBox(viewer,box,id,Color(1,1,0));
                ++id;
            }
            if (viewer) viewer->spinOnce(100);
            ++frame;
            if (headless && frame==files.size()) break;
            frame%=files.size();
        } while (!viewer || !viewer->wasStopped());
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
