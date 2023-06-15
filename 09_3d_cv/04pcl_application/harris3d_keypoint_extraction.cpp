#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>

#include <cstdlib>

using namespace std;

typedef pcl::PointXYZ PointType;

int
main(int argc, char **argv) {
    // Load the RGBD point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud);

    // Convert the RGBD point cloud data to grayscale
    pcl::PointCloud<pcl::PointXYZI>::Ptr gray_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud, *gray_cloud);
    for (size_t i = 0; i < gray_cloud->size(); ++i) {
        gray_cloud->points[i].intensity =
                0.299 * cloud->points[i].r + 0.587 * cloud->points[i].g + 0.114 * cloud->points[i].b;
    }

    // Compute the Harris 3D keypoints
    pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris;
    harris.setInputCloud(gray_cloud);
    harris.setNonMaxSupression(true);
    harris.setRadius(0.5);
    harris.setThreshold(0.01);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    harris.compute(*keypoints);
    std::cout << "keypoints size : " << keypoints->size() << std::endl;

    // Visualize the keypoints in the RGBD point cloud data
    pcl::visualization::PCLVisualizer viewer("Keypoints");
    viewer.addPointCloud(cloud, "cloud");
    viewer.addPointCloud<pcl::PointXYZI>(keypoints, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
    viewer.spin();
}


