#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {
    // Load input point cloud from PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);

    // Compute surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    // Compute ISS keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZRGB> iss;
    iss.setInputCloud(cloud);
    iss.setNormals(normals);
    iss.setSalientRadius(6 * ne.getRadiusSearch());
    iss.setNonMaxRadius(4 * ne.getRadiusSearch());
    iss.setThreshold21(0.975);
    iss.setThreshold32(0.975);
    iss.setMinNeighbors(5);
    iss.compute(*keypoints);
    std::cout << keypoints->size() << std::endl;

    // Visualize keypoints
    pcl::visualization::PCLVisualizer viewer("Keypoints");
    viewer.addPointCloud(cloud, "cloud");
    viewer.addPointCloud(keypoints, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,  0, 0, 1, "keypoints");

    viewer.spin();

    return 0;
}
