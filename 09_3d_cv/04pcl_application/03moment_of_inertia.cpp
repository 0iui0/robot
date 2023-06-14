//
// Created by zcb on 6/14/21.
//
/*
 * 3d 包容盒
 * http://robot.czxy.com/docs/pcl/chapter04/bounding_volumes/#3d
 */

#include <vector>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include "bounding_box.h"
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    if (argc != 2)
        return (0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
        return (-1);
//    pcl::io::savePCDFileASCII("~/robot/09_3d_cv/04pcl_application/test_pcd.pcd", *cloud);
    BoundingBox box = BoundingBox::extract(cloud, BoundingBox::Type::OBB);
    BoundingBox box2 = BoundingBox::extract(cloud, BoundingBox::Type::AABB);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    // 添加AABB包容盒
    viewer->addCube(box2.min.x(), box2.max.x(), box2.min.y(),
                    box2.max.y(), box2.min.z(), box2.max.z(), 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    // 添加OBB包容盒
    Eigen::Quaternionf q(box.world_R_center);
    viewer->addCube(box.world_P_center, q, box.max.x() - box.min.x(), box.max.y() - box.min.y(), box.max.z() - box.min.z(), "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");


    // 创建惯性矩估算对象，设置输入点云，并进行计算
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    // 获取主轴major_vector，中轴middle_vector，辅助轴minor_vector
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    pcl::PointXYZ center(box.world_P_center(0), box.world_P_center(1), box.world_P_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + box.world_P_center(0), major_vector(1) + box.world_P_center(1),
                         major_vector(2) + box.world_P_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + box.world_P_center(0), middle_vector(1) + box.world_P_center(1),
                         middle_vector(2) + box.world_P_center(2));
    pcl::PointXYZ z_axis(minor_vector(0) + box.world_P_center(0), minor_vector(1) + box.world_P_center(1),
                         minor_vector(2) + box.world_P_center(2));
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    viewer->addLine(pcl::PointXYZ(box2.world_P_center.x(),box2.world_P_center.y(),box2.world_P_center.z()),x_axis, 1.0f, 0.0f, 0.0f);
    viewer->addLine(pcl::PointXYZ(box2.world_P_center.x(),box2.world_P_center.y(),box2.world_P_center.z()),y_axis, 0.0f, 1.0f, 0.0f);
    viewer->addLine(pcl::PointXYZ(box2.world_P_center.x(),box2.world_P_center.y(),box2.world_P_center.z()),z_axis, 0.0f, 0.0f, 1.0f);
    while (!viewer->wasStopped()) {
        viewer->spin();
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}