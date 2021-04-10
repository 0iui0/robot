//
// Created by zcb on 2021/4/9.
//


#include <ros/ros.h>
#include <iostream>
#include <demo_srvs/GetStudentId.h>

using namespace std;

bool client_callback(demo_srvs::GetStudentIdRequest &request, demo_srvs::GetStudentIdResponse &response) {
    string name = request.stu.name;
    int age = request.stu.age;

    response.id = name + to_string(age);

    return true;
}


int main(int argc, char **argv) {

    string nodeName = "cpp_server_node";
    ros::init(argc, argv, nodeName);
    ros::NodeHandle node;

    string serviceName = "/cpp/service";
    const ros::ServiceServer &server = node.advertiseService(serviceName, client_callback);

    ros::spin();

    return 0;
}








