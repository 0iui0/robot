//
// Created by zcb on 5/4/21.
//

#include <ros/ros.h>
#include <iostream>
#include <custom_srvs/NumOption.h>


int main(int argc, char **argv) {

    std::string nodeName = "client_node";
    ros::init(argc, argv, nodeName);

    ros::NodeHandle node;

    std::string serviceName = "/zcb01/service";
    ros::ServiceClient client = node.serviceClient<custom_srvs::NumOptionRequest, custom_srvs::NumOptionResponse>(
            serviceName);
    client.waitForExistence();


    custom_srvs::NumOption service;
    service.request.a = 10;
    service.request.b = 13;
    service.request.op = "*";
    if (client.call(service)) {
        ROS_INFO_STREAM("call success,res is " << service.response.res);
    } else {
        ROS_INFO_STREAM("call failed!");
    }


    return 0;
}





