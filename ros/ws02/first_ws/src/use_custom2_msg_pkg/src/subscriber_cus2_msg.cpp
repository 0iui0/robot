//
// Created by zcb on 5/3/21.
//

#include <iostream>
#include <ros/ros.h>
#include <custom2_msgs/Team.h>


void topicCallback(const custom2_msgs::Team::ConstPtr &msg) {
    std::cout << msg->name << " || " << msg->leader.name <<" " << msg->leader.age << std::endl;
}

int main(int argc, char **argv) {

    std::string nodeName = "subscriber_node";
    ros::init(argc, argv, nodeName);
    ros::NodeHandle node;

    std::string topicName = "/zcb01/topic";
    ros::Subscriber sub = node.subscribe(topicName, 1000, topicCallback);

    ros::spin();
}



