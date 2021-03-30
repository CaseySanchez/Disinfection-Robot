#include "ros/ros.h"
#include "ros/console.h"

#include "rest_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rest_node");
   
    RestNode rest_node;

    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}
