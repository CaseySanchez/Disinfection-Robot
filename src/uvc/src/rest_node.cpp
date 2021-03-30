#include "ros/ros.h"
#include "ros/console.h"

#include "rest_node.hpp"

#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rest_node");

    ros::NodeHandle node_handle;

    auto set_lamp_client = node_handle.serviceClient<uvc::set_lamp>("lamp_node/set_lamp");

    uvc::set_lamp set_lamp;

    set_lamp.request.active = true;

    set_lamp_client.call(set_lamp);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    set_lamp.request.active = false;

    set_lamp_client.call(set_lamp);

    RestNode rest_node;

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
