#include "ros/ros.h"
#include "ros/console.h"

#include "system_control.hpp"

SystemControl::SystemControl(ros::NodeHandle &node_handle)
{
    m_service_server = node_handle.advertiseService("set_enable", &SystemControl::onSetEnable, this);

    m_publisher = node_handle.advertise<uvc::enable>("enable", 100);
}

bool SystemControl::onSetEnable(uvc::set_enable::Request &request, uvc::set_enable::Response &response)
{
    uvc::enable message;

    message.enable = request.enable;

    m_publisher.publish(message);

    return true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "system_control");

    ros::NodeHandle node_handle;

    SystemControl system_control(node_handle);

    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}