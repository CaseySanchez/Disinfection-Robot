#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/set_enable.h"
#include "uvc/set_lamp.h"

#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uvc_client");
   
    ros::NodeHandle node_handle;

    ros::ServiceClient service_client_set_enable = node_handle.serviceClient<uvc::set_enable>("set_enable");

    uvc::set_enable srv_set_enable;

    srv_set_enable.request.enable = false;

    service_client_set_enable.call(srv_set_enable);

    ros::ServiceClient service_client_set_lamp = node_handle.serviceClient<uvc::set_lamp>("set_lamp");
    
    uvc::set_lamp srv_set_lamp;

    srv_set_lamp.request.active = false;

    service_client_set_lamp.call(srv_set_lamp);

    ROS_INFO("OFF");

    std::this_thread::sleep_for(std::chrono::seconds(10));

    srv_set_lamp.request.active = true;
    
    service_client_set_lamp.call(srv_set_lamp);
    
    ROS_INFO("ON");

    std::this_thread::sleep_for(std::chrono::seconds(10));

    srv_set_lamp.request.active = false;

    service_client_set_lamp.call(srv_set_lamp);

    ROS_INFO("OFF");

    return 0;
}
