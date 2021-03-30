#include "ros/ros.h"
#include "ros/console.h"

#include "lamp_control.hpp"

LampControl::LampControl(ros::NodeHandle &node_handle) : EnableControl(node_handle)
{
    wiringPiSetup();
    
    pinMode(8, OUTPUT);

    m_set_lamp_service = node_handle.advertiseService("set_lamp", &LampControl::onSetLamp, this);
}

bool LampControl::onSetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response)
{
    if (getEnable()) {
        if (request.active) {
            ROS_INFO("ON");

            digitalWrite(8, 1);
        }
        else {
            ROS_INFO("OFF");

            digitalWrite(8, 0);
        }

        return true;
    }

    return false;
}

void LampControl::onEnable()
{
    digitalWrite(8, 0);
}

void LampControl::onDisable()
{
    digitalWrite(8, 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lamp_control");

    ros::NodeHandle node_handle;

    LampControl lamp_control(node_handle);
    
    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}
