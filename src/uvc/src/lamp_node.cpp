#include "ros/ros.h"
#include "ros/console.h"

#include "lamp_node.hpp"

LampNode::LampNode() : ros::NodeHandle("~")
{
    wiringPiSetup();
    
    pinMode(8, OUTPUT);

    m_set_lamp_service = advertiseService("set_lamp", &LampNode::onSetLamp, this);
}

bool LampNode::onSetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response)
{
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lamp_node");

    LampNode lamp_node;
    
    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}
