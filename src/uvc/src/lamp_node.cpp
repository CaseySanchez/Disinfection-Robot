#include "ros/ros.h"
#include "ros/console.h"

#include "lamp_node.hpp"

LampNode::LampNode() : ros::NodeHandle("~"), m_active(false)
{
    wiringPiSetup();
    
    pinMode(8, OUTPUT);

    m_get_lamp_service = advertiseService("get_lamp", std::bind(&LampNode::onGetLamp, this, std::placeholders::_1, std::placeholders::_2));
    m_set_lamp_service = advertiseService("set_lamp", std::bind(&LampNode::onSetLamp, this, std::placeholders::_1, std::placeholders::_2));
}

bool LampNode::onGetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response)
{
    response.active = m_active;

    return true;
}

bool LampNode::onSetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response)
{
    m_active = request.active;

    if (m_active) {
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
        ros::spinOnce();
    }

    return 0;
}
