#include <wiringPi.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/set_lamp.h"

class LampNode : public ros::NodeHandle
{
    ros::ServiceServer m_set_lamp_service;

public:
    LampNode();

    bool onSetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response);
};
