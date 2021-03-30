#include <wiringPi.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/set_lamp.h"

class LampNode : public ros::NodeHandle
{
    bool m_active;

    ros::ServiceServer m_get_lamp_service;
    ros::ServiceServer m_set_lamp_service;

public:
    LampNode();

    bool onGetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response);
    bool onSetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response);
};
