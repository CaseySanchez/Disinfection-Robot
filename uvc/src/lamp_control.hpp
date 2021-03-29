#include <wiringPi.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/set_lamp.h"

#include "enable_control.hpp"

class LampControl : public EnableControl
{
    ros::ServiceServer m_set_lamp_service;

public:
    LampControl(ros::NodeHandle &node_handle);

    bool onSetLamp(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response);

    void onEnable() override;
    void onDisable() override;
};
