#pragma once

#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/get_mode.h"
#include "uvc/set_mode.h"

class CoreNode : public ros::NodeHandle
{
    enum ModeType : int32_t
    {
        IDLE = 0,
        MANUAL,
        AUTO
    };

    ModeType m_mode;

    std::unique_ptr<Controller> m_controller;

    ros::ServiceServer m_get_mode_service;
    ros::ServiceServer m_set_mode_service;

public:
    CoreNode();

    bool onGetMode(uvc::get_mode::Request &request, uvc::get_mode::Response &response);
    bool onSetMode(uvc::set_mode::Request &request, uvc::set_mode::Response &response);

    void update();
};
