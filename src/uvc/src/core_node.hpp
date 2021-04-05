#pragma once

#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/get_state.h"
#include "uvc/set_state.h"

#include "state_machine.hpp"
#include "core_states.hpp"

class CoreNode : public ros::NodeHandle
{
    enum StateType : int32_t
    {
        IDLE = 0,
        MANUAL,
        AUTO
    };

    StateType m_state;

    StateMachine m_state_machine;

    ros::ServiceServer m_get_state_service;
    ros::ServiceServer m_set_state_service;

public:
    CoreNode();

    bool onGetState(uvc::get_state::Request &request, uvc::get_state::Response &response);
    bool onSetState(uvc::set_state::Request &request, uvc::set_state::Response &response);

    void update();
};
