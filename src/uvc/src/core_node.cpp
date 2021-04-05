#include "ros/ros.h"
#include "ros/console.h"

#include "core_node.hpp"

CoreNode::CoreNode() : ros::NodeHandle("~"), m_state_machine(std::make_shared<IdleState>())
{
    m_get_state_service = advertiseService("get_state", &CoreNode::onGetState, this);
    m_set_state_service = advertiseService("set_state", &CoreNode::onSetState, this);
}

bool CoreNode::onGetState(uvc::get_state::Request &request, uvc::get_state::Response &response)
{
    int32_t state = static_cast<int32_t>(m_state);

    response.state = state;

    return true;
}

bool CoreNode::onSetState(uvc::set_state::Request &request, uvc::set_state::Response &response)
{
    StateType state = static_cast<StateType>(request.state);

    m_state = state;

    switch (state) {
    case IDLE:
        m_state_machine.setState(std::make_shared<IdleState>());
        
        break;

    case MANUAL:
        m_state_machine.setState(std::make_shared<ManualState>());

        break;
    
    case AUTO:
        m_state_machine.setState(std::make_shared<AutoState>());

        break;

    default:
        return false;
    }

    return true;
}

void CoreNode::update()
{
    m_state_machine.update();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core_node");

    CoreNode core_node;
    
    while (ros::ok()) {
        ros::spinOnce();

        core_node.update();
    }

    return 0;
}
