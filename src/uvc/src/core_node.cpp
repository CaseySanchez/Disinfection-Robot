#include "core_node.hpp"

CoreNode::CoreNode() : ros::NodeHandle("~")
{
    std::map<std::string, std::shared_ptr<State>> state_map = { { "idle", std::make_shared<IdleState>() }, { "manual", std::make_shared<ManualState>() }, { "auto", std::make_shared<AutoState>() } };

    m_state_machine = StateMachine(state_map, "idle");

    m_get_state_service = advertiseService("get_state", &CoreNode::onGetState, this);
    m_set_state_service = advertiseService("set_state", &CoreNode::onSetState, this);
}

bool CoreNode::onGetState(uvc::get_state::Request &request, uvc::get_state::Response &response)
{
    ROS_INFO("GET");

    int32_t state = static_cast<int32_t>(m_state);

    response.state = state;

    return true;
}

bool CoreNode::onSetState(uvc::set_state::Request &request, uvc::set_state::Response &response)
{
    ROS_INFO("SET");

    StateType state = static_cast<StateType>(request.state);

    m_state = state;

    switch (state) {
    case IDLE:
        m_state_machine.transition("idle");
        
        break;

    case MANUAL:
        m_state_machine.transition("manual");

        break;
    
    case AUTO:
        m_state_machine.transition("auto");

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
        core_node.update();

        ros::spinOnce();
    }

    return 0;
}
