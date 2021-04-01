#include "ros/ros.h"
#include "ros/console.h"

#include "core_node.hpp"

CoreNode::CoreNode() : ros::NodeHandle("~"), m_mode(0)
{
    m_get_mode_service = advertiseService("get_mode", &CoreNode::onGetMode, this);
    m_set_mode_service = advertiseService("set_mode", &CoreNode::onSetMode, this);
}

bool CoreNode::onGetMode(uvc::get_lamp::Request &request, uvc::get_lamp::Response &response)
{
    int32_t mode = static_cast<int32_t>(m_mode);

    response.mode = mode;

    return true;
}

bool CoreNode::onSetMode(uvc::set_lamp::Request &request, uvc::set_lamp::Response &response)
{
    ModeType mode = static_cast<ModeType>(request.mode);

    switch (mode) {
        case IDLE:
            m_controller.reset();

            m_controller = std::shared_ptr<IdleController>(new IdleController);

            m_mode = IDLE;
        
            break;

        case MANUAL:
            m_controller.reset();

            m_controller = std::shared_ptr<ManualController>(new ManualController);

            m_mode = MANUAL;

            break;

        case AUTO:
            m_controller.reset();

            m_controller = std::shared_ptr<AutoController>(new AutoController);

            m_mode = AUTO;

            break;

        default:
            return false;
    }

    return true;
}

void CoreNode::update()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core_node");

    CoreNode core_node;
    
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}