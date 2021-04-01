#include "ros/ros.h"
#include "ros/console.h"

#include "core_node.hpp"

CoreNode::CoreNode() : ros::NodeHandle("~")
{
    setMode(IDLE);

    m_get_mode_service = advertiseService("get_mode", &CoreNode::onGetMode, this);
    m_set_mode_service = advertiseService("set_mode", &CoreNode::onSetMode, this);
}

bool CoreNode::onGetMode(uvc::get_mode::Request &request, uvc::get_mode::Response &response)
{
    int32_t mode = static_cast<int32_t>(m_mode);

    response.mode = mode;

    return true;
}

bool CoreNode::onSetMode(uvc::set_mode::Request &request, uvc::set_mode::Response &response)
{
    ModeType mode = static_cast<ModeType>(request.mode);

    setMode(mode);

    return true;
}

void CoreNode::setMode(ModeType const &mode)
{
    m_mode = mode;

    m_controller.reset();

    switch (mode) {
        case IDLE:
            m_controller = std::unique_ptr<IdleController>(new IdleController);
        
            break;

        case MANUAL:
            m_controller = std::unique_ptr<ManualController>(new ManualController);

            break;

        case AUTO:
            m_controller = std::unique_ptr<AutoController>(new AutoController);

            break;

        default:
            break;
    }
}

void CoreNode::update()
{
    m_controller->update();
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
