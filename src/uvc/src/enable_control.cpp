#include "enable_control.hpp"

EnableControl::EnableControl(ros::NodeHandle &node_handle) : m_enable(true)
{
    m_subscriber = node_handle.subscribe("enable", 100, &EnableControl::onEnableChanged, this);
}

bool EnableControl::getEnable() const
{
    return m_enable;
}

void EnableControl::onEnableChanged(uvc::enable::ConstPtr const &message)
{
    m_enable = message->enable;

    if (m_enable) {
        onEnable();
    }
    else {
        onDisable();
    }
}

void EnableControl::onEnable()
{
}

void EnableControl::onDisable()
{
}
