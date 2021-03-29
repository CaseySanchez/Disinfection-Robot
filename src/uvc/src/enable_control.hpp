#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/enable.h"

class EnableControl
{
    ros::Subscriber m_subscriber;

    bool m_enable;

public:
    EnableControl(ros::NodeHandle &node_handle);

    bool getEnable() const;

    void onEnableChanged(uvc::enable::ConstPtr const &message);

    virtual void onEnable();
    virtual void onDisable();
};
