#include "ros/ros.h"
#include "ros/console.h"

#include "uvc/set_enable.h"
#include "uvc/enable.h"

class SystemControl
{
    ros::ServiceServer m_service_server;

    ros::Publisher m_publisher;

public:
    SystemControl(ros::NodeHandle &node_handle);
    
    bool onSetEnable(uvc::set_enable::Request &request, uvc::set_enable::Response &response);
};
