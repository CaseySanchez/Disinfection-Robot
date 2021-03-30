#include "ros/ros.h"
#include "ros/console.h"

#include "rest_node.hpp"

#include <thread>

RestNode::RestNode() : ros::NodeHandle("/"), m_listener("http://0.0.0.0:8080")
{
    m_listener.support(methods::POST, std::bind(&RestNode::postHandler, this, std::placeholders::_1));

    m_listener.open().wait();

    m_set_lamp_client = serviceClient<uvc::set_lamp>("lamp_node/set_lamp");
}

~RestNode::RestNode()
{
    m_listener.close().wait();
}

void RestNode::postHandler(http_request message)
{
    auto paths = uri::split_path(uri::decode(message.relative_uri().path()));

    auto query = uri::split_query(uri::decode(message.request_uri().query()));

    if (paths[0] == "set_lamp") {
        if (query["active"] == "true") {
            uvc::set_lamp set_lamp;

            set_lamp.request.active = true;

            m_set_lamp_client.call(set_lamp);
        }
        else if (query["active"] == "false") {
            uvc::set_lamp set_lamp;

            set_lamp.request.active = false;

            m_set_lamp_client.call(set_lamp);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rest_node");

    RestNode rest_node;

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
