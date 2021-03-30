#include "ros/ros.h"
#include "ros/console.h"

#include "rest_node.hpp"

#include <thread>

RestNode::RestNode() : ros::NodeHandle("/"), m_listener("http://0.0.0.0:8080")
{
    m_listener.support(methods::GET, std::bind(&RestNode::getHandler, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&RestNode::postHandler, this, std::placeholders::_1));

    m_listener.open().wait();

    m_get_lamp_client = serviceClient<uvc::set_lamp>("lamp_node/get_lamp");
    m_set_lamp_client = serviceClient<uvc::set_lamp>("lamp_node/set_lamp");
}

RestNode::~RestNode()
{
    m_listener.close().wait();
}

void RestNode::getHandler(http_request request)
{
    auto paths = uri::split_path(uri::decode(request.relative_uri().path()));

    if (paths[0] == "get_lamp") {
        uvc::get_lamp get_lamp;

        m_get_lamp_client.call(get_lamp);

        auto response = json::value::object();

        response["active"] = get_lamp.response.active;

        request.reply(status_codes::OK, response);
    }
    else {
        request.reply(status_codes::BadRequest);
    }
}

void RestNode::postHandler(http_request request)
{
    auto paths = uri::split_path(uri::decode(request.relative_uri().path()));

    auto query = uri::split_query(uri::decode(request.request_uri().query()));

    if (paths[0] == "set_lamp") {
        if (query["active"] == "true") {
            uvc::set_lamp set_lamp;

            set_lamp.request.active = true;

            m_set_lamp_client.call(set_lamp);

            request.reply(status_codes::OK);
        }
        else if (query["active"] == "false") {
            uvc::set_lamp set_lamp;

            set_lamp.request.active = false;

            m_set_lamp_client.call(set_lamp);

            request.reply(status_codes::OK);
        }
    }
    else {
        request.reply(status_codes::BadRequest);
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
