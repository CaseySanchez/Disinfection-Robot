#include "ros/ros.h"
#include "ros/console.h"

#include "rest_node.hpp"

#include <thread>

RestNode::RestNode() : ros::NodeHandle("/"), m_listener("http://0.0.0.0:8080")
{
    m_listener.support(methods::GET, std::bind(&RestNode::getHandler, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&RestNode::postHandler, this, std::placeholders::_1));

    m_listener.open().wait();

    m_get_mode_client = serviceClient<uvc::get_mode>("core_node/get_mode");
    m_set_mode_client = serviceClient<uvc::set_mode>("core_node/set_mode");
}

RestNode::~RestNode()
{
    m_listener.close().wait();
}

void RestNode::getHandler(http_request request)
{
    auto paths = uri::split_path(uri::decode(request.relative_uri().path()));

    if (paths[0] == "get_mode") {
        uvc::get_mode get_mode;

        m_get_mode_client.call(get_mode);

        auto response = json::value::object();

        response["mode"] = get_mode.response.mode;

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

    if (paths[0] == "set_mode") {
        if (query["mode"] == "idle") {
            uvc::set_mode set_mode;

            set_mode.request.mode = 0;

            m_set_mode_client.call(set_mode);

            request.reply(status_codes::OK);
        }
        else if (query["mode"] == "manual") {
            uvc::set_mode set_mode;

            set_mode.request.mode = 1;

            m_set_mode_client.call(set_mode);

            request.reply(status_codes::OK);
        }
        else if (query["mode"] == "auto") {
            uvc::set_mode set_mode;

            set_mode.request.mode = 2;

            m_set_mode_client.call(set_mode);

            request.reply(status_codes::OK);
        }
        else {
            request.reply(status_codes::BadRequest);
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
