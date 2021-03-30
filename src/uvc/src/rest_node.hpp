#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "cpprest/asyncrt_utils.h"
#include "cpprest/http_listener.h"
#include "cpprest/json.h"
#include "cpprest/uri.h"

#include "uvc/set_lamp.h"

using namespace web;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;

class RestNode : public ros::NodeHandle
{
    http_listener m_listener;

    ros::ServiceClient set_lamp_client;

public:
    RestNode() : ros::NodeHandle("~"), m_listener("http://0.0.0.0:8080")
    {
        m_listener.support(methods::POST, std::bind(&RestNode::postHandler, this, std::placeholders::_1));

        m_listener.open();

        set_lamp_client = serviceClient<uvc::set_lamp>("set_lamp");
    }

    ~RestNode()
    {
        m_listener.close();
    }

    void postHandler(http_request message)
    {
        auto paths = uri::split_path(uri::decode(message.relative_uri().path()));

        auto query = uri::split_query(uri::decode(message.request_uri().query()));

        if (paths[0] == "set_lamp") {
            if (query["active"] == "true") {
                uvc::set_lamp set_lamp;

                set_lamp.request.active = true;

                set_lamp_client.call(set_lamp);
            }
            else if (query["active"] == "false") {
                uvc::set_lamp set_lamp;

                set_lamp.request.active = false;

                set_lamp_client.call(set_lamp);
            }
        }
    }
};
