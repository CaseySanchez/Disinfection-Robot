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

    ros::ServiceClient m_get_lamp_client;
    ros::ServiceClient m_set_lamp_client;

public:
    RestNode();

    ~RestNode();

    void getHandler(http_request message);
    void postHandler(http_request message);
};
