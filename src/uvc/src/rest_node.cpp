#include "rest_node.hpp"

RestNode::RestNode() : ros::NodeHandle("/"), m_listener("http://0.0.0.0:8080")
{
    m_listener.support(methods::GET, std::bind(&RestNode::getHandler, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&RestNode::postHandler, this, std::placeholders::_1));

    m_listener.open().wait();

    m_get_state_client = serviceClient<uvc::get_state>("core_node/get_state");
    m_set_state_client = serviceClient<uvc::set_state>("core_node/set_state");
}

RestNode::~RestNode()
{
    m_listener.close().wait();
}

void RestNode::getHandler(http_request request)
{
    auto paths = uri::split_path(uri::decode(request.relative_uri().path()));

    if (paths[0] == "get_state") {
        uvc::get_state get_state;

        m_get_state_client.call(get_state);

        auto response = json::value::object();

        response["state"] = get_state.response.state;

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

    if (paths[0] == "set_state") {
        if (query["state"] == "idle") {
            uvc::set_state set_state;

            set_state.request.state = 0;

            m_set_state_client.call(set_state);

            request.reply(status_codes::OK);
        }
        else if (query["state"] == "manual") {
            uvc::set_state set_state;

            set_state.request.state = 1;

            m_set_state_client.call(set_state);

            request.reply(status_codes::OK);
        }
        else if (query["state"] == "auto") {
            uvc::set_state set_state;

            set_state.request.state = 2;

            m_set_state_client.call(set_state);

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
