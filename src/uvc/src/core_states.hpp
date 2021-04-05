#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <wiringPi.h>
#include <softPwm.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "json.hpp"

#include "state_machine.hpp"

class IdleState : public State
{
public:
    void enter() override;
    void exit() override;
    std::optional<std::shared_ptr<State>> update() override;
};

class ManualState : public State
{
    websocketpp::server<websocketpp::config::asio> m_server;
    
    std::thread m_thread;

    std::mutex m_mutex;

    std::array<float, 4> m_motor_speed;

public:
    void enter() override;
    void exit() override;
    std::optional<std::shared_ptr<State>> update() override;

    void serverThread();

    void openHandler(websocketpp::connection_hdl connection_hdl);
    void closeHandler(websocketpp::connection_hdl connection_hdl);
    void messageHandler(websocketpp::connection_hdl connection_hdl, websocketpp::server<websocketpp::config::asio>::message_ptr message) ;
};

class AutoState : public State
{
public:
    void enter() override;
    void exit() override;
    std::optional<std::shared_ptr<State>> update() override;
};
