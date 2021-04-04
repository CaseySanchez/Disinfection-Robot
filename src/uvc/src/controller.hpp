#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <wiringPi.h>
#include <softPwm.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "json.hpp"

class Controller
{
public:
    virtual void start();
    virtual void stop();
    virtual void update();
};

class IdleController : public Controller
{
public:
    void start() override
    {
    }

    void stop() override
    {
    }

    void update() override
    {
    }
};

class ManualController : public Controller
{
    websocketpp::server<websocketpp::config::asio> m_server;
    
    std::thread m_thread;

    std::mutex m_mutex;

    std::array<float, 4> m_motor_speed;

public:
    void start() override
    {
        wiringPiSetup();

        pinMode(26, OUTPUT);
        pinMode(27, OUTPUT);
        pinMode(28, OUTPUT);
        pinMode(29, OUTPUT);

        softPwmCreate(21, 0, 100);
        softPwmCreate(22, 0, 100);
        softPwmCreate(23, 0, 100);
        softPwmCreate(24, 0, 100);

        m_server.set_open_handler(websocketpp::lib::bind(&ManualController::openHandler, this, websocketpp::lib::placeholders::_1));
        m_server.set_close_handler(websocketpp::lib::bind(&ManualController::closeHandler, this, websocketpp::lib::placeholders::_1));
        m_server.set_message_handler(websocketpp::lib::bind(&ManualController::messageHandler, this, websocketpp::lib::placeholders::_1, websocketpp::lib::placeholders::_2));
        
        m_server.set_access_channels(websocketpp::log::alevel::all);
        m_server.set_error_channels(websocketpp::log::elevel::all);

        m_server.init_asio();
        m_server.listen(9002);
        m_server.start_accept();

        m_thread = std::thread(&ManualController::serverThread, this);
    }

    void stop() override
    {
        m_server.stop_listening();

        m_thread.join();
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock_guard(m_mutex);

        digitalWrite(26, m_motor_speed[0] > 0.0 ? 0 : 1);
        digitalWrite(27, m_motor_speed[1] > 0.0 ? 0 : 1);
        digitalWrite(28, m_motor_speed[2] > 0.0 ? 0 : 1);
        digitalWrite(29, m_motor_speed[3] > 0.0 ? 0 : 1);

        std::array<int32_t, 4> duty_cycle = {
            static_cast<int32_t>(std::abs(m_motor_speed[0]) * 100.0f),
            static_cast<int32_t>(std::abs(m_motor_speed[1]) * 100.0f),
            static_cast<int32_t>(std::abs(m_motor_speed[2]) * 100.0f),
            static_cast<int32_t>(std::abs(m_motor_speed[3]) * 100.0f)
        };

        softPwmWrite(21, duty_cycle[0]);
        softPwmWrite(22, duty_cycle[1]);
        softPwmWrite(23, duty_cycle[2]);
        softPwmWrite(24, duty_cycle[3]);
    }

    void serverThread()
    {
        m_server.run();
    }

    void openHandler(websocketpp::connection_hdl connection_hdl)
    {
    }

    void closeHandler(websocketpp::connection_hdl connection_hdl)
    {
    }

    void messageHandler(websocketpp::connection_hdl connection_hdl, websocketpp::server<websocketpp::config::asio>::message_ptr message) 
    {
        std::lock_guard<std::mutex> lock_guard(m_mutex);

        auto json = nlohmann::json::parse(message->get_payload());

        m_motor_speed[0] = json["motor_0"];
        m_motor_speed[1] = json["motor_1"];
        m_motor_speed[2] = json["motor_2"];
        m_motor_speed[3] = json["motor_3"];

        std::cout << message->get_payload() << std::endl;
    }
};

class AutoController : public Controller
{
public:
    void start() override
    {
        ROS_INFO("start");

        wiringPiSetup();
        
        pinMode(8, OUTPUT);

        digitalWrite(8, 1);
    }

    void stop() override
    {
        ROS_INFO("stop");

        digitalWrite(8, 0);
    }

    void update() override
    {
        ROS_INFO("update");
    }
};
