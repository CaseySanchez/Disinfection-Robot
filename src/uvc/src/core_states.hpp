#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <wiringPi.h>
#include <softPwm.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "json.hpp"

#include "state_machine.hpp"

class IdleState : private State
{
    void enter() override;
    void exit() override;
    std::optional<std::shared_ptr<State>> update() override;
};

class ManualState : private State
{
    websocketpp::server<websocketpp::config::asio> m_server;
    
    std::thread m_thread;

    std::mutex m_mutex;

    std::array<float, 4> m_motor_speed;
    std::array<float, 4> m_target_speed;

    StateMachine m_state_machine;

    void enter() override;
    void exit() override;
    std::optional<std::shared_ptr<State>> update() override;

    void serverThread();

    void openHandler(websocketpp::connection_hdl connection_hdl);
    void closeHandler(websocketpp::connection_hdl connection_hdl);
    void messageHandler(websocketpp::connection_hdl connection_hdl, websocketpp::server<websocketpp::config::asio>::message_ptr message) ;
};

class AutoState : private State
{
    StateMachine m_state_machine;

    class ScanState : private State
    {
        int i;

        void enter() override
        {
            ROS_INFO("Scan enter");
        }

        void exit() override
        {
            ROS_INFO("Scan exit");
        }

        std::optional<std::shared_ptr<State>> update() override
        {
            if (i++ > 10) {
                return std::make_shared<PlanState>();
            }

            return std::make_shared<NavigateState>();
        }

    public:
        ScanState() : i(0)
        {
        }
    };

    class NavigateState : private State
    {
        void enter() override
        {
            ROS_INFO("Navigate enter");
        }

        void exit() override
        {
            ROS_INFO("Navigate exit");
        }

        std::optional<std::shared_ptr<State>> update() override
        {
            return std::make_shared<ScanState>();
        }
    };

    class PlanState : private State
    {
        void enter() override
        {
            ROS_INFO("Plan enter");
        }

        void exit() override
        {
            ROS_INFO("Plan exit");
        }

        std::optional<std::shared_ptr<State>> update() override
        {
            return { };
        }
    };

    void enter() override;
    void exit() override;
    std::optional<std::shared_ptr<State>> update() override;
};
