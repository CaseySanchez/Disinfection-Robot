#pragma once

#include <thread>
#include <chrono>

#include <wiringPi.h>

#include "ros/ros.h"
#include "ros/console.h"

class Controller
{
	std::unique_ptr<std::thread> m_thread;
	std::atomic<bool> m_running;

    std::chrono::time_point m_time_stamp;

public:
	Controller(Controller const &) = delete;
	Controller &operator=(Controller const &) = delete;

    Controller();

    virtual ~Controller();

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void update(double const &delta_time) = 0;

private:
    void task();
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

    void update(double const &delta_time) override
    {
    }
};

class ManualController : public Controller
{
public:
    void start() override
    {
    }

    void stop() override
    {
    }

    void update(double const &delta_time) override
    {
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

        m_time_stamp = std::chrono::high_resolution_clock::now();
    }

    void stop() override
    {
        ROS_INFO("stop");

        digitalWrite(8, 0);
    }

    void update(double const &delta_time) override
    {
        ROS_INFO("update");
    }
};
