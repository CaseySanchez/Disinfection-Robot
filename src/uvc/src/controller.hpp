#pragma once

#include <wiringPi.h>

#include "ros/ros.h"
#include "ros/console.h"

#include "thread.hpp"

class Controller : private Thread
{
public:
    Controller();

    ~Controller();

    void run() override;

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
