#pragma once

#include <wiringPi.h>

#include "thread.hpp"

class Controller : private Thread
{
public:
    Controller();

    ~Controller();

    void run() override;

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void update() = 0;
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
        wiringPiSetup();
        
        pinMode(8, OUTPUT);

        digitalWrite(8, 1);
    }

    void stop() override
    {
        digitalWrite(8, 0);
    }

    void update() override
    {
    }
};