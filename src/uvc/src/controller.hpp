#pragma once

#include "thread.hpp"

#include "uvc/get_lamp.h"
#include "uvc/set_lamp.h"

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
        uvc::set_lamp set_lamp;

        set_lamp.request.active = true;

        m_set_lamp_client.call(set_lamp);
    }

    void stop() override
    {
        uvc::set_lamp set_lamp;

        set_lamp.request.active = false;

        m_set_lamp_client.call(set_lamp);
    }

    void update() override
    {
    }
};