#include "controller.hpp"

Controller::Controller()
{
    std::atomic_init(&m_running, true);

    m_thread = std::thread(&Controller::controllerThread, this);
}

Controller::~Controller()
{
    m_running.store(false);

    m_thread.join();
}

void Controller::start()
{
}

void Controller::stop()
{
}

void Controller::update()
{
}

void Controller::controllerThread()
{
    start();

    while (m_running.load()) {
        update();
    }

    stop();
}
