#include "controller.hpp"

Controller::Controller()
{
    std::atomic_init(&m_running, false);

    if (!m_thread) {
        m_running.store(true);

        m_thread = std::make_unique<std::thread>(&Thread::task, this);
    }
}

Controller::~Controller()
{
    if (m_thread) {
        m_running.store(false);

        m_thread->join();

        m_thread.reset();
    }
}

void Controller::task()
{
    start();

    while (m_running.load()) {
        std::chrono::time_point const time_stamp = std::chrono::high_resolution_clock::now();

        double const delta_time = std::chrono::duration<double>(time_stamp - m_time_stamp).count();

        m_time_stamp = time_stamp;

        update(delta_time);
    }

    stop();
}