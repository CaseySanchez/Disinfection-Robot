#include "thread.hpp"

Thread::Thread()
{
    std::atomic_init(&m_running, false);
}

Thread::~Thread()
{
    stop();
}

void Thread::start()
{
    if (!m_thread) {
        m_running.store(true);

        m_thread = std::make_unique<std::thread>(&Thread::task, this);
    }
}

void Thread::stop()
{
    if (m_thread) {
        m_running.store(false);

        m_thread->join();

        m_thread.reset();
    }
}

void Thread::task()
{
    while (m_running.load()) {
        run();
    }
}
