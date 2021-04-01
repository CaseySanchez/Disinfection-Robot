#pragma once

#include <thread>
#include <atomic>

class Thread
{
	std::unique_ptr<std::thread> m_thread;
	std::atomic<bool> m_running;

public:
	Thread(Thread const &) = delete;
	Thread &operator=(Thread const &) = delete;

	Thread();
	~Thread();

	void start();
	void stop();

private:
	virtual void run() = 0;

	void task();
};
