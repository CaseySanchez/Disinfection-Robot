#include "controller.hpp"

Controller::Controller()
{
    start();

    Thread::start();
}

Controller::~Controller()
{
    Thread::stop();
    
    stop();
}

void Controller::run()
{
    update();
}
