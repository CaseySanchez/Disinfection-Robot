#pragma once

#include <memory>
#include <optional>

class State
{
    friend class StateMachine;

    virtual void enter();

    virtual void exit();
    
    virtual std::optional<std::shared_ptr<State>> update();
};

class StateMachine
{
    std::shared_ptr<State> m_state;

public:
    StateMachine(std::shared_ptr<State> const &state = State());

    void update();

    void setState(std::shared_ptr<State> const &state);
};