#pragma once

#include <memory>
#include <map>
#include <string>
#include <optional>

class State
{
    friend class StateMachine;

    virtual void enter();

    virtual void exit();

    virtual std::optional<std::string> update();
};

class StateMachine
{
    std::map<std::string, std::shared_ptr<State>> m_state_map;

    std::shared_ptr<State> m_state;

public:
    StateMachine(std::map<std::string, std::shared_ptr<State>> const &state_map, std::string const &initial_state);

    void update();

    void transition(std::string const &state);
};