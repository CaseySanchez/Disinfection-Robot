#include "state_machine.hpp"

void State::enter()
{
}

void State::exit()
{
}

std::optional<std::string> State::update()
{
    return { };
}

StateMachine::StateMachine(std::map<std::string, std::shared_ptr<State>> const &state_map, std::string const &initial_state) : m_state_map(state_map)
{
    m_state = m_state_map[initial_state];

    m_state->enter();
}

void StateMachine::update()
{
    std::optional<std::string> state = m_state->update();

    if (state.has_value()) {
        transition(state.value());
    }
}

void StateMachine::transition(std::string const &state)
{
    m_state->exit();

    m_state = m_state_map[state];

    m_state->enter();
}