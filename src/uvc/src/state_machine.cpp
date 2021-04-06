#include "state_machine.hpp"

void State::enter()
{
}

void State::exit()
{
}

std::optional<std::shared_ptr<State>> State::update()
{
    return { };
}

StateMachine::StateMachine(std::shared_ptr<State> const &state) : m_state(state)
{
    m_state->enter();
}

void StateMachine::update()
{
    std::optional<std::shared_ptr<State>> state = m_state->update();

    if (state.has_value()) {
        setState(state.value());
    }
}

void StateMachine::setState(std::shared_ptr<State> const &state)
{
    m_state->exit();

    m_state = state;

    m_state->enter();
}
