#include "state_machine.hpp"

#pragma once

#include <memory>
#include <optional>

void State::enter()
{
}

std::optional<std::shared_ptr<State>> State::update()
{
    return { };
}

void State::exit()
{
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