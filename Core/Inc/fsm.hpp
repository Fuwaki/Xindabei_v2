#pragma once

#include <stdint.h>
#include <vector>
#include <memory>
#include "log.h"

// ============================================================================
// 通用状态机基础设施 (Generic FSM Infrastructure)
// ============================================================================

// --- 1. 状态接口 ---
template<typename Context, typename StateEnum>
struct IState {
    virtual void Enter(Context& ctx) {}
    virtual void Exit(Context& ctx) {}
    // 返回下一状态，如果无需切换则返回当前状态
    virtual StateEnum Update(Context& ctx, uint32_t dt) = 0;
    virtual const char* Name() const = 0;
    virtual ~IState() = default;
};

// --- 2. 状态机核心 ---
template<typename StateEnum, typename Context>
class StateMachine {
public:
    using StateType = IState<Context, StateEnum>;
    
    StateMachine() : m_currentStateID(static_cast<StateEnum>(0)) {}
    
    void RegisterState(StateEnum stateID, std::unique_ptr<StateType> state) {
        size_t index = static_cast<size_t>(stateID);
        if (m_states.size() <= index) {
            m_states.resize(index + 1);
        }
        m_states[index] = std::move(state);
    }
    
    void ChangeState(StateEnum newStateID) {
        size_t index = static_cast<size_t>(newStateID);
        if (index >= m_states.size() || !m_states[index]) return;
        if (m_currentStateID == newStateID) return;

        size_t currentIndex = static_cast<size_t>(m_currentStateID);
        const char* oldStateName = (currentIndex < m_states.size() && m_states[currentIndex]) 
                                   ? m_states[currentIndex]->Name() : "UNKNOWN";
        const char* newStateName = m_states[index]->Name();
        
        LOG_INFO("FSM: %s -> %s", oldStateName, newStateName);

        if (currentIndex < m_states.size() && m_states[currentIndex]) {
            m_states[currentIndex]->Exit(m_context);
        }

        m_currentStateID = newStateID;
        m_states[index]->Enter(m_context);
    }
    
    void Update(uint32_t dt) {
        size_t index = static_cast<size_t>(m_currentStateID);
        if (index < m_states.size() && m_states[index]) {
            StateEnum nextState = m_states[index]->Update(m_context, dt);
            if (nextState != m_currentStateID) {
                ChangeState(nextState);
            }
        }
    }
    
    StateEnum GetCurrentStateID() const { return m_currentStateID; }
    
    const char* GetCurrentStateName() const {
        size_t index = static_cast<size_t>(m_currentStateID);
        return (index < m_states.size() && m_states[index]) ? m_states[index]->Name() : "UNKNOWN";
    }
    
    Context& GetContext() { return m_context; }
    const Context& GetContext() const { return m_context; }

private:
    std::vector<std::unique_ptr<StateType>> m_states;
    StateEnum m_currentStateID;
    Context m_context;
};