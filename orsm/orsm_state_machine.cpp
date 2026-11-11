// 越野堡垒模式状态机实现
// 来源：M18-3先导车越野堡垒功能详细说明书V1.2
// T1-T6 状态切换逻辑

#include "orsm_state_machine.h"
#include "orsm_settings.h"

namespace sentinel {

OrsmStateMachine::OrsmStateMachine() = default;

void OrsmStateMachine::reset() {
    state_ = OrsmState::OFF;
    warning_count_ = 0;
}

void OrsmStateMachine::transition(OrsmState to, const std::string& reason, uint64_t ts) {
    if (to == state_) return;
    OrsmStateEvent evt{state_, to, reason, ts};
    state_ = to;
    if (to == OrsmState::WARNING) {
        ++warning_count_;
    }
    if (cb_) {
        cb_(evt);
    }
}

void OrsmStateMachine::update(const OrsmOdd& odd, bool threat_detected, bool fault, uint64_t ts) {
    // 综合故障判断：智驾故障 或 传感器故障
    const bool has_fault = odd.had_fault || odd.sensor_fault || fault;

    // T2退出条件（适用于 ACTIVE / WARNING）：
    //   软开关=0，或非P档，或SOC<20%，或车速>20km/h
    const bool t2_exit = (!odd.sw_enable || !orsmEnabled(odd.sw_mode)) ||
                         !odd.gear_park ||
                         (odd.soc < odd.soc_deactivate_threshold) ||
                         (odd.speed_kmh > odd.speed_exit_threshold_kmh);

    switch (state_) {
    case OrsmState::OFF: {
        // T1 Off→Active：电源ON，P档，SOC≥30%，手刹，无故障，软开关开启
        if (has_fault) {
            transition(OrsmState::FAILURE, "T6: fault detected in OFF", ts);
        } else if (odd.power_on &&
                   odd.gear_park &&
                   odd.soc >= odd.soc_activate_threshold &&
                   odd.handbrake_on &&
                   !has_fault &&
                   odd.sw_enable &&
                   orsmEnabled(odd.sw_mode)) {
            transition(OrsmState::ACTIVE, "T1: ODD satisfied, sw_enable ON", ts);
        }
        break;
    }

    case OrsmState::ACTIVE: {
        // T2 优先级最高：退出条件
        if (t2_exit) {
            transition(OrsmState::OFF, "T2: exit condition met", ts);
        }
        // T6 故障 → FAILURE
        else if (has_fault) {
            transition(OrsmState::FAILURE, "T6: fault detected in ACTIVE", ts);
        }
        // T3 报警条件 → WARNING
        else if (threat_detected) {
            transition(OrsmState::WARNING, "T3: threat detected", ts);
        }
        break;
    }

    case OrsmState::WARNING: {
        // T2 退出条件
        if (t2_exit) {
            transition(OrsmState::OFF, "T2: exit condition met in WARNING", ts);
        }
        // T6 故障 → FAILURE
        else if (has_fault) {
            transition(OrsmState::FAILURE, "T6: fault detected in WARNING", ts);
        }
        // T4 报警解除 → ACTIVE
        else if (!threat_detected) {
            transition(OrsmState::ACTIVE, "T4: threat cleared", ts);
        }
        break;
    }

    case OrsmState::FAILURE: {
        // T5 故障解除 → ACTIVE（需要ODD基本条件仍满足）
        if (!has_fault &&
            odd.power_on &&
            odd.gear_park &&
            odd.soc >= odd.soc_activate_threshold &&
            odd.sw_enable &&
            orsmEnabled(odd.sw_mode)) {
            transition(OrsmState::ACTIVE, "T5: fault cleared, ODD OK", ts);
        }
        break;
    }
    }  // switch
}

}  // namespace sentinel
