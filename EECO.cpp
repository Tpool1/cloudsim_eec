#include "SchedulingAlgorithm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// TUNING PARAMETERS: edit here and rebuild. Defaults preserve the original policy.
constexpr double kDemandSafetyFactor = 1.0; // >= 1; 1.20 requests 20% extra MIPS in admission, DVFS, and reserves.
constexpr double kAggressiveness = 0.5;     // [0, 1]; lower values generally retain more standby hosts.
constexpr double kIdleStandbyFraction = 0.10; // [0, 1]; fraction reserved when no hosts run in a CPU family.
constexpr double kIdleHostPenalty = 1e20;   // W*us added to placement score for opening an idle host; 0 disables bias.
constexpr MachineState_t kStandbyState = S0i1; // S0..S4; deeper standby saves power but increases wake delay.
constexpr MachineState_t kExcessIdleState = S5; // S0..S5; S0 keeps excess capacity ready; S5 powers it off.
// Pool updates occur on every scheduling event; there is currently no idle grace period.
// Task priorities are shared in SchedulingAlgorithm.cpp; wake costs come from the simulator.

static_assert(kDemandSafetyFactor >= 1 && kDemandSafetyFactor <= std::numeric_limits<double>::max() &&
              kAggressiveness >= 0 && kAggressiveness <= 1 &&
              kIdleStandbyFraction >= 0 && kIdleStandbyFraction <= 1 &&
              kIdleHostPenalty >= 0 && kIdleHostPenalty <= std::numeric_limits<double>::max(),
              "Invalid E-ECO demand, reserve, or placement parameters");
static_assert(kStandbyState >= S0 && kStandbyState <= S4 &&
              kExcessIdleState >= S0 && kExcessIdleState <= S5, "Invalid E-ECO power states");
}

// Rossi et al., JNCA 2017, section 4; running/intermediate/off pools plus DVFS.
void EECO::Init() {
    std::fill(std::begin(previous_running), std::end(previous_running), 0);
    SchedulingAlgorithm::Init();
}

bool EECO::HasCapacity(Time_t now, const TaskInfo_t& task, MachineId_t machine) const {
    const auto host = Machine_GetInfo(machine);
    if (TasksOn(machine).empty()) return true; // Even a missed deadline must be admitted on an idle host.
    return kDemandSafetyFactor * RequiredRate(now, machine, &task) <= host.performance[FastestState(machine)];
}

MachineId_t EECO::SelectMachine(Time_t now, const TaskInfo_t& task,
                                const vector<MachineId_t>& candidates) const {
    MachineId_t best = SchedulingAlgorithm::SelectMachine(now, task, candidates);
    double best_score = std::numeric_limits<double>::infinity();
    for (auto m : candidates) {
        const auto host = Machine_GetInfo(m);
        const double rate = kDemandSafetyFactor * RequiredRate(now, m, &task);
        if (rate > host.performance[FastestState(m)]) continue;
        const unsigned p = StateForRate(m, rate);
        if (p >= host.p_states.size() || host.s_states.empty()) continue;
        // Estimate energy for this task at the state needed by the combined demand.
        const double watts = host.s_states[S0] + host.num_cpus * double(host.p_states[p]);
        // Reuse running hosts before opening an idle host when the deadlines allow packing.
        const double score = (TasksOn(m).empty() ? kIdleHostPenalty : 0) +
                             watts * Work(task, host) / host.performance[p];
        if (score < best_score) { best = m; best_score = score; }
    }
    return best;
}

void EECO::UpdatePower(Time_t now) {
    for (unsigned cpu = 0; cpu < 4; ++cpu) {
        unsigned running = 0;
        double demand = 0, capacity = 0;
        vector<MachineId_t> idle;
        for (unsigned m = 0; m < Machine_GetTotal(); ++m) {
            const auto host = Machine_GetInfo(m);
            if (unsigned(host.cpu) != cpu || !host.num_cpus || host.performance.empty()) continue;
            if (host.active_tasks || !TasksOn(m).empty()) {
                ++running;
                const double rate = kDemandSafetyFactor * RequiredRate(now, m);
                demand += rate * host.num_cpus;
                capacity += double(host.performance[FastestState(m)]) * host.num_cpus;
                SetPerformance(m, StateForRate(m, rate));
            } else idle.push_back(m);
        }
        // Size the standby pool from utilization and the change in running hosts.
        const double lambda = previous_running[cpu] ?
            std::clamp(kAggressiveness * running / previous_running[cpu], 0.0, 1.0) : kAggressiveness;
        const double utilization = capacity ? demand / capacity : 0;
        const double alpha = std::clamp(utilization - lambda, 0.0, 1.0);
        const unsigned reserve = running ? unsigned(std::ceil(running * alpha)) :
                                          unsigned(std::ceil(idle.size() * kIdleStandbyFraction));
        previous_running[cpu] = running;
        // A pending request must not lose capacity to a simultaneous sleep decision.
        bool pending = false;
        for (auto id : waiting) if (unsigned(GetTaskInfo(id).required_cpu) == cpu) pending = true;
        if (pending) continue;
        // Keep existing warm hosts first, avoiding unnecessary standby/off exchanges.
        std::stable_sort(idle.begin(), idle.end(), [](auto a, auto b) {
            return Machine_GetInfo(a).s_state < Machine_GetInfo(b).s_state;
        });
        for (unsigned i = 0; i < idle.size(); ++i) {
            const auto m = idle[i];
            const auto state = Machine_GetInfo(m).s_state;
            // shortcut: standby choice is fixed; calibrate wake costs before selecting deeper states.
            if (i < reserve) {
                if (state == S0) RequestState(m, kStandbyState);
                else if (state != kStandbyState) RequestState(m, S0);
            } else RequestState(m, kExcessIdleState);
        }
    }
}
