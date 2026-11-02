#include "SchedulingAlgorithm.hpp"
#include <algorithm>

namespace {
// TUNING PARAMETERS: edit here and rebuild. All times are simulated microseconds.
constexpr Time_t kControlIntervalUs = 5'000'000;  // Shorter intervals react sooner to latency changes.
constexpr Time_t kLatencyWindowUs = 30'000'000;   // Longer windows smooth the response-time mean.
constexpr Time_t kInstantWindowUs = 5'000'000;    // Recent completions used for instantaneous latency.
constexpr Time_t kEmergencyHoldUs = 300'000'000; // Full-power hold after a mean/SLA violation.
constexpr double kLatencyTarget = 0.95;          // Fraction of each task's arrival-to-deadline budget.
constexpr double kMaxPowerLatencyRatio = 1.35;   // Relative to kLatencyTarget; exceed to restore full power.
constexpr double kSmallDecreaseLatencyRatio = 0.85; // Below this target ratio, decrease power slowly.
constexpr double kLargeDecreaseLatencyRatio = 0.60; // Below this target ratio, decrease power faster.
constexpr double kPowerIncreaseStep = 0.07;      // Add 7 percentage points of full CPU power.
constexpr double kSmallPowerDecreaseStep = 0.01; // Subtract 1 percentage point of full CPU power.
constexpr double kLargePowerDecreaseStep = 0.03; // Subtract 3 percentage points of full CPU power.
constexpr double kMinimumPowerFraction = 0.10;   // Lower bound on the requested fraction of full power.
// Task priority mapping is shared in SchedulingAlgorithm.cpp.

static_assert(kControlIntervalUs > 0 && kInstantWindowUs > 0 &&
              kLatencyWindowUs >= kInstantWindowUs, "Pegasus latency windows must be positive and ordered");
static_assert(kLatencyTarget > 0 && kLatencyTarget <= 1 && kMaxPowerLatencyRatio > 1 &&
              kLargeDecreaseLatencyRatio > 0 &&
              kLargeDecreaseLatencyRatio < kSmallDecreaseLatencyRatio &&
              kSmallDecreaseLatencyRatio < 1, "Invalid Pegasus latency thresholds");
static_assert(kMinimumPowerFraction > 0 && kMinimumPowerFraction <= 1 &&
              kPowerIncreaseStep > 0 && kPowerIncreaseStep <= 1 &&
              kSmallPowerDecreaseStep > 0 && kSmallPowerDecreaseStep <= kLargePowerDecreaseStep &&
              kLargePowerDecreaseStep <= 1, "Pegasus power fractions must be in (0, 1]");
}

// Lo et al., ISCA 2014, section 5 and table 1; RAPL is mapped to discrete P-states.
void Pegasus::Init() {
    samples.clear();
    last_control = emergency_until = 0;
    power_limit = 1;
    SchedulingAlgorithm::Init();
}

void Pegasus::TaskComplete(Time_t now, TaskId_t id) {
    // Normalize latency by the task’s available deadline budget.
    const auto task = GetTaskInfo(id);
    if (task.required_sla != SLA3 && task.target_completion > task.arrival)
        samples.push_back({now, double(now - task.arrival) /
                               double(task.target_completion - task.arrival)});
    SchedulingAlgorithm::TaskComplete(now, id);
}

void Pegasus::SLAWarning(Time_t now, TaskId_t id) {
    power_limit = 1;
    emergency_until = now + kEmergencyHoldUs;
    for (unsigned m = 0; m < Machine_GetTotal(); ++m) SetPerformance(m, FastestState(m));
    SchedulingAlgorithm::SLAWarning(now, id);
}

void Pegasus::UpdatePower(Time_t now) {
    if (now - last_control < kControlIntervalUs) return;
    last_control = now;
    while (!samples.empty() && now - samples.front().time > kLatencyWindowUs) samples.pop_front();
    double average = 0, instant = 0;
    unsigned recent = 0;
    for (const auto& sample : samples) {
        average += sample.latency;
        if (now - sample.time <= kInstantWindowUs) { instant += sample.latency; ++recent; }
    }
    if (!samples.empty()) average /= samples.size();
    if (recent) instant /= recent;
    // In-flight requests prevent long-running tasks from hiding an impending violation.
    for (const auto& entry : assignments) {
        const auto task = GetTaskInfo(entry.first);
        if (task.required_sla != SLA3 && task.target_completion > task.arrival)
            instant = std::max(instant, double(now - task.arrival) /
                                        double(task.target_completion - task.arrival));
    }
    // Trade power for latency headroom; violations restore full power.
    if (average > kLatencyTarget) { power_limit = 1; emergency_until = now + kEmergencyHoldUs; }
    else if (now < emergency_until || instant > kMaxPowerLatencyRatio * kLatencyTarget) power_limit = 1;
    else if (instant > kLatencyTarget) power_limit += kPowerIncreaseStep;
    else if (recent && instant < kLargeDecreaseLatencyRatio * kLatencyTarget) power_limit -= kLargePowerDecreaseStep;
    else if (recent && instant < kSmallDecreaseLatencyRatio * kLatencyTarget) power_limit -= kSmallPowerDecreaseStep;
    power_limit = std::clamp(power_limit, kMinimumPowerFraction, 1.0);

    for (unsigned m = 0; m < Machine_GetTotal(); ++m) {
        const auto host = Machine_GetInfo(m);
        unsigned best = FastestState(m);
        if (best >= host.p_states.size()) continue;
        // Map the continuous power limit to a supported host state.
        const double budget = host.p_states[best] * power_limit;
        for (unsigned p = 0; p < std::min({size_t(P_STATES), host.p_states.size(), host.performance.size()}); ++p)
            if (host.performance[p] > 0 && host.p_states[p] >= budget &&
                host.p_states[p] < host.p_states[best]) best = p;
        // shortcut: round power upward to a supported state; use RAPL if finer control becomes available.
        SetPerformance(m, CPUPerformance_t(best));
    }
}
