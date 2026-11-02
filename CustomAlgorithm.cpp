#include "SchedulingAlgorithm.hpp"

namespace {
// TUNING PARAMETERS: priority assigned to each SLA class; edit here and rebuild.
constexpr Priority_t kSla0Priority = HIGH_PRIORITY;
constexpr Priority_t kSla1Priority = MID_PRIORITY;
constexpr Priority_t kSla2Priority = MID_PRIORITY;
constexpr Priority_t kSla3Priority = LOW_PRIORITY;
// Add constants for your own placement/power policy here as you implement its TODOs.
// Baseline placement uses CompletionEstimate; baseline power always uses the fastest state.
static_assert(kSla0Priority >= HIGH_PRIORITY && kSla0Priority <= LOW_PRIORITY &&
              kSla1Priority >= HIGH_PRIORITY && kSla1Priority <= LOW_PRIORITY &&
              kSla2Priority >= HIGH_PRIORITY && kSla2Priority <= LOW_PRIORITY &&
              kSla3Priority >= HIGH_PRIORITY && kSla3Priority <= LOW_PRIORITY,
              "Custom priorities must be HIGH_PRIORITY, MID_PRIORITY, or LOW_PRIORITY");
}

MachineId_t CustomAlgorithm::SelectMachine(Time_t now, const TaskInfo_t& task,
                                           const vector<MachineId_t>& candidates) const {
    // TODO: Return one of candidates using your own placement rule.
    return SchedulingAlgorithm::SelectMachine(now, task, candidates);
}

Priority_t CustomAlgorithm::TaskPriority(const TaskInfo_t& task) const {
    // TODO: Assign a priority, taking account of starvation of lower priorities.
    if (task.required_sla == SLA0) return kSla0Priority;
    if (task.required_sla == SLA1) return kSla1Priority;
    return task.required_sla == SLA3 ? kSla3Priority : kSla2Priority;
}

void CustomAlgorithm::UpdatePower(Time_t /* now */) {
    // TODO: Inspect TasksOn(machine), RequiredRate(now, machine), and Machine_GetInfo.
    // Use SetPerformance and RequestState so readiness and active-task checks are preserved.
    // This runnable baseline uses maximum performance and leaves idle machines awake.
    for (unsigned m = 0; m < Machine_GetTotal(); ++m) SetPerformance(m, FastestState(m));
}
