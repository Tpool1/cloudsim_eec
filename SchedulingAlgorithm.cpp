#include "SchedulingAlgorithm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// SHARED TUNING: priorities used by Pegasus and E-ECO (YDS/custom have their own maps).
constexpr Priority_t kSla0Priority = HIGH_PRIORITY;
constexpr Priority_t kSla1Priority = MID_PRIORITY;
constexpr Priority_t kSla2Priority = MID_PRIORITY;
constexpr Priority_t kSla3Priority = LOW_PRIORITY;
static_assert(kSla0Priority >= HIGH_PRIORITY && kSla0Priority <= LOW_PRIORITY &&
              kSla1Priority >= HIGH_PRIORITY && kSla1Priority <= LOW_PRIORITY &&
              kSla2Priority >= HIGH_PRIORITY && kSla2Priority <= LOW_PRIORITY &&
              kSla3Priority >= HIGH_PRIORITY && kSla3Priority <= LOW_PRIORITY, "Invalid shared task priorities");

// SIMULATOR CALIBRATION: not a policy knob; change only if the simulator's GPU model changes.
constexpr double kGpuSpeedup = 20.0;

bool ValidVM(const TaskInfo_t& task) {
    return (task.required_vm == LINUX || task.required_vm == LINUX_RT) ||
           (task.required_vm == WIN && (task.required_cpu == ARM || task.required_cpu == X86)) ||
           (task.required_vm == AIX && task.required_cpu == POWER);
}
}

std::unique_ptr<SchedulingAlgorithm> CreateSchedulingAlgorithm(const string& name) {
    if (name == "pegasus") return std::make_unique<Pegasus>();
    if (name == "eeco") return std::make_unique<EECO>();
    if (name == "yds") return std::make_unique<YDS>();
    if (name == "custom") return std::make_unique<CustomAlgorithm>();
    throw std::invalid_argument("Unknown CLOUDSIM_ALGORITHM '" + name +
                                "'; use pegasus, eeco, yds, or custom");
}

void SchedulingAlgorithm::Init() {
    vms.clear();
    waiting.clear();
    assignments.clear();
    transitions.clear();
    SimOutput(string("Scheduling algorithm: ") + Name(), 0);
}

CPUPerformance_t SchedulingAlgorithm::FastestState(MachineId_t machine) const {
    const auto host = Machine_GetInfo(machine);
    unsigned best = 0;
    for (unsigned p = 1; p < std::min(size_t(P_STATES), host.performance.size()); ++p)
        if (host.performance[p] > host.performance[best]) best = p;
    return CPUPerformance_t(best);
}

CPUPerformance_t SchedulingAlgorithm::StateForRate(MachineId_t machine, double rate) const {
    const auto host = Machine_GetInfo(machine);
    unsigned best = FastestState(machine);
    for (unsigned p = 0; p < std::min(size_t(P_STATES), host.performance.size()); ++p)
        if (host.performance[p] > 0 && host.performance[p] >= rate &&
            host.performance[p] < host.performance[best]) best = p;
    return CPUPerformance_t(best);
}

void SchedulingAlgorithm::SetPerformance(MachineId_t machine, CPUPerformance_t state) {
    const auto host = Machine_GetInfo(machine);
    if (host.s_state == S0 && !transitions.count(machine) && host.p_state != state &&
        unsigned(state) < host.performance.size() && host.performance[state] > 0)
        Machine_SetCorePerformance(machine, 0, state); // The public API sets all cores.
}

void SchedulingAlgorithm::RequestState(MachineId_t machine, MachineState_t state) {
    const auto host = Machine_GetInfo(machine);
    if (transitions.count(machine) || host.s_state == state) return;
    if (state != S0 && (host.active_tasks || !TasksOn(machine).empty())) return;
    transitions[machine] = state; // A transition can complete through a synchronous upcall.
    Machine_SetState(machine, state);
}

vector<TaskId_t> SchedulingAlgorithm::TasksOn(MachineId_t machine) const {
    vector<TaskId_t> tasks;
    for (auto vm : vms) {
        const auto guest = VM_GetInfo(vm);
        if (guest.machine_id == machine)
            for (auto id : guest.active_tasks)
                if (!GetTaskInfo(id).completed) tasks.push_back(id);
    }
    return tasks;
}

double SchedulingAlgorithm::Work(const TaskInfo_t& task, const MachineInfo_t& host) const {
    // This simulator's GPU model accelerates a GPU-capable task by 20x.
    return double(task.remaining_instructions) / (task.gpu_capable && host.gpus ? kGpuSpeedup : 1);
}

bool SchedulingAlgorithm::Eligible(const TaskInfo_t& task, MachineId_t machine) const {
    const auto host = Machine_GetInfo(machine);
    if (host.cpu != task.required_cpu || !host.num_cpus || host.performance.empty() ||
        !host.performance[FastestState(machine)] || host.memory_used > host.memory_size)
        return false;
    unsigned overhead = VM_MEMORY_OVERHEAD;
    for (auto vm : vms) {
        const auto guest = VM_GetInfo(vm);
        if (guest.machine_id == machine && guest.vm_type == task.required_vm &&
            guest.cpu == task.required_cpu) overhead = 0;
    }
    return uint64_t(task.required_memory) + overhead <= host.memory_size - host.memory_used;
}

double SchedulingAlgorithm::CompletionEstimate(const TaskInfo_t& task, MachineId_t machine) const {
    const auto host = Machine_GetInfo(machine);
    unsigned state = host.p_state;
    if (state >= host.performance.size() || host.performance[state] == 0)
        state = FastestState(machine);
    double work = Work(task, host), total = work;
    for (auto id : TasksOn(machine)) total += Work(GetTaskInfo(id), host);
    // shortcut: fluid sharing ignores hypervisor slices and priorities; use a queue model for tighter estimates.
    return std::max(work, total / host.num_cpus) / host.performance[state];
}

double SchedulingAlgorithm::RequiredRate(Time_t now, MachineId_t machine,
                                         const TaskInfo_t* extra) const {
    const auto host = Machine_GetInfo(machine);
    vector<TaskInfo_t> tasks;
    for (auto id : TasksOn(machine)) tasks.push_back(GetTaskInfo(id));
    if (extra) tasks.push_back(*extra);
    std::sort(tasks.begin(), tasks.end(), [](const auto& a, const auto& b) {
        return a.target_completion < b.target_completion;
    });
    // Each deadline must cover its own work and all earlier-deadline work.
    double cumulative = 0, rate = 0;
    for (const auto& task : tasks) {
        if (task.required_sla == SLA3) continue;
        if (task.target_completion <= now) return std::numeric_limits<double>::infinity();
        const double time = double(task.target_completion - now);
        const double work = Work(task, host);
        cumulative += work;
        rate = std::max({rate, work / time, cumulative / (host.num_cpus * time)});
    }
    return rate;
}

MachineId_t SchedulingAlgorithm::SelectMachine(Time_t, const TaskInfo_t& task,
                                               const vector<MachineId_t>& candidates) const {
    return *std::min_element(candidates.begin(), candidates.end(), [&](auto a, auto b) {
        return CompletionEstimate(task, a) < CompletionEstimate(task, b);
    });
}

Priority_t SchedulingAlgorithm::TaskPriority(const TaskInfo_t& task) const {
    if (task.required_sla == SLA0) return kSla0Priority;
    if (task.required_sla == SLA1) return kSla1Priority;
    return task.required_sla == SLA3 ? kSla3Priority : kSla2Priority;
}

bool SchedulingAlgorithm::HasCapacity(Time_t, const TaskInfo_t&, MachineId_t) const {
    return true;
}

void SchedulingAlgorithm::NewTask(Time_t now, TaskId_t id) {
    ReleaseIdleVMs();
    const auto task = GetTaskInfo(id);
    bool possible = false;
    for (unsigned m = 0; m < Machine_GetTotal(); ++m) {
        const auto host = Machine_GetInfo(m);
        if (host.cpu == task.required_cpu && host.num_cpus && !host.performance.empty() &&
            host.performance[FastestState(m)] &&
            uint64_t(task.required_memory) + VM_MEMORY_OVERHEAD <= host.memory_size)
            possible = true;
    }
    if (!ValidVM(task) || !possible || task.completed || task.arrival > now)
        throw std::runtime_error("Cannot schedule task " + std::to_string(id) +
                                 ": invalid requirements or no compatible host capacity");
    if (assignments.count(id) || std::find(waiting.begin(), waiting.end(), id) != waiting.end())
        throw std::runtime_error("Duplicate task arrival: " + std::to_string(id));
    waiting.push_back(id);
    // An arrival cannot free resources for older waiters; completion/state/timer events retry those.
    Dispatch(now, waiting.size() - 1);
    UpdatePower(now);
}

void SchedulingAlgorithm::Dispatch(Time_t now, size_t start) {
    for (auto it = waiting.begin() + start; it != waiting.end();) {
        const auto task = GetTaskInfo(*it);
        vector<MachineId_t> candidates, overloaded;
        for (unsigned m = 0; m < Machine_GetTotal(); ++m)
            if (!transitions.count(m) && Machine_GetInfo(m).s_state == S0 && Eligible(task, m)) {
                if (HasCapacity(now, task, m)) candidates.push_back(m);
                else overloaded.push_back(m);
            }
        if (candidates.empty()) {
            // Prefer a warm standby over a cold boot, and wait for any in-flight wakeup.
            bool waking = false;
            for (const auto& change : transitions)
                if (Eligible(task, change.first) && HasCapacity(now, task, change.first)) waking = true;
            MachineId_t wake = Machine_GetTotal();
            for (unsigned m = 0; !waking && m < Machine_GetTotal(); ++m)
                if (!transitions.count(m) && Eligible(task, m) && HasCapacity(now, task, m) &&
                    Machine_GetInfo(m).s_state != S0 &&
                    (wake == Machine_GetTotal() ||
                     Machine_GetInfo(m).s_state < Machine_GetInfo(wake).s_state)) wake = m;
            if (wake != Machine_GetTotal()) RequestState(wake, S0);
            if (wake != Machine_GetTotal() || waking || overloaded.empty()) {
                ++it;
                continue;
            }
            candidates = overloaded; // An infeasible deadline must still make progress at full capacity.
        }
        const auto machine = SelectMachine(now, task, candidates);
        if (std::find(candidates.begin(), candidates.end(), machine) == candidates.end())
            throw std::runtime_error("Algorithm selected an ineligible machine");
        // Reuse a compatible VM before creating one on the chosen host.
        VMId_t selected = 0;
        bool found = false;
        for (auto vm : vms) {
            const auto guest = VM_GetInfo(vm);
            if (guest.machine_id == machine && guest.cpu == task.required_cpu &&
                guest.vm_type == task.required_vm) { selected = vm; found = true; break; }
        }
        if (!found) {
            selected = VM_Create(task.required_vm, task.required_cpu);
            VM_Attach(selected, machine);
            vms.push_back(selected);
        }
        VM_AddTask(selected, *it, TaskPriority(task));
        assignments[*it] = selected;
        it = waiting.erase(it);
    }
}

void SchedulingAlgorithm::ReleaseIdleVMs() {
    for (auto it = vms.begin(); it != vms.end();) {
        if (VM_GetInfo(*it).active_tasks.empty()) {
            VM_Shutdown(*it);
            it = vms.erase(it);
        } else ++it;
    }
}

void SchedulingAlgorithm::TaskComplete(Time_t, TaskId_t task) {
    assignments.erase(task);
    // The simulator still owns the completing core here; dispatching can run that core twice.
}

void SchedulingAlgorithm::PeriodicCheck(Time_t now) {
    ReleaseIdleVMs();
    Dispatch(now);
    UpdatePower(now);
}
void SchedulingAlgorithm::MigrationComplete(Time_t now, VMId_t) { Dispatch(now); UpdatePower(now); }
void SchedulingAlgorithm::StateChangeComplete(Time_t now, MachineId_t machine) {
    transitions.erase(machine);
    Dispatch(now);
    UpdatePower(now);
}

void SchedulingAlgorithm::MemoryWarning(Time_t, MachineId_t machine) {
    SimOutput("Memory overcommit on machine " + std::to_string(machine), 0);
    SetPerformance(machine, FastestState(machine));
}

void SchedulingAlgorithm::SLAWarning(Time_t, TaskId_t task) {
    auto entry = assignments.find(task);
    if (entry != assignments.end() && !GetTaskInfo(task).completed) {
        SetTaskPriority(task, HIGH_PRIORITY);
        auto machine = VM_GetInfo(entry->second).machine_id;
        SetPerformance(machine, FastestState(machine));
    }
}

void SchedulingAlgorithm::Shutdown(Time_t) {
    if (!waiting.empty() || !assignments.empty())
        throw std::runtime_error("Simulation ended with unfinished scheduler tasks");
    ReleaseIdleVMs();
}
