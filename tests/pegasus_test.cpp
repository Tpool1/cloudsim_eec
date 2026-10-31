#include "../Scheduler.cpp"
#include <cassert>

static MachineInfo_t machine;
MachineInfo_t Machine_GetInfo(MachineId_t) { return machine; }
void Machine_SetCorePerformance(MachineId_t, unsigned, CPUPerformance_t state) {
    machine.p_state = state;
}

int main() {
    machine.num_cpus = 2;
    machine.p_states = {12, 8, 6, 4};
    machine.p_state = P0;
    LatencyMonitor monitor;
    Controller controller;
    TaskInfo_t task{};
    task.target_completion = 1000;
    task.required_sla = SLA0;
    for (Time_t now = 5000000; now <= 60000000; now += 5000000) {
        task.arrival = now - 400;
        task.target_completion = task.arrival + 1000;
        monitor.Record(now, task);
        controller.Check(now, monitor, {0});
    }
    assert(machine.p_state == P1);
    task.arrival = 65000000 - 1500;
    task.target_completion = task.arrival + 1000;
    monitor.Record(65000000, task);
    controller.Check(65000000, monitor, {0});
    assert(machine.p_state == P0);
    controller.Check(70000000, monitor, {0});
    assert(machine.p_state == P0);

    LatencyMonitor emergency_monitor;
    Controller emergency_controller;
    task.arrival = 5000000 - 1100;
    task.target_completion = task.arrival + 1000;
    emergency_monitor.Record(5000000, task);
    emergency_controller.Check(5000000, emergency_monitor, {0});
    for (Time_t now = 10000000; now <= 80000000; now += 5000000) {
        task.arrival = now - 400;
        task.target_completion = task.arrival + 1000;
        emergency_monitor.Record(now, task);
        emergency_controller.Check(now, emergency_monitor, {0});
    }
    assert(machine.p_state == P0);

    LatencyMonitor ignored;
    task.required_sla = SLA3;
    ignored.Record(65000000, task);
    double average, instantaneous;
    assert(!ignored.Read(65000000, average, instantaneous));
}
