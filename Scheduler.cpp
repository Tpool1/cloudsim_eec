//
//  Scheduler.cpp
//  CloudSim
//
//  Created by ELMOOTAZBELLAH ELNOZAHY on 10/20/24.
//

#include <cstdint>
#include "Scheduler.hpp"
#include <algorithm>
#include <deque>
#include <limits>

static bool migrating = false;
static unsigned active_machines = 16;

// TODO: double check these classes
class LatencyMonitor {
    struct Sample { Time_t time; double latency; };
    deque<Sample> samples;
    double total = 0;
    double recent_total = 0;
    unsigned recent_count = 0;
public:
    void Record(Time_t now, const TaskInfo_t& task) {
        if (task.required_sla == SLA3 || task.target_completion <= task.arrival || now < task.arrival)
            return;
        double latency = double(now - task.arrival) / (task.target_completion - task.arrival);
        samples.push_back({now, latency});
        total += latency;
        recent_total += latency;
        ++recent_count;
    }

    bool Read(Time_t now, double& average, double& instantaneous) {
        while (!samples.empty() && now - samples.front().time > 30000000) {
            total -= samples.front().latency;
            samples.pop_front();
        }
        bool available = !samples.empty() && recent_count != 0;
        if (available) {
            average = total / samples.size();
            instantaneous = recent_total / recent_count;
        }
        recent_total = 0;
        recent_count = 0;
        return available;
    }
};

class Controller {
    double power = 100;
    Time_t last_check = 0;
    Time_t emergency_until = 0;
public:
    void Check(Time_t now, LatencyMonitor& monitor, const vector<MachineId_t>& machines) {
        if (now - last_check < 5000000) return;
        last_check = now;
        double average, instantaneous;
        bool available = monitor.Read(now, average, instantaneous);
        // shortcut: normalized task deadlines approximate a shared SLO, calibrate for a real workload.
        const double target = 0.95;
        if (available && average > target) {
            power = 100;
            emergency_until = now + 300000000;
        } else if (now < emergency_until || !available || instantaneous > 1.35 * target) {
            power = 100;
        } else if (instantaneous > target) {
            power = min(100.0, power + 7);
        } else if (instantaneous < 0.60 * target) {
            power = max(0.0, power - 3);
        } else if (instantaneous < 0.85 * target) {
            power = max(0.0, power - 1);
        }

        for (MachineId_t machine : machines) {
            MachineInfo_t info = Machine_GetInfo(machine);
            if (info.p_states.empty() || info.p_states[0] == 0) continue;
            // shortcut: discrete P-states approximate RAPL, upgrade when a power-cap API exists.
            unsigned state = 0;
            for (unsigned i = 1; i < min<unsigned>(P_STATES, info.p_states.size()); ++i)
                if (100.0 * info.p_states[i] >= power * info.p_states[0]) state = i;
            if (info.p_state != static_cast<CPUPerformance_t>(state))
                for (unsigned core = 0; core < info.num_cpus; ++core)
                    Machine_SetCorePerformance(machine, core, static_cast<CPUPerformance_t>(state));
        }
    }
};

static LatencyMonitor latency_monitor;
static Controller controller;

void Scheduler::Init() {
    // Find the parameters of the clusters
    // Get the total number of machines
    // For each machine:
    //      Get the type of the machine
    //      Get the memory of the machine
    //      Get the number of CPUs
    //      Get if there is a GPU or not
    // 
    SimOutput("Scheduler::Init(): Total number of machines is " + to_string(Machine_GetTotal()), 3);
    SimOutput("Scheduler::Init(): Initializing scheduler", 1);
    for(unsigned i = 0; i < active_machines; i++)
        vms.push_back(VM_Create(LINUX, X86));
    for(unsigned i = 0; i < active_machines; i++) {
        machines.push_back(MachineId_t(i));
    }    
    for(unsigned i = 0; i < active_machines; i++) {
        VM_Attach(vms[i], machines[i]);
    }

    bool dynamic = false;
    if(dynamic)
        for(unsigned i = 0; i<4 ; i++)
            for(unsigned j = 0; j < 8; j++)
                Machine_SetCorePerformance(MachineId_t(0), j, P3);
    // Turn off the ARM machines
    for(unsigned i = 24; i < Machine_GetTotal(); i++)
        Machine_SetState(MachineId_t(i), S5);

    SimOutput("Scheduler::Init(): VM ids are " + to_string(vms[0]) + " ahd " + to_string(vms[1]), 3);
}

void Scheduler::MigrationComplete(Time_t time, VMId_t vm_id) {
    // Update your data structure. The VM now can receive new tasks
}

void Scheduler::NewTask(Time_t now, TaskId_t task_id) {
    Priority_t priority = (task_id == 0 || task_id == 64)? HIGH_PRIORITY : MID_PRIORITY;
    TaskInfo_t task = GetTaskInfo(task_id);
    auto best_vm = vms.end();
    double lowest_work = numeric_limits<double>::infinity();
    for (unsigned offset = 0; offset < vms.size(); ++offset) {
        auto vm = vms.begin() + (task_id % vms.size() + offset) % vms.size();
        if (migrating && *vm == 1) continue;
        VMInfo_t info = VM_GetInfo(*vm);
        MachineInfo_t machine = Machine_GetInfo(info.machine_id);
        unsigned state = machine.p_state;
        if (info.vm_type != task.required_vm || info.cpu != task.required_cpu ||
            machine.cpu != task.required_cpu || machine.s_state != S0 ||
            machine.memory_used > machine.memory_size ||
            task.required_memory > machine.memory_size - machine.memory_used ||
            machine.num_cpus == 0 || state >= machine.performance.size() ||
            machine.performance[state] == 0)
            continue;

        double work = 0;
        // Include all VMs on the current host, including both VMs after migration.
        for (VMId_t other : vms) {
            VMInfo_t guest = VM_GetInfo(other);
            if (guest.machine_id != info.machine_id) continue;
            for (TaskId_t queued : guest.active_tasks) {
                TaskInfo_t pending = GetTaskInfo(queued);
                double rate = machine.performance[state] * 1e6;
                if (pending.gpu_capable && machine.gpus) rate *= 20;
                work += double(pending.remaining_instructions) / rate;
            }
        }
        work /= machine.num_cpus;
        if (work < lowest_work) {
            best_vm = vm;
            lowest_work = work;
        }
    }
    if (best_vm == vms.end()) {
        ThrowException("Scheduler::NewTask(): No eligible VM for task", task_id);
        return;
    }
    VM_AddTask(*best_vm, task_id, priority);
}

void Scheduler::PeriodicCheck(Time_t now) {
    // This method should be called from SchedulerCheck()
    // SchedulerCheck is called periodically by the simulator to allow you to monitor, make decisions, adjustments, etc.
    // Unlike the other invocations of the scheduler, this one doesn't report any specific event
    // Recommendation: Take advantage of this function to do some monitoring and adjustments as necessary
    controller.Check(now, latency_monitor, machines);
}

void Scheduler::Shutdown(Time_t time) {
    // Do your final reporting and bookkeeping here.
    // Report about the total energy consumed
    // Report about the SLA compliance
    // Shutdown everything to be tidy :-)
    for(auto & vm: vms) {
        VM_Shutdown(vm);
    }
    SimOutput("SimulationComplete(): Finished!", 4);
    SimOutput("SimulationComplete(): Time is " + to_string(time), 4);
}

void Scheduler::TaskComplete(Time_t now, TaskId_t task_id) {
    // Do any bookkeeping necessary for the data structures
    // Decide if a machine is to be turned off, slowed down, or VMs to be migrated according to your policy
    // This is an opportunity to make any adjustments to optimize performance/energy
    latency_monitor.Record(now, GetTaskInfo(task_id));
    SimOutput("Scheduler::TaskComplete(): Task " + to_string(task_id) + " is complete at " + to_string(now), 4);
}

// Public interface below

static Scheduler Scheduler;

void InitScheduler() {
    SimOutput("InitScheduler(): Initializing scheduler", 4);
    Scheduler.Init();
}

void HandleNewTask(Time_t time, TaskId_t task_id) {
    SimOutput("HandleNewTask(): Received new task " + to_string(task_id) + " at time " + to_string(time), 4);
    Scheduler.NewTask(time, task_id);
}

void HandleTaskCompletion(Time_t time, TaskId_t task_id) {
    SimOutput("HandleTaskCompletion(): Task " + to_string(task_id) + " completed at time " + to_string(time), 4);
    Scheduler.TaskComplete(time, task_id);
}

void MemoryWarning(Time_t time, MachineId_t machine_id) {
    // The simulator is alerting you that machine identified by machine_id is overcommitted
    SimOutput("MemoryWarning(): Overflow at " + to_string(machine_id) + " was detected at time " + to_string(time), 0);
}

void MigrationDone(Time_t time, VMId_t vm_id) {
    // The function is called on to alert you that migration is complete
    SimOutput("MigrationDone(): Migration of VM " + to_string(vm_id) + " was completed at time " + to_string(time), 4);
    Scheduler.MigrationComplete(time, vm_id);
    migrating = false;
}

void SchedulerCheck(Time_t time) {
    // This function is called periodically by the simulator, no specific event
    SimOutput("SchedulerCheck(): SchedulerCheck() called at " + to_string(time), 4);
    Scheduler.PeriodicCheck(time);
    static unsigned counts = 0;
    counts++;
    if(counts == 10) {
        migrating = true;
        VM_Migrate(1, 9);
    }
}

void SimulationComplete(Time_t time) {
    // This function is called before the simulation terminates Add whatever you feel like.
    cout << "SLA violation report" << endl;
    cout << "SLA0: " << GetSLAReport(SLA0) << "%" << endl;
    cout << "SLA1: " << GetSLAReport(SLA1) << "%" << endl;
    cout << "SLA2: " << GetSLAReport(SLA2) << "%" << endl;     // SLA3 do not have SLA violation issues
    cout << "Total Energy " << Machine_GetClusterEnergy() << "KW-Hour" << endl;
    cout << "Simulation run finished in " << double(time)/1000000 << " seconds" << endl;
    SimOutput("SimulationComplete(): Simulation finished at time " + to_string(time), 4);
    
    Scheduler.Shutdown(time);
}

void SLAWarning(Time_t time, TaskId_t task_id) {
    
}

void StateChangeComplete(Time_t time, MachineId_t machine_id) {
    // Called in response to an earlier request to change the state of a machine
}

