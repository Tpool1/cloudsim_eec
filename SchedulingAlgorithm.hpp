#ifndef CLOUDSIM_SCHEDULING_ALGORITHM_HPP
#define CLOUDSIM_SCHEDULING_ALGORITHM_HPP

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include "Interfaces.h"

// Generated with OpenAI Codex; literature and simulator adaptations: ALGORITHMS.md.
class SchedulingAlgorithm {
public:
    virtual ~SchedulingAlgorithm() = default;
    virtual const char* Name() const = 0;
    virtual void Init();
    virtual void NewTask(Time_t now, TaskId_t task);
    virtual void TaskComplete(Time_t now, TaskId_t task);
    virtual void PeriodicCheck(Time_t now);
    virtual void MigrationComplete(Time_t now, VMId_t vm);
    virtual void StateChangeComplete(Time_t now, MachineId_t machine);
    virtual void MemoryWarning(Time_t now, MachineId_t machine);
    virtual void SLAWarning(Time_t now, TaskId_t task);
    virtual void Shutdown(Time_t now);

protected:
    virtual MachineId_t SelectMachine(Time_t now, const TaskInfo_t& task,
                                     const vector<MachineId_t>& candidates) const;
    virtual Priority_t TaskPriority(const TaskInfo_t& task) const;
    virtual bool HasCapacity(Time_t now, const TaskInfo_t& task, MachineId_t machine) const;
    virtual void UpdatePower(Time_t) {}
    void Dispatch(Time_t now, size_t start = 0);
    bool Eligible(const TaskInfo_t& task, MachineId_t machine) const;
    vector<TaskId_t> TasksOn(MachineId_t machine) const;
    double Work(const TaskInfo_t& task, const MachineInfo_t& host) const;
    double CompletionEstimate(const TaskInfo_t& task, MachineId_t machine) const;
    double RequiredRate(Time_t now, MachineId_t machine,
                        const TaskInfo_t* extra = nullptr) const;
    CPUPerformance_t FastestState(MachineId_t machine) const;
    CPUPerformance_t StateForRate(MachineId_t machine, double rate) const;
    void SetPerformance(MachineId_t machine, CPUPerformance_t state);
    void RequestState(MachineId_t machine, MachineState_t state);
    void ReleaseIdleVMs();

    vector<VMId_t> vms;
    vector<TaskId_t> waiting;
    std::map<TaskId_t, VMId_t> assignments;
    std::map<MachineId_t, MachineState_t> transitions;
};

class Pegasus final : public SchedulingAlgorithm {
public:
    const char* Name() const override { return "Pegasus"; }
    void Init() override;
    void TaskComplete(Time_t now, TaskId_t task) override;
    void SLAWarning(Time_t now, TaskId_t task) override;
protected:
    void UpdatePower(Time_t now) override;
private:
    struct Sample { Time_t time; double latency; };
    std::deque<Sample> samples;
    Time_t last_control = 0, emergency_until = 0;
    double power_limit = 1;
};

class EECO final : public SchedulingAlgorithm {
public:
    const char* Name() const override { return "E-ECO"; }
    void Init() override;
protected:
    bool HasCapacity(Time_t now, const TaskInfo_t& task, MachineId_t machine) const override;
    MachineId_t SelectMachine(Time_t now, const TaskInfo_t& task,
                             const vector<MachineId_t>& candidates) const override;
    void UpdatePower(Time_t now) override;
private:
    unsigned previous_running[4] = {};
};

class YDS final : public SchedulingAlgorithm {
public:
    const char* Name() const override { return "YDS (replanned backlog)"; }
    // Times in microseconds; work in instructions; speed in instructions/us (= MIPS).
    struct Job { TaskId_t id; double release, deadline, work; };
    struct Segment { TaskId_t task; double begin, end, speed; };
    static vector<Segment> BuildSchedule(vector<Job> jobs);
protected:
    void UpdatePower(Time_t now) override;
};

class CustomAlgorithm final : public SchedulingAlgorithm {
public:
    const char* Name() const override { return "Custom template"; }
protected:
    MachineId_t SelectMachine(Time_t now, const TaskInfo_t& task,
                             const vector<MachineId_t>& candidates) const override;
    Priority_t TaskPriority(const TaskInfo_t& task) const override;
    void UpdatePower(Time_t now) override;
};

std::unique_ptr<SchedulingAlgorithm> CreateSchedulingAlgorithm(const string& name);

#endif
