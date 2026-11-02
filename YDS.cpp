#include "SchedulingAlgorithm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// TUNING PARAMETERS for the simulator adapter; the offline YDS algorithm is parameter-free.
constexpr double kSpeedSafetyFactor = 1.0; // >= 1; 1.20 requests 20% extra MIPS before P-state rounding.
constexpr double kBestEffortHorizonScale = 1.0; // > 0; multiplies slowest-state runtime for SLA3's synthetic deadline.
constexpr double kMinimumHorizonUs = 1.0; // > 0 simulated microseconds; prevents a zero-length synthetic window.
constexpr Priority_t kEarliestDeadlinePriority = HIGH_PRIORITY; // Earliest one task per core.
constexpr Priority_t kOtherDeadlinePriority = MID_PRIORITY;    // Remaining tasks with deadlines.
constexpr Priority_t kBestEffortPriority = LOW_PRIORITY;       // SLA3 tasks.

// NUMERICAL CONSTANTS: floating-point epsilon multipliers, not energy/SLA tuning knobs.
constexpr double kCompletionToleranceMultiplier = 32;
constexpr double kBoundaryToleranceMultiplier = 64;

static_assert(kSpeedSafetyFactor >= 1 && kSpeedSafetyFactor <= std::numeric_limits<double>::max() &&
              kBestEffortHorizonScale > 0 && kBestEffortHorizonScale <= std::numeric_limits<double>::max() &&
              kMinimumHorizonUs > 0 && kMinimumHorizonUs <= std::numeric_limits<double>::max(),
              "YDS speed/horizon parameters must be finite and positive; speed factor must be >= 1");
static_assert(kEarliestDeadlinePriority >= HIGH_PRIORITY && kEarliestDeadlinePriority <= LOW_PRIORITY &&
              kOtherDeadlinePriority >= HIGH_PRIORITY && kOtherDeadlinePriority <= LOW_PRIORITY &&
              kBestEffortPriority >= HIGH_PRIORITY && kBestEffortPriority <= LOW_PRIORITY,
              "Invalid YDS priority bands");
}

// Yao, Demers and Shenker, FOCS 1995, section 3: critical intervals and time compression.
vector<YDS::Segment> YDS::BuildSchedule(vector<Job> jobs) {
    for (const auto& job : jobs)
        if (!std::isfinite(job.release) || !std::isfinite(job.deadline) ||
            !std::isfinite(job.work) || job.work < 0 || job.deadline <= job.release)
            throw std::invalid_argument("YDS requires finite work >= 0 and release < deadline");
    jobs.erase(std::remove_if(jobs.begin(), jobs.end(), [](const auto& job) {
        return job.work == 0;
    }), jobs.end());
    if (jobs.empty()) return {};

    // Find the interval with the greatest work per unit time.
    vector<double> releases;
    for (const auto& job : jobs) releases.push_back(job.release);
    std::sort(releases.begin(), releases.end());
    releases.erase(std::unique(releases.begin(), releases.end()), releases.end());
    std::sort(jobs.begin(), jobs.end(), [](const auto& a, const auto& b) {
        return a.deadline < b.deadline;
    });
    double start = 0, end = 0, density = -1;
    for (double release : releases) {
        double work = 0;
        for (const auto& job : jobs) {
            if (job.release < release) continue;
            work += job.work;
            const double rate = work / (job.deadline - release);
            if (!std::isfinite(rate)) throw std::invalid_argument("YDS density exceeds numeric range");
            if (rate > density) { density = rate; start = release; end = job.deadline; }
        }
    }

    // Remove the critical interval and compress the remaining time.
    vector<Job> critical, rest;
    const double length = end - start;
    const auto compress = [=](double t) {
        return t <= start ? t : (t >= end ? t - length : start);
    };
    for (auto job : jobs) {
        if (job.release >= start && job.deadline <= end) critical.push_back(job);
        else {
            job.release = compress(job.release);
            job.deadline = compress(job.deadline);
            rest.push_back(job);
        }
    }

    // Run critical jobs at the interval density, earliest deadline first.
    vector<Segment> schedule;
    double time = start;
    while (!critical.empty()) {
        auto next = critical.end();
        double arrival = std::numeric_limits<double>::infinity();
        for (auto it = critical.begin(); it != critical.end(); ++it) {
            if (it->release > time) arrival = std::min(arrival, it->release);
            else if (next == critical.end() || it->deadline < next->deadline) next = it;
        }
        if (next == critical.end()) { time = arrival; continue; }
        const double finish = time + next->work / density;
        const double until = std::min(finish, arrival);
        schedule.push_back({next->id, time, until, density});
        const double tolerance = kCompletionToleranceMultiplier * std::numeric_limits<double>::epsilon() *
                                 std::max(1.0, std::abs(finish));
        if (until == finish || finish - until <= tolerance) critical.erase(next);
        else next->work -= (until - time) * density;
        time = until;
    }

    // Recursively schedule the rest, then restore the removed interval.
    for (auto segment : BuildSchedule(rest)) {
        const double tolerance = kBoundaryToleranceMultiplier * std::numeric_limits<double>::epsilon() *
                                 std::max(1.0, std::abs(start));
        if (std::abs(segment.begin - start) <= tolerance) segment.begin = start;
        if (std::abs(segment.end - start) <= tolerance) segment.end = start;
        if (segment.end <= start) schedule.push_back(segment);
        else if (segment.begin >= start) {
            segment.begin += length;
            segment.end += length;
            schedule.push_back(segment);
        } else {
            schedule.push_back({segment.task, segment.begin, start, segment.speed});
            schedule.push_back({segment.task, end, segment.end + length, segment.speed});
        }
    }
    std::sort(schedule.begin(), schedule.end(), [](const auto& a, const auto& b) {
        return a.begin < b.begin;
    });
    return schedule;
}

void YDS::UpdatePower(Time_t now) {
    for (unsigned m = 0; m < Machine_GetTotal(); ++m) {
        const auto host = Machine_GetInfo(m);
        if (host.s_state != S0 || !host.num_cpus || host.performance.empty()) continue;
        auto tasks = TasksOn(m);
        vector<Job> jobs;
        const unsigned slow = StateForRate(m, 0);
        for (auto id : tasks) {
            const auto task = GetTaskInfo(id);
            const double work = Work(task, host);
            if (!work) continue;
            double deadline = double(task.target_completion);
            // Give best-effort and overdue work a finite planning horizon.
            if (task.required_sla == SLA3 || task.target_completion <= now)
                deadline = double(now) + std::max(kMinimumHorizonUs,
                    kBestEffortHorizonScale * work / host.performance[slow]);
            // shortcut: use a fluid multicore host; exact pinning/EDF needs a richer simulator API.
            jobs.push_back({id, double(now), deadline, work / host.num_cpus});
        }
        // Use the current segment’s speed, bounded by the shared deadline demand.
        double rate = RequiredRate(now, m);
        for (const auto& segment : BuildSchedule(jobs))
            if (segment.begin <= double(now) && double(now) < segment.end)
                rate = std::max(rate, segment.speed);
        SetPerformance(m, StateForRate(m, kSpeedSafetyFactor * rate));

        std::sort(tasks.begin(), tasks.end(), [](auto a, auto b) {
            const auto left = GetTaskInfo(a), right = GetTaskInfo(b);
            if ((left.required_sla == SLA3) != (right.required_sla == SLA3))
                return left.required_sla != SLA3;
            return left.target_completion < right.target_completion;
        });
        // The simulator exposes three priority bands, rather than an EDF dispatch primitive.
        for (unsigned i = 0; i < tasks.size(); ++i) {
            const auto task = GetTaskInfo(tasks[i]);
            const auto priority = task.required_sla == SLA3 ? kBestEffortPriority :
                                  (i < host.num_cpus ? kEarliestDeadlinePriority : kOtherDeadlinePriority);
            if (task.priority != priority) SetTaskPriority(tasks[i], priority);
        }
    }
}
