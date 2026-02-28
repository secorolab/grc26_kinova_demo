#ifndef TASK_STATE_HPP
#define TASK_STATE_HPP

#include <mutex>
#include <chrono>
#include <cstdint>

struct TaskStatusData
{
    bool idle = false;
    bool human_initiation = false;
    bool task_completion = false;
    bool obj_held_by_human = false;

    uint64_t sequence_number = 0;
    std::chrono::high_resolution_clock::time_point timestamp;
};

class TaskState
{
public:
    TaskState() = default;

    void update(const TaskStatusData& new_state);

    bool getLatest(TaskStatusData& out) const;

private:
    mutable std::mutex mutex_;
    TaskStatusData current_;
    uint64_t sequence_counter_ = 0;
};

#endif // TASK_STATE_HPP