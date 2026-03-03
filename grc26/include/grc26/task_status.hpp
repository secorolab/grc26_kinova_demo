#ifndef TASK_STATUS_HPP
#define TASK_STATUS_HPP

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

class TaskStatus
{
public:
    TaskStatus() = default;

    void update(const TaskStatusData& new_status)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        current_ = new_status;
        current_.sequence_number = sequence_counter_++;
        current_.timestamp = std::chrono::high_resolution_clock::now();
    }

    bool getLatest(TaskStatusData& out) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        out = current_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    TaskStatusData current_;
    uint64_t sequence_counter_ = 0;
};

#endif // TASK_STATUS_HPP