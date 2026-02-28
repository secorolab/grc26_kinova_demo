#include "grc26/task_state.hpp"

void TaskState::update(const TaskStatusData& new_state)
{
    std::lock_guard<std::mutex> lock(mutex_);

    current_ = new_state;
    current_.sequence_number = sequence_counter_++;
    current_.timestamp = std::chrono::high_resolution_clock::now();
}

bool TaskState::getLatest(TaskStatusData& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    out = current_;
    return true;
}