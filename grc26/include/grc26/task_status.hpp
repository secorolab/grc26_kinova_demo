#ifndef TASK_STATUS_HPP
#define TASK_STATUS_HPP

#include <mutex>
#include <chrono>
#include <cstdint>

#include "unique_identifier_msgs/msg/uuid.hpp"

struct TaskStatusData
{
    unique_identifier_msgs::msg::UUID bhv_ctx_id;
    bool goal_in = false;
    bool task_completed = false;
    bool is_obj_located_at_pick_location = false;
    bool is_obj_held_by_robot = false;
    bool is_obj_located_at_place_location = false;
    bool is_pick_start = false;
    bool is_pick_end = false;
    bool is_place_start = false;
    bool is_place_end = false;

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
    
    bool consumeLatest(TaskStatusData& out)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        out = current_;
        current_.is_pick_start = false;
        current_.is_pick_end = false;
        current_.is_place_start = false;
        current_.is_place_end = false;
        return true;
    }

private:
    mutable std::mutex mutex_;
    TaskStatusData current_;
    uint64_t sequence_counter_ = 0;
};

#endif // TASK_STATUS_HPP