#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <memory>
#include "kdl/frames.hpp"
#include "kdl/trajectory.hpp"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(
        KDL::Frame start_pose,
        KDL::Frame end_pose,
        double max_vel,
        double max_acc
    );

    KDL::Trajectory& get() { return *trajectory_; }

private:
    KDL::Frame start_pose_;
    KDL::Frame end_pose_;
    std::unique_ptr<KDL::Trajectory> trajectory_;

};

#endif // TRAJECTORY_GENERATOR_HPP