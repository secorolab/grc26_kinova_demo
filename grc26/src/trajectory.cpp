#include "grc26/trajectory.hpp"
#include "kdl/path_circle.hpp"
#include "kdl/rotational_interpolation_sa.hpp"
#include "kdl/trajectory_segment.hpp"
#include "kdl/velocityprofile_trap.hpp"

TrajectoryGenerator::TrajectoryGenerator(
    KDL::Frame start_pose,
    KDL::Frame end_pose,
    double max_vel,
    double max_acc
)
    : start_pose_(start_pose), end_pose_(end_pose)
{
    // Build semi-circle path
    KDL::Vector center = (start_pose_.p + end_pose_.p) / 2.0;
    double radius = (end_pose_.p - start_pose_.p).Norm() / 2.0;
    KDL::Vector V_base_p = center + KDL::Vector(0.0, 0.0, radius);
    KDL::RotationalInterpolation_SingleAxis* orient = new KDL::RotationalInterpolation_SingleAxis();

    KDL::Path_Circle* path = new KDL::Path_Circle(
        start_pose_,
        center,
        V_base_p,
        end_pose_.M,
        M_PI,   // alpha
        orient,
        1.0,    // eqradius
        true    // aggregate
    );

    // Build velocity profile
    KDL::VelocityProfile_Trap* vel_profile = new KDL::VelocityProfile_Trap(max_vel, max_acc);
    vel_profile->SetProfile(0.0, path->PathLength());

    // Create trajectory
    trajectory_ = std::make_unique<KDL::Trajectory_Segment>(path, vel_profile);
}