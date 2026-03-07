#ifndef SINE_PROFILE_HPP
#define SINE_PROFILE_HPP

#include "kdl/frames.hpp"

class SineProfile {
public:
    SineProfile(KDL::Frame start_pose, KDL::Frame end_pose, double amplitude);

    KDL::Frame pos(double s);
    KDL::Twist vel(double s, double sd);

private:
    KDL::Frame start_pose_;
    KDL::Frame end_pose_;
    double amplitude_;
};

#endif // SINE_PROFILE_HPP