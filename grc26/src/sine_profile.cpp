#include "grc26/sine_profile.hpp"
#include <cmath>

SineProfile::SineProfile(KDL::Frame start_pose, KDL::Frame end_pose, double amplitude)
    : start_pose_(start_pose), end_pose_(end_pose), amplitude_(amplitude)
{}

KDL::Frame SineProfile::pos(double s)
{
    double x = start_pose_.p.x() + s * (end_pose_.p.x() - start_pose_.p.x());
    double y = start_pose_.p.y() + s * (end_pose_.p.y() - start_pose_.p.y());
    double z = start_pose_.p.z() + amplitude_ * std::sin(M_PI * s);

    return KDL::Frame(start_pose_.M, KDL::Vector(x, y, z));
}

KDL::Twist SineProfile::vel(double s, double sd)
{
    double vx = (end_pose_.p.x() - start_pose_.p.x()) * sd;
    double vy = (end_pose_.p.y() - start_pose_.p.y()) * sd;
    double vz = amplitude_ * M_PI * std::cos(M_PI * s) * sd;

    return KDL::Twist(KDL::Vector(vx, vy, vz), KDL::Vector(0.0, 0.0, 0.0));
}
