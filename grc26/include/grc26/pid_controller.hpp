#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "grc26/filters.hpp"

class PID {
  public:
    PID(double p_gain,
        double i_gain,
        double d_gain,
        double error_sum_tol = 0.7,
        double decay_rate    = 0.007,
        double dead_zone_limit = 0.005,
        double lp_filter_alpha = 0.01);

    void set_params(
        double p_gain,
        double i_gain,
        double d_gain,
        double error_sum_tol = 0.7,
        double decay_rate    = 0.007,
        double dead_zone_limit = 0.005,
        double lp_filter_alpha = 0.01
    );

    double control(double error, double dt = 1.0);

  public:
    double err_integ;
    double err_last;
    double kp;
    double ki;
    double kd;
    double err_sum_tol;
    double decay_rate;
    double dead_zone_limit;
    double lp_filter_alpha;
    LowPassFilter d_signal_filter;
};

#endif // PID_CONTROLLER_HPP