#include "grc26/pid_controller.hpp"
#include <cmath>

PID::PID(double p_gain,
         double i_gain,
         double d_gain,
         double error_sum_tol,
         double decay_rate,
         double dead_zone_limit,
         double lp_filter_alpha)
    : err_integ(0.0),
      err_last(0.0),
      kp(p_gain),
      ki(i_gain),
      kd(d_gain),
      err_sum_tol(error_sum_tol),
      decay_rate(decay_rate),
      dead_zone_limit(dead_zone_limit),
      lp_filter_alpha(lp_filter_alpha),
      d_signal_filter(lp_filter_alpha)
{
}

void PID::set_params(double p_gain,
                    double i_gain,
                    double d_gain,
                    double error_sum_tol,
                    double decay_rate,
                    double dead_zone_limit,
                    double lp_filter_alpha)
{
    err_integ             = 0.0;
    err_last              = 0.0;
    kp                    = p_gain;
    ki                    = i_gain;
    kd                    = d_gain;
    this->err_sum_tol     = error_sum_tol;
    this->decay_rate      = decay_rate;
    this->dead_zone_limit = dead_zone_limit;
    this->lp_filter_alpha = lp_filter_alpha;
    d_signal_filter = LowPassFilter(lp_filter_alpha);
}

double PID::control(double error, double dt)
{
    if (dt <= 0.0) {dt = 1.0;}  // safety fallback

    if (std::abs(error) < dead_zone_limit)
    { error = 0.0; }
    
    if (err_last == 0.0)
    { err_last = error; } // avoid large derivative kick at startup
    
    // computing derivative term
    double err_diff = (error - err_last) / dt;
    err_last = error;
    // filtering the derivative term
    double filtered_d = d_signal_filter.update(err_diff);

    // integral term with anti-windup via integral clamping and decay
    err_integ += error * dt;
    if ((error > 0 && err_integ < 0) || (error < 0 && err_integ > 0))
    {
        err_integ = decay_rate * err_integ + (1.0 - decay_rate) * error;
    }
    // Clamp integral to prevent windup
    if (err_integ > err_sum_tol) {
        err_integ = err_sum_tol;
    } else if (err_integ < -err_sum_tol) {
        err_integ = -err_sum_tol;
    }

    return kp * error + ki * err_integ + kd * filtered_d;
}