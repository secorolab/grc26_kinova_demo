#include "grc26/pid_controller.hpp"
#include <cmath>

PID::PID(double p_gain,
         double i_gain,
         double d_gain,
         double err_sum_tol,
         double decay_rate,
         double dead_zone_limit,
         double lp_filter_alpha,
         double saturation_limit)
    : err_integ(0.0),
      err_last(0.0),
      kp(p_gain),
      ki(i_gain),
      kd(d_gain),
      saturation_limit(saturation_limit),
      err_sum_tol(err_sum_tol),
      decay_rate(decay_rate),
      dead_zone_limit(dead_zone_limit),
      lp_filter_alpha(lp_filter_alpha),
      d_signal_filter(lp_filter_alpha)
{
}

void PID::set_params(double p_gain,
                    double i_gain,
                    double d_gain,
                    double err_sum_tol,
                    double decay_rate,
                    double dead_zone_limit,
                    double lp_filter_alpha,
                    double saturation_limit)
{
    err_integ             = 0.0;
    err_last              = 0.0;
    kp                    = p_gain;
    ki                    = i_gain;
    kd                    = d_gain;
    this->err_sum_tol     = err_sum_tol;
    this->decay_rate      = decay_rate;
    this->dead_zone_limit = dead_zone_limit;
    this->lp_filter_alpha = lp_filter_alpha;
    this->saturation_limit = saturation_limit;
    d_signal_filter = LowPassFilter(lp_filter_alpha);
}

double PID::control(double error, double dt)
{
    if (dt <= 0.0) {dt = 1e-3;}  // safety fallback

    // Deadzone applied only to P and D
    double effective_error = error;
    if (std::abs(error) < dead_zone_limit)
    { effective_error = 0.0; }
    
    if (first_update)
    {
        err_last = error;
        first_update = false;
    } // handle first update case to prevent large derivative kick
    
    if (stiffness_control_mode)
    {
        last_error = error;
        last_p_term = kp * effective_error;
        last_i_term = 0.0;
        last_d_term = 0.0;
        last_output = last_p_term;
        return last_output;
    }

    // computing derivative term
    double err_diff = (error - err_last) / dt;
    err_last = error;
    // filtering the derivative term
    double filtered_d = d_signal_filter.update(err_diff);

    err_integ += error * dt;
    // if ((error > 0 && err_integ < 0) || (error < 0 && err_integ > 0))
    // {
    //     err_integ = decay_rate * err_integ + (1.0 - decay_rate) * error;
    // }
    // Clamp integral to prevent windup
    if (err_integ > err_sum_tol) {
        err_integ = err_sum_tol;
    } else if (err_integ < -err_sum_tol) {
        err_integ = -err_sum_tol;
    }

    const double p_term = kp * effective_error;
    const double i_term = ki * err_integ;
    const double d_term = kd * filtered_d;
    double out = p_term + i_term + d_term;

    double unsat = out;

    if (out > saturation_limit) out = saturation_limit;
    if (out < -saturation_limit) out = -saturation_limit;

    // anti-windup
    if (unsat != out)
        err_integ -= error * dt;

    last_error = error;
    last_p_term = p_term;
    last_i_term = i_term;
    last_d_term = d_term;
    last_output = out;

    return out;
}