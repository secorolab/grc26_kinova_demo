#include "grc26/filters.hpp"

LowPassFilter::LowPassFilter(double cutoff_freq, double sample_rate)
    : filtered_value_(0.0), initialized_(false)
{
    double rc = 1.0 / (2.0 * M_PI * cutoff_freq);
    double dt = 1.0 / sample_rate;
    alpha_ = dt / (rc + dt);
}

LowPassFilter::LowPassFilter(double alpha)
    : alpha_(alpha), filtered_value_(0.0), initialized_(false)
{
    if (alpha_ <= 0.0 || alpha_ > 1.0) {
        throw std::invalid_argument("Alpha must be in the range (0, 1]");
    }
}

double LowPassFilter::update(double raw_value)
{
    if (!initialized_) {
        filtered_value_ = raw_value;
        initialized_ = true;
    } else {
        filtered_value_ =
            alpha_ * raw_value + (1.0 - alpha_) * filtered_value_;
    }
    return filtered_value_;
}

void LowPassFilter::reset()
{
    filtered_value_ = 0.0;
    initialized_ = false;
}