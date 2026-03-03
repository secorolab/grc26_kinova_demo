#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <cmath>
#include <stdexcept>

class LowPassFilter {
private:
    double alpha_;            // Smoothing factor (0 < alpha <= 1)
    double filtered_value_;   // Current filtered value
    bool initialized_;        // Indicates if the filter has been initialized

public:
    // Constructor
    // cutoff_freq: Cutoff frequency in Hz
    // sample_rate: Sampling rate in Hz
    LowPassFilter(double cutoff_freq, double sample_rate);

    LowPassFilter(double alpha);

    // Update filter with new raw value
    double update(double raw_value);

    // Reset filter state
    void reset();
};

#endif // FILTERS_HPP