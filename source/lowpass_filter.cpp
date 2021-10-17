#include "../include/franka/lowpass_filter.h"

double franka::lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency)
{
    return 0.0;
}

std::array<double, 16> franka::cartesianLowpassFilter(
    double sample_time,
    std::array<double, 16> y,
    std::array<double, 16> y_last,
    double cutoff_frequency)
{
    return {};
}
