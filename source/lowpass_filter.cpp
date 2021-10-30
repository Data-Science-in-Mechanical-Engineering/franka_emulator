#include "../include/franka_emulator/lowpass_filter.h"

double FRANKA_EMULATOR_CXX_NAME::lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency)
{
    return 0.0;
}

std::array<double, 16> FRANKA_EMULATOR_CXX_NAME::cartesianLowpassFilter(
    double sample_time,
    std::array<double, 16> y,
    std::array<double, 16> y_last,
    double cutoff_frequency)
{
    return {};
}
