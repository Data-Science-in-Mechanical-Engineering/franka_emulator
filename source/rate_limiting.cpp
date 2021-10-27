#include "../include/franka/rate_limiting.h"
#include <stddef.h>

std::array<double, 7> FRANKA_EMULATOR_CXX_NAME::limitRate(
    const std::array<double, 7>& max_derivatives,
    const std::array<double, 7>& commanded_values,
    const std::array<double, 7>& last_commanded_values)
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}

double FRANKA_EMULATOR_CXX_NAME::limitRate(
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    double commanded_velocity,
    double last_commanded_velocity,
    double last_commanded_acceleration)
{
    return 0.0;
}

double FRANKA_EMULATOR_CXX_NAME::limitRate(
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    double commanded_position,
    double last_commanded_position,
    double last_commanded_velocity,
    double last_commanded_acceleration)
{
    return 0.0;
}

std::array<double, 7> FRANKA_EMULATOR_CXX_NAME::limitRate(
    const std::array<double, 7>& max_velocity,
    const std::array<double, 7>& max_acceleration,
    const std::array<double, 7>& max_jerk,
    const std::array<double, 7>& commanded_velocities,
    const std::array<double, 7>& last_commanded_velocities,
    const std::array<double, 7>& last_commanded_accelerations)
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}

std::array<double, 7> FRANKA_EMULATOR_CXX_NAME::limitRate(
    const std::array<double, 7>& max_velocity,
    const std::array<double, 7>& max_acceleration,
    const std::array<double, 7>& max_jerk,
    const std::array<double, 7>& commanded_positions,
    const std::array<double, 7>& last_commanded_positions,
    const std::array<double, 7>& last_commanded_velocities,
    const std::array<double, 7>& last_commanded_accelerations)
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}

std::array<double, 6> FRANKA_EMULATOR_CXX_NAME::limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 6>& O_dP_EE_c,
    const std::array<double, 6>& last_O_dP_EE_c,
    const std::array<double, 6>& last_O_ddP_EE_c)
{
    std::array<double, 6> result;
    for (size_t i = 0; i < 6; i++) result[i] = 0.0;
    return result;
}

std::array<double, 16> FRANKA_EMULATOR_CXX_NAME::limitRate(
    double max_translational_velocity,
    double max_translational_acceleration,
    double max_translational_jerk,
    double max_rotational_velocity,
    double max_rotational_acceleration,
    double max_rotational_jerk,
    const std::array<double, 16>& O_T_EE_c,
    const std::array<double, 16>& last_O_T_EE_c,
    const std::array<double, 6>& last_O_dP_EE_c,
    const std::array<double, 6>& last_O_ddP_EE_c)
{
    std::array<double, 16> result;
    for (size_t i = 0; i < 16; i++) result[i] = 0.0;
    return result;
}