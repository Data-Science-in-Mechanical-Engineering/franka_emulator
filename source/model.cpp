#include "../include/franka/model.h"

franka::Frame franka::operator++(Frame& frame, int /* dummy */) noexcept
{
    return static_cast<Frame>(static_cast<int>(frame) + 1);
}

franka::Model::Model(franka::Network&)
{}

franka::Model::Model(Model&&) noexcept
{}

franka::Model& franka::Model::operator=(Model&&) noexcept
{
    return *this;
}

franka::Model::~Model() noexcept
{}

std::array<double, 16> franka::Model::pose(Frame frame, const franka::RobotState& robot_state) const
{
    std::array<double, 16> result;
    for (size_t i = 0; i < 16; i++) result[i] = 0.0;
    return result;
}

std::array<double, 16> franka::Model::pose(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
        const std::array<double, 16>& EE_T_K) const
{
    std::array<double, 16> result;
    for (size_t i = 0; i < 16; i++) result[i] = 0.0;
    return result;
}

std::array<double, 42> franka::Model::bodyJacobian(Frame frame, const franka::RobotState& robot_state) const
{
    std::array<double, 42> result;
    for (size_t i = 0; i < 42; i++) result[i] = 0.0;
    return result;
}

std::array<double, 42> franka::Model::bodyJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    std::array<double, 42> result;
    for (size_t i = 0; i < 42; i++) result[i] = 0.0;
    return result;
}

std::array<double, 42> franka::Model::zeroJacobian(Frame frame, const franka::RobotState& robot_state) const
{
    std::array<double, 42> result;
    for (size_t i = 0; i < 42; i++) result[i] = 0.0;
    return result;
}

std::array<double, 42> franka::Model::zeroJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    std::array<double, 42> result;
    for (size_t i = 0; i < 42; i++) result[i] = 0.0;
    return result;
}

std::array<double, 49> franka::Model::mass(const franka::RobotState& robot_state) const noexcept
{
    std::array<double, 49> result;
    for (size_t i = 0; i < 49; i++) result[i] = 0.0;
    return result;
}

std::array<double, 49> franka::Model::mass(
    const std::array<double, 7>& q,
    const std::array<double, 9>& I_total,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) const noexcept
{
    std::array<double, 49> result;
    for (size_t i = 0; i < 49; i++) result[i] = 0.0;
    return result;
}

std::array<double, 7> franka::Model::coriolis(const franka::RobotState& robot_state) const noexcept
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}

std::array<double, 7> franka::Model::coriolis(
    const std::array<double, 7>& q,
    const std::array<double, 7>& dq,
    const std::array<double, 9>& I_total,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) const noexcept
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}

std::array<double, 7> franka::Model::gravity(
    const std::array<double, 7>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,
    const std::array<double, 3>& gravity_earth) const noexcept
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}

std::array<double, 7> franka::Model::gravity(
    const franka::RobotState& robot_state,
    const std::array<double, 3>& gravity_earth) const noexcept
{
    std::array<double, 7> result;
    for (size_t i = 0; i < 7; i++) result[i] = 0.0;
    return result;
}