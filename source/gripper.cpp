#include "../include/franka/gripper.h"

FRANKA_EMULATOR_CXX_NAME::Gripper::Gripper(const std::string& franka_address)
{}

FRANKA_EMULATOR_CXX_NAME::Gripper::Gripper(Gripper&& gripper) noexcept
{}

FRANKA_EMULATOR_CXX_NAME::Gripper& FRANKA_EMULATOR_CXX_NAME::Gripper::operator=(Gripper&& gripper) noexcept
{
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Gripper::~Gripper() noexcept
{}

bool FRANKA_EMULATOR_CXX_NAME::Gripper::homing() const
{
    return true;
}

bool FRANKA_EMULATOR_CXX_NAME::Gripper::grasp(double width,
    double speed,
    double force,
    double epsilon_inner,
    double epsilon_outer) const
{
    return true;
}

bool FRANKA_EMULATOR_CXX_NAME::Gripper::move(double width, double speed) const
{
    return true;
}

bool FRANKA_EMULATOR_CXX_NAME::Gripper::stop() const
{
    return true;
}

FRANKA_EMULATOR_CXX_NAME::GripperState FRANKA_EMULATOR_CXX_NAME::Gripper::readOnce() const
{
    return FRANKA_EMULATOR_CXX_NAME::GripperState();
}

FRANKA_EMULATOR_CXX_NAME::Gripper::ServerVersion FRANKA_EMULATOR_CXX_NAME::Gripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}