#include "../include/franka/gripper.h"

franka::Gripper::Gripper(const std::string& franka_address)
{}

franka::Gripper::Gripper(Gripper&& gripper) noexcept
{}

franka::Gripper& franka::Gripper::operator=(Gripper&& gripper) noexcept
{
    return *this;
}

franka::Gripper::~Gripper() noexcept
{}

bool franka::Gripper::homing() const
{
    return true;
}

bool franka::Gripper::grasp(double width,
    double speed,
    double force,
    double epsilon_inner,
    double epsilon_outer) const
{
    return true;
}

bool franka::Gripper::move(double width, double speed) const
{
    return true;
}

bool franka::Gripper::stop() const
{
    return true;
}

franka::GripperState franka::Gripper::readOnce() const
{
    return franka::GripperState();
}

franka::Gripper::ServerVersion franka::Gripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}