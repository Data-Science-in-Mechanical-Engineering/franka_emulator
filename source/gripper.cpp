#include "../include/franka_emulator/gripper.h"

FRANKA_EMULATOR::Gripper::Gripper(const std::string& franka_address)
{}

FRANKA_EMULATOR::Gripper::Gripper(Gripper&& gripper) noexcept
{}

FRANKA_EMULATOR::Gripper& FRANKA_EMULATOR::Gripper::operator=(Gripper&& gripper) noexcept
{
    return *this;
}

FRANKA_EMULATOR::Gripper::~Gripper() noexcept
{}

bool FRANKA_EMULATOR::Gripper::homing() const
{
    return true;
}

bool FRANKA_EMULATOR::Gripper::grasp(double width,
    double speed,
    double force,
    double epsilon_inner,
    double epsilon_outer) const
{
    return true;
}

bool FRANKA_EMULATOR::Gripper::move(double width, double speed) const
{
    return true;
}

bool FRANKA_EMULATOR::Gripper::stop() const
{
    return true;
}

FRANKA_EMULATOR::GripperState FRANKA_EMULATOR::Gripper::readOnce() const
{
    return FRANKA_EMULATOR::GripperState();
}

FRANKA_EMULATOR::Gripper::ServerVersion FRANKA_EMULATOR::Gripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}