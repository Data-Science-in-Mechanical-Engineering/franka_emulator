#include "../include/franka_emulator/vacuum_gripper.h"

FRANKA_EMULATOR::VacuumGripper::VacuumGripper(const std::string&)
{}

FRANKA_EMULATOR::VacuumGripper::VacuumGripper(VacuumGripper&&) noexcept
{}

FRANKA_EMULATOR::VacuumGripper& FRANKA_EMULATOR::VacuumGripper::operator=(VacuumGripper&&) noexcept
{
    return *this;
}

FRANKA_EMULATOR::VacuumGripper::~VacuumGripper() noexcept
{
}

bool FRANKA_EMULATOR::VacuumGripper::vacuum(uint8_t, std::chrono::milliseconds, ProductionSetupProfile) const
{
    return false;
}

bool FRANKA_EMULATOR::VacuumGripper::dropOff(std::chrono::milliseconds) const
{
    return false;
}

bool FRANKA_EMULATOR::VacuumGripper::stop() const
{
    return false;
}

FRANKA_EMULATOR::VacuumGripperState FRANKA_EMULATOR::VacuumGripper::readOnce() const
{
    return VacuumGripperState();
}

FRANKA_EMULATOR::VacuumGripper::ServerVersion FRANKA_EMULATOR::VacuumGripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}