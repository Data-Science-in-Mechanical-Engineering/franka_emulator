#include "../include/franka/vacuum_gripper.h"

franka::VacuumGripper::VacuumGripper(const std::string&)
{}

franka::VacuumGripper::VacuumGripper(VacuumGripper&&) noexcept
{}

franka::VacuumGripper& franka::VacuumGripper::operator=(VacuumGripper&&) noexcept
{
    return *this;
}

franka::VacuumGripper::~VacuumGripper() noexcept
{
}

bool franka::VacuumGripper::vacuum(uint8_t, std::chrono::milliseconds, ProductionSetupProfile) const
{
    return false;
}

bool franka::VacuumGripper::dropOff(std::chrono::milliseconds) const
{
    return false;
}

bool franka::VacuumGripper::stop() const
{
    return false;
}

franka::VacuumGripperState franka::VacuumGripper::readOnce() const
{
    return VacuumGripperState();
}

franka::VacuumGripper::ServerVersion franka::VacuumGripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}