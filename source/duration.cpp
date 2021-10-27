#include "../include/franka/duration.h"

FRANKA_EMULATOR_CXX_NAME::Duration::Duration() noexcept
{
    duration_ =  std::chrono::duration<uint64_t, std::milli>::zero();
}

FRANKA_EMULATOR_CXX_NAME::Duration::Duration(uint64_t milliseconds) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(milliseconds);
}

FRANKA_EMULATOR_CXX_NAME::Duration::Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept
{
    duration_ = duration;
}

FRANKA_EMULATOR_CXX_NAME::Duration::operator std::chrono::duration<uint64_t, std::milli>() const noexcept
{
    return duration_;
}

double FRANKA_EMULATOR_CXX_NAME::Duration::toSec() const noexcept
{
    return (std::chrono::duration<double>(duration_)).count();
}

uint64_t FRANKA_EMULATOR_CXX_NAME::Duration::toMSec() const noexcept
{
    return duration_.count();
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::Duration::operator+(const Duration& rhs) const noexcept
{
    return duration_ + rhs.duration_;
}

FRANKA_EMULATOR_CXX_NAME::Duration& FRANKA_EMULATOR_CXX_NAME::Duration::operator+=(const Duration& rhs) noexcept
{
    duration_ += rhs.duration_;
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::Duration::operator-(const Duration& rhs) const noexcept
{
    return duration_ - rhs.duration_;
}

FRANKA_EMULATOR_CXX_NAME::Duration& FRANKA_EMULATOR_CXX_NAME::Duration::operator-=(const Duration& rhs) noexcept
{
    duration_ -= rhs.duration_;
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::Duration::operator*(uint64_t rhs) const noexcept
{
    return FRANKA_EMULATOR_CXX_NAME::Duration(rhs * duration_.count());
}

FRANKA_EMULATOR_CXX_NAME::Duration& FRANKA_EMULATOR_CXX_NAME::Duration::operator*=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(rhs * duration_.count());
    return *this;
}

uint64_t FRANKA_EMULATOR_CXX_NAME::Duration::operator/(const Duration& rhs) const noexcept
{
    return duration_.count() / rhs.duration_.count();
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::Duration::operator/(uint64_t rhs) const noexcept
{
    return FRANKA_EMULATOR_CXX_NAME::Duration(duration_.count() / rhs);
}

FRANKA_EMULATOR_CXX_NAME::Duration& FRANKA_EMULATOR_CXX_NAME::Duration::operator/=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() / rhs);
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::Duration::operator%(const Duration& rhs) const noexcept
{
    return FRANKA_EMULATOR_CXX_NAME::Duration(duration_.count() % rhs.duration_.count());
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::Duration::operator%(uint64_t rhs) const noexcept
{
    return FRANKA_EMULATOR_CXX_NAME::Duration(duration_.count() % rhs);
}

FRANKA_EMULATOR_CXX_NAME::Duration& FRANKA_EMULATOR_CXX_NAME::Duration::operator%=(const Duration& rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() % rhs.duration_.count());
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Duration& FRANKA_EMULATOR_CXX_NAME::Duration::operator%=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() % rhs);
    return *this;
}

bool FRANKA_EMULATOR_CXX_NAME::Duration::operator==(const Duration& rhs) const noexcept
{
    return duration_ == rhs.duration_;
}

bool FRANKA_EMULATOR_CXX_NAME::Duration::operator!=(const Duration& rhs) const noexcept
{
    return duration_ != rhs.duration_;
}

bool FRANKA_EMULATOR_CXX_NAME::Duration::operator<(const Duration& rhs) const noexcept
{
    return duration_ < rhs.duration_;
}

bool FRANKA_EMULATOR_CXX_NAME::Duration::operator<=(const Duration& rhs) const noexcept
{
    return duration_ <= rhs.duration_;
}

bool FRANKA_EMULATOR_CXX_NAME::Duration::operator>(const Duration& rhs) const noexcept
{
    return duration_ > rhs.duration_;
}

bool FRANKA_EMULATOR_CXX_NAME::Duration::operator>=(const Duration& rhs) const noexcept
{
    return duration_ >= rhs.duration_;
}

FRANKA_EMULATOR_CXX_NAME::Duration FRANKA_EMULATOR_CXX_NAME::operator*(uint64_t lhs, const Duration& rhs) noexcept
{
    return FRANKA_EMULATOR_CXX_NAME::Duration(lhs * rhs.toMSec());
}