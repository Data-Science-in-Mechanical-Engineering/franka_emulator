#include "../include/franka_emulator/duration.h"

FRANKA_EMULATOR::Duration::Duration() noexcept
{
    duration_ =  std::chrono::duration<uint64_t, std::milli>::zero();
}

FRANKA_EMULATOR::Duration::Duration(uint64_t milliseconds) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(milliseconds);
}

FRANKA_EMULATOR::Duration::Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept
{
    duration_ = duration;
}

FRANKA_EMULATOR::Duration::operator std::chrono::duration<uint64_t, std::milli>() const noexcept
{
    return duration_;
}

double FRANKA_EMULATOR::Duration::toSec() const noexcept
{
    return (std::chrono::duration<double>(duration_)).count();
}

uint64_t FRANKA_EMULATOR::Duration::toMSec() const noexcept
{
    return duration_.count();
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::Duration::operator+(const Duration& rhs) const noexcept
{
    return duration_ + rhs.duration_;
}

FRANKA_EMULATOR::Duration& FRANKA_EMULATOR::Duration::operator+=(const Duration& rhs) noexcept
{
    duration_ += rhs.duration_;
    return *this;
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::Duration::operator-(const Duration& rhs) const noexcept
{
    return duration_ - rhs.duration_;
}

FRANKA_EMULATOR::Duration& FRANKA_EMULATOR::Duration::operator-=(const Duration& rhs) noexcept
{
    duration_ -= rhs.duration_;
    return *this;
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::Duration::operator*(uint64_t rhs) const noexcept
{
    return FRANKA_EMULATOR::Duration(rhs * duration_.count());
}

FRANKA_EMULATOR::Duration& FRANKA_EMULATOR::Duration::operator*=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(rhs * duration_.count());
    return *this;
}

uint64_t FRANKA_EMULATOR::Duration::operator/(const Duration& rhs) const noexcept
{
    return duration_.count() / rhs.duration_.count();
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::Duration::operator/(uint64_t rhs) const noexcept
{
    return FRANKA_EMULATOR::Duration(duration_.count() / rhs);
}

FRANKA_EMULATOR::Duration& FRANKA_EMULATOR::Duration::operator/=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() / rhs);
    return *this;
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::Duration::operator%(const Duration& rhs) const noexcept
{
    return FRANKA_EMULATOR::Duration(duration_.count() % rhs.duration_.count());
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::Duration::operator%(uint64_t rhs) const noexcept
{
    return FRANKA_EMULATOR::Duration(duration_.count() % rhs);
}

FRANKA_EMULATOR::Duration& FRANKA_EMULATOR::Duration::operator%=(const Duration& rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() % rhs.duration_.count());
    return *this;
}

FRANKA_EMULATOR::Duration& FRANKA_EMULATOR::Duration::operator%=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() % rhs);
    return *this;
}

bool FRANKA_EMULATOR::Duration::operator==(const Duration& rhs) const noexcept
{
    return duration_ == rhs.duration_;
}

bool FRANKA_EMULATOR::Duration::operator!=(const Duration& rhs) const noexcept
{
    return duration_ != rhs.duration_;
}

bool FRANKA_EMULATOR::Duration::operator<(const Duration& rhs) const noexcept
{
    return duration_ < rhs.duration_;
}

bool FRANKA_EMULATOR::Duration::operator<=(const Duration& rhs) const noexcept
{
    return duration_ <= rhs.duration_;
}

bool FRANKA_EMULATOR::Duration::operator>(const Duration& rhs) const noexcept
{
    return duration_ > rhs.duration_;
}

bool FRANKA_EMULATOR::Duration::operator>=(const Duration& rhs) const noexcept
{
    return duration_ >= rhs.duration_;
}

FRANKA_EMULATOR::Duration FRANKA_EMULATOR::operator*(uint64_t lhs, const Duration& rhs) noexcept
{
    return FRANKA_EMULATOR::Duration(lhs * rhs.toMSec());
}