#include "../include/franka/duration.h"

franka::Duration::Duration() noexcept
{
    duration_ =  std::chrono::duration<uint64_t, std::milli>::zero();
}

franka::Duration::Duration(uint64_t milliseconds) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(milliseconds);
}

franka::Duration::Duration(std::chrono::duration<uint64_t, std::milli> duration) noexcept
{
    duration_ = duration;
}

franka::Duration::operator std::chrono::duration<uint64_t, std::milli>() const noexcept
{
    return duration_;
}

double franka::Duration::toSec() const noexcept
{
    return (std::chrono::duration<double>(duration_)).count();
}

uint64_t franka::Duration::toMSec() const noexcept
{
    return duration_.count();
}

franka::Duration franka::Duration::operator+(const Duration& rhs) const noexcept
{
    return duration_ + rhs.duration_;
}

franka::Duration& franka::Duration::operator+=(const Duration& rhs) noexcept
{
    duration_ += rhs.duration_;
    return *this;
}

franka::Duration franka::Duration::operator-(const Duration& rhs) const noexcept
{
    return duration_ - rhs.duration_;
}

franka::Duration& franka::Duration::operator-=(const Duration& rhs) noexcept
{
    duration_ -= rhs.duration_;
    return *this;
}

franka::Duration franka::Duration::operator*(uint64_t rhs) const noexcept
{
    return franka::Duration(rhs * duration_.count());
}

franka::Duration& franka::Duration::operator*=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(rhs * duration_.count());
    return *this;
}

uint64_t franka::Duration::operator/(const Duration& rhs) const noexcept
{
    return duration_.count() / rhs.duration_.count();
}

franka::Duration franka::Duration::operator/(uint64_t rhs) const noexcept
{
    return franka::Duration(duration_.count() / rhs);
}

franka::Duration& franka::Duration::operator/=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() / rhs);
    return *this;
}

franka::Duration franka::Duration::operator%(const Duration& rhs) const noexcept
{
    return franka::Duration(duration_.count() % rhs.duration_.count());
}

franka::Duration franka::Duration::operator%(uint64_t rhs) const noexcept
{
    return franka::Duration(duration_.count() % rhs);
}

franka::Duration& franka::Duration::operator%=(const Duration& rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() % rhs.duration_.count());
    return *this;
}

franka::Duration& franka::Duration::operator%=(uint64_t rhs) noexcept
{
    duration_ = std::chrono::duration<uint64_t, std::milli>(duration_.count() % rhs);
    return *this;
}

bool franka::Duration::operator==(const Duration& rhs) const noexcept
{
    return duration_ == rhs.duration_;
}

bool franka::Duration::operator!=(const Duration& rhs) const noexcept
{
    return duration_ != rhs.duration_;
}

bool franka::Duration::operator<(const Duration& rhs) const noexcept
{
    return duration_ < rhs.duration_;
}

bool franka::Duration::operator<=(const Duration& rhs) const noexcept
{
    return duration_ <= rhs.duration_;
}

bool franka::Duration::operator>(const Duration& rhs) const noexcept
{
    return duration_ > rhs.duration_;
}

bool franka::Duration::operator>=(const Duration& rhs) const noexcept
{
    return duration_ >= rhs.duration_;
}

franka::Duration franka::operator*(uint64_t lhs, const Duration& rhs) noexcept
{
    return franka::Duration(lhs * rhs.toMSec());
}