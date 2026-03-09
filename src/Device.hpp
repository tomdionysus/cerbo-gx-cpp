#pragma once

namespace cerbo
{

class CerboGX;

class Device
{
public:
    virtual ~Device() = default;

    int instance() const noexcept
    {
        return instance_;
    }

    const CerboGX* parent() const noexcept
    {
        return parent_;
    }

    virtual const char* service_name() const noexcept = 0;

protected:
    Device() = default;

    Device(const CerboGX* parent, int instance) noexcept
        : parent_(parent), instance_(instance)
    {
    }

    const CerboGX* parent_ = nullptr;
    int instance_ = -1;
};

} // namespace cerbo