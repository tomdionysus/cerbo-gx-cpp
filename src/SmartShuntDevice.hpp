#pragma once

#include "Device.hpp"

#include <cstdint>
#include <optional>
#include <sstream>
#include <string>

namespace cerbo
{

class SmartShuntDevice : public Device
{
public:
    struct Info
    {
        int instance = -1;
        std::optional<double> voltage_v;
        std::optional<double> current_a;
        std::optional<double> soc_pct;
        std::optional<int32_t> power_w;
    };

    SmartShuntDevice() = default;

    SmartShuntDevice(const CerboGX* parent, int instance) noexcept
        : Device(parent, instance)
    {
    }

    const char* service_name() const noexcept override
    {
        return "battery";
    }

    std::optional<Info> read() const;

    static std::string state_to_string(int state)
    {
        switch (state)
        {
            case 0: return "Idle";
            case 1: return "Charging";
            case 2: return "Discharging";
            default:
            {
                std::ostringstream os;
                os << "Unknown(" << state << ")";
                return os.str();
            }
        }
    }
};

} // namespace cerbo