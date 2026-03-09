#pragma once

#include "Device.hpp"

#include <optional>
#include <sstream>
#include <string>

namespace cerbo
{

class MultiPlusDevice : public Device
{
public:
    struct Info
    {
        int instance = -1;
        std::optional<double> dc_voltage_v;
        std::optional<double> dc_current_a;
        std::optional<double> soc_pct;
        std::optional<int> state;
        std::optional<int> mode;
        std::optional<int> phase_count;
        std::optional<double> out_l1_power_w;
        std::optional<double> out_l2_power_w;
        std::optional<double> out_l3_power_w;
    };

    MultiPlusDevice() = default;

    MultiPlusDevice(const CerboGX* parent, int instance) noexcept
        : Device(parent, instance)
    {
    }

    const char* service_name() const noexcept override
    {
        return "vebus";
    }

    std::optional<Info> read() const;

    static std::string mode_to_string(int mode)
    {
        switch (mode)
        {
            case 1: return "Charger Only";
            case 2: return "Inverter Only";
            case 3: return "On";
            case 4: return "Off";
            default:
            {
                std::ostringstream os;
                os << "Unknown(" << mode << ")";
                return os.str();
            }
        }
    }

    static std::string state_to_string(int state)
    {
        switch (state)
        {
            case 0: return "Off";
            case 1: return "Low Power";
            case 2: return "Fault";
            case 3: return "Bulk";
            case 4: return "Absorption";
            case 5: return "Float";
            case 6: return "Storage";
            case 7: return "Equalize";
            case 8: return "Passthru";
            case 9: return "Inverting";
            case 10: return "Power Assist";
            case 11: return "Power Supply";
            case 244: return "Sustain";
            case 252: return "External Control";
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