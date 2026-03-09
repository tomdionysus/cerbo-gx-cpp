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
    struct Alarms
    {
        std::optional<int> alarm;
        std::optional<int> high_starter_voltage;
        std::optional<int> high_temperature;
        std::optional<int> high_voltage;
        std::optional<int> low_soc;
        std::optional<int> low_starter_voltage;
        std::optional<int> low_temperature;
        std::optional<int> low_voltage;
        std::optional<int> mid_voltage;
    };

    struct Settings
    {
        std::optional<int> has_mid_voltage;
        std::optional<int> has_starter_voltage;
        std::optional<int> has_temperature;
        std::optional<int> relay_mode;
    };

    struct DeviceDetails
    {
        std::optional<std::string> custom_name;
        std::optional<int> device_instance;
        std::optional<int> firmware_version;
        std::optional<int> product_id;
        std::optional<std::string> product_name;
        std::optional<std::string> service_name;
        std::optional<std::string> vreg_link;
    };

    struct History
    {
        std::optional<int> automatic_syncs;
        std::optional<double> average_discharge;
        std::optional<int> charge_cycles;
        std::optional<double> charged_energy;
        std::optional<double> deepest_discharge;
        std::optional<double> discharged_energy;
        std::optional<int> full_discharges;
        std::optional<int> high_starter_voltage_alarms;
        std::optional<int> high_voltage_alarms;
        std::optional<double> last_discharge;
        std::optional<int> low_starter_voltage_alarms;
        std::optional<int> low_voltage_alarms;
        std::optional<double> maximum_starter_voltage;
        std::optional<double> maximum_voltage;
        std::optional<double> minimum_starter_voltage;
        std::optional<double> minimum_voltage;
        std::optional<int> time_since_last_full_charge;
        std::optional<double> total_ah_drawn;
    };

    struct VEDirectErrors
    {
        std::optional<int> hex_checksum_errors;
        std::optional<int> hex_invalid_character_errors;
        std::optional<int> hex_unfinished_errors;
        std::optional<int> text_checksum_errors;
        std::optional<int> text_parse_error;
        std::optional<int> text_unfinished_errors;
    };

    struct Info
    {
        int instance = -1;

        std::optional<int> connected;
        std::optional<double> consumed_amphours;
        std::optional<std::string> custom_name;

        std::optional<double> voltage_v;
        std::optional<double> current_a;
        std::optional<double> power_w;
        std::optional<double> midpoint_voltage_v;
        std::optional<double> midpoint_deviation_pct;
        std::optional<double> temperature_c;
        std::optional<double> aux_voltage_v;

        std::optional<int> device_instance;
        std::optional<int> firmware_version;
        std::optional<int> group_id;
        std::optional<int> hardware_version;

        std::optional<std::string> mgmt_connection;
        std::optional<std::string> mgmt_process_name;
        std::optional<std::string> mgmt_process_version;

        std::optional<int> product_id;
        std::optional<std::string> product_name;
        std::optional<int> relay_state;
        std::optional<std::string> serial;
        std::optional<double> soc_pct;
        std::optional<double> time_to_go_s;

        Alarms alarms;
        Settings settings;
        DeviceDetails device0;
        History history;
        VEDirectErrors vedirect;
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

    static const char* alarm_state_to_string(int value) noexcept
    {
        return value == 0 ? "OK" : "Alarm";
    }
};

} // namespace cerbo