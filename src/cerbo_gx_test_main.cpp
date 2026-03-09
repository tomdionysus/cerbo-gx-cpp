#include <iomanip>
#include <iostream>
#include <optional>
#include <string>

#include "CerboGX.hpp"

struct Args
{
    std::string ip;
    int mqtt_port = 1883;
    int legacy_port = 502;
};

static void usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " --ip <cerbo-ip> [--mqtt-port <port>] [--port <legacy-port>]\n"
        << "  --mqtt-port  MQTT port used for transport (default: 1883)\n"
        << "  --port       Legacy compatibility field only (default: 502)\n";
}

static std::optional<Args> parse_args(int argc, char** argv)
{
    Args args;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];

        if (a == "--ip")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for --ip\n";
                return std::nullopt;
            }
            args.ip = argv[++i];
        }
        else if (a == "--mqtt-port")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for --mqtt-port\n";
                return std::nullopt;
            }
            try
            {
                args.mqtt_port = std::stoi(argv[++i]);
            }
            catch (...)
            {
                std::cerr << "Invalid MQTT port\n";
                return std::nullopt;
            }
        }
        else if (a == "--port")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for --port\n";
                return std::nullopt;
            }
            try
            {
                args.legacy_port = std::stoi(argv[++i]);
            }
            catch (...)
            {
                std::cerr << "Invalid port\n";
                return std::nullopt;
            }
        }
        else
        {
            std::cerr << "Unknown argument: " << a << "\n";
            return std::nullopt;
        }
    }

    if (args.ip.empty())
    {
        std::cerr << "--ip is required\n";
        return std::nullopt;
    }

    if (args.mqtt_port <= 0 || args.mqtt_port > 65535)
    {
        std::cerr << "MQTT port must be 1..65535\n";
        return std::nullopt;
    }

    if (args.legacy_port <= 0 || args.legacy_port > 65535)
    {
        std::cerr << "Port must be 1..65535\n";
        return std::nullopt;
    }

    return args;
}

template <typename T>
static void print_opt_value(const std::string& label,
                            const std::optional<T>& value,
                            const std::string& unit = "")
{
    std::cout << "  " << std::left << std::setw(28) << label << ": ";
    if (value)
    {
        std::cout << *value;
        if (!unit.empty())
            std::cout << " " << unit;
    }
    else
    {
        std::cout << "n/a";
    }
    std::cout << "\n";
}

static void print_opt_double(const std::string& label,
                             const std::optional<double>& value,
                             const std::string& unit = "",
                             int precision = 1)
{
    std::cout << "  " << std::left << std::setw(28) << label << ": ";
    if (value)
    {
        std::cout << std::fixed << std::setprecision(precision) << *value;
        if (!unit.empty())
            std::cout << " " << unit;
    }
    else
    {
        std::cout << "n/a";
    }
    std::cout << "\n";
}

static void print_opt_string(const std::string& label,
                             const std::optional<std::string>& value)
{
    std::cout << "  " << std::left << std::setw(28) << label << ": ";
    if (value)
        std::cout << *value;
    else
        std::cout << "n/a";
    std::cout << "\n";
}

static void print_alarm(const std::string& label, const std::optional<int>& value)
{
    std::cout << "  " << std::left << std::setw(28) << label << ": ";
    if (value)
        std::cout << *value << " (" << cerbo::SmartShuntDevice::alarm_state_to_string(*value) << ")";
    else
        std::cout << "n/a";
    std::cout << "\n";
}

int main(int argc, char** argv)
{
    const auto args = parse_args(argc, argv);
    if (!args)
    {
        usage(argv[0]);
        return 1;
    }

    cerbo::CerboGX cerbo(args->ip, args->legacy_port, args->mqtt_port);

    if (!cerbo.connect())
    {
        std::cerr << "Failed to connect to " << cerbo.ip() << ":" << cerbo.mqtt_port()
                  << " : " << cerbo.last_error() << "\n";
        return 1;
    }

    std::cout << "Connected to Cerbo GX MQTT at "
              << cerbo.ip() << ":" << cerbo.mqtt_port() << "\n\n";

    std::cout << "System\n";
    std::cout << "======\n";

    const auto sys = cerbo.info();
    if (!sys)
    {
        std::cout << "  Unable to read system info\n\n";
    }
    else
    {
        std::cout << "  " << std::left << std::setw(28) << "Serial"
                  << ": " << (sys->serial.empty() ? "n/a" : sys->serial) << "\n";
        print_opt_double("Battery voltage", sys->battery_voltage_v, "V", 1);
        print_opt_double("Battery current", sys->battery_current_a, "A", 1);
        print_opt_value("Battery power", sys->battery_power_w, "W");
        print_opt_double("Battery SoC", sys->battery_soc_pct, "%", 1);

        std::cout << "  " << std::left << std::setw(28) << "Battery state"
                  << ": ";
        if (sys->battery_state)
            std::cout << cerbo::SmartShuntDevice::state_to_string(*sys->battery_state);
        else
            std::cout << "n/a";
        std::cout << "\n\n";
    }

    cerbo.scan();

    std::cout << "VE.Bus / Multi-class devices\n";
    std::cout << "===========================\n";

    if (cerbo.multiplus_devices().empty())
    {
        std::cout << "  None detected.\n\n";
    }
    else
    {
        for (const auto& dev : cerbo.multiplus_devices())
        {
            const auto info = dev.read();
            if (!info)
                continue;

            std::cout << "Unit ID " << info->instance << "\n";
            std::cout << "  " << std::left << std::setw(28) << "Class"
                      << ": VE.Bus / Multi / MultiPlus-type service\n";
            print_opt_double("DC voltage", info->dc_voltage_v, "V", 2);
            print_opt_double("DC current", info->dc_current_a, "A", 1);
            print_opt_double("SoC", info->soc_pct, "%", 1);

            std::cout << "  " << std::left << std::setw(28) << "Mode"
                      << ": ";
            if (info->mode)
                std::cout << cerbo::MultiPlusDevice::mode_to_string(*info->mode);
            else
                std::cout << "n/a";
            std::cout << "\n";

            std::cout << "  " << std::left << std::setw(28) << "State"
                      << ": ";
            if (info->state)
                std::cout << cerbo::MultiPlusDevice::state_to_string(*info->state);
            else
                std::cout << "n/a";
            std::cout << "\n";

            print_opt_value("Phase count", info->phase_count);
            print_opt_double("AC out L1 power", info->out_l1_power_w, "W", 0);
            print_opt_double("AC out L2 power", info->out_l2_power_w, "W", 0);
            print_opt_double("AC out L3 power", info->out_l3_power_w, "W", 0);
            std::cout << "\n";
        }
    }

    std::cout << "Battery monitor services\n";
    std::cout << "========================\n";

    if (cerbo.smartshunt_devices().empty())
    {
        std::cout << "  None detected.\n\n";
    }
    else
    {
        for (const auto& dev : cerbo.smartshunt_devices())
        {
            const auto info = dev.read();
            if (!info)
                continue;

            std::cout << "Unit ID " << info->instance << "\n";
            std::cout << "  " << std::left << std::setw(28) << "Class"
                      << ": Battery service (SmartShunt/BMV/managed battery)\n";

            print_opt_value("Connected", info->connected);
            print_opt_string("Serial", info->serial);
            print_opt_string("Product name", info->product_name);
            print_opt_value("Product ID", info->product_id);
            print_opt_value("Firmware version", info->firmware_version);
            print_opt_value("Hardware version", info->hardware_version);
            print_opt_string("Custom name", info->custom_name);

            print_opt_double("Voltage", info->voltage_v, "V", 2);
            print_opt_double("Current", info->current_a, "A", 2);
            print_opt_double("Power", info->power_w, "W", 1);
            print_opt_double("Consumed Ah", info->consumed_amphours, "Ah", 2);
            print_opt_double("SoC", info->soc_pct, "%", 2);
            print_opt_double("Time to go", info->time_to_go_s, "s", 0);

            print_opt_value("Has midpoint", info->settings.has_mid_voltage);
            print_opt_value("Has starter voltage", info->settings.has_starter_voltage);
            print_opt_value("Has temperature", info->settings.has_temperature);

            print_opt_double("Midpoint voltage", info->midpoint_voltage_v, "V", 2);
            print_opt_double("Midpoint deviation", info->midpoint_deviation_pct, "%", 2);
            print_opt_double("Temperature", info->temperature_c, "C", 1);
            print_opt_double("Aux voltage", info->aux_voltage_v, "V", 2);

            print_opt_string("Mgmt connection", info->mgmt_connection);
            print_opt_string("Mgmt process name", info->mgmt_process_name);
            print_opt_string("Mgmt process version", info->mgmt_process_version);

            std::cout << "  Device[0]\n";
            print_opt_string("    Product name", info->device0.product_name);
            print_opt_string("    Service name", info->device0.service_name);
            print_opt_value("    Product ID", info->device0.product_id);
            print_opt_value("    Firmware version", info->device0.firmware_version);
            print_opt_value("    Device instance", info->device0.device_instance);

            std::cout << "  Alarms\n";
            print_alarm("    Alarm", info->alarms.alarm);
            print_alarm("    High starter voltage", info->alarms.high_starter_voltage);
            print_alarm("    High temperature", info->alarms.high_temperature);
            print_alarm("    High voltage", info->alarms.high_voltage);
            print_alarm("    Low SoC", info->alarms.low_soc);
            print_alarm("    Low starter voltage", info->alarms.low_starter_voltage);
            print_alarm("    Low temperature", info->alarms.low_temperature);
            print_alarm("    Low voltage", info->alarms.low_voltage);
            print_alarm("    Mid voltage", info->alarms.mid_voltage);

            std::cout << "  History\n";
            print_opt_value("    Automatic syncs", info->history.automatic_syncs);
            print_opt_double("    Average discharge", info->history.average_discharge, "Ah", 2);
            print_opt_value("    Charge cycles", info->history.charge_cycles);
            print_opt_double("    Charged energy", info->history.charged_energy, "kWh", 2);
            print_opt_double("    Deepest discharge", info->history.deepest_discharge, "Ah", 2);
            print_opt_double("    Discharged energy", info->history.discharged_energy, "kWh", 2);
            print_opt_value("    Full discharges", info->history.full_discharges);
            print_opt_value("    High starter V alarms", info->history.high_starter_voltage_alarms);
            print_opt_value("    High voltage alarms", info->history.high_voltage_alarms);
            print_opt_double("    Last discharge", info->history.last_discharge, "Ah", 2);
            print_opt_value("    Low starter V alarms", info->history.low_starter_voltage_alarms);
            print_opt_value("    Low voltage alarms", info->history.low_voltage_alarms);
            print_opt_double("    Max starter voltage", info->history.maximum_starter_voltage, "V", 2);
            print_opt_double("    Max voltage", info->history.maximum_voltage, "V", 2);
            print_opt_double("    Min starter voltage", info->history.minimum_starter_voltage, "V", 2);
            print_opt_double("    Min voltage", info->history.minimum_voltage, "V", 2);
            print_opt_value("    Time since full charge", info->history.time_since_last_full_charge, "s");
            print_opt_double("    Total Ah drawn", info->history.total_ah_drawn, "Ah", 2);

            std::cout << "  VE.Direct parser stats\n";
            print_opt_value("    Hex checksum errors", info->vedirect.hex_checksum_errors);
            print_opt_value("    Hex invalid char errors", info->vedirect.hex_invalid_character_errors);
            print_opt_value("    Hex unfinished errors", info->vedirect.hex_unfinished_errors);
            print_opt_value("    Text checksum errors", info->vedirect.text_checksum_errors);
            print_opt_value("    Text parse errors", info->vedirect.text_parse_error);
            print_opt_value("    Text unfinished errors", info->vedirect.text_unfinished_errors);

            std::cout << "\n";
        }
    }

    cerbo.disconnect();
    return 0;
}