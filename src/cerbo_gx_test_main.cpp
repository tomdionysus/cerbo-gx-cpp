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
        << "  --port       Legacy API compatibility field only (default: 502)\n";
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
    std::cout << "  " << std::left << std::setw(24) << label << ": ";
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
    std::cout << "  " << std::left << std::setw(24) << label << ": ";
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
        std::cout << "  " << std::left << std::setw(24) << "Serial"
                  << ": " << (sys->serial.empty() ? "n/a" : sys->serial) << "\n";
        print_opt_double("Battery voltage", sys->battery_voltage_v, "V", 1);
        print_opt_double("Battery current", sys->battery_current_a, "A", 1);
        print_opt_value("Battery power", sys->battery_power_w, "W");
        print_opt_double("Battery SoC", sys->battery_soc_pct, "%", 0);

        std::cout << "  " << std::left << std::setw(24) << "Battery state"
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
            std::cout << "  " << std::left << std::setw(24) << "Class"
                      << ": VE.Bus / Multi / MultiPlus-type service\n";
            print_opt_double("DC voltage", info->dc_voltage_v, "V", 2);
            print_opt_double("DC current", info->dc_current_a, "A", 1);
            print_opt_double("SoC", info->soc_pct, "%", 1);

            std::cout << "  " << std::left << std::setw(24) << "Mode"
                      << ": ";
            if (info->mode)
                std::cout << cerbo::MultiPlusDevice::mode_to_string(*info->mode);
            else
                std::cout << "n/a";
            std::cout << "\n";

            std::cout << "  " << std::left << std::setw(24) << "State"
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
            std::cout << "  " << std::left << std::setw(24) << "Class"
                      << ": Battery service (SmartShunt/BMV/managed battery)\n";
            print_opt_double("Voltage", info->voltage_v, "V", 2);
            print_opt_double("Current", info->current_a, "A", 1);
            print_opt_double("SoC", info->soc_pct, "%", 1);
            print_opt_value("Power", info->power_w, "W");
            std::cout << "\n";
        }
    }

    cerbo.disconnect();
    return 0;
}