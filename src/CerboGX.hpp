#pragma once

#include <mosquitto.h>

#include "MultiPlusDevice.hpp"
#include "SmartShuntDevice.hpp"

#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace cerbo
{

class CerboGX
{
public:
    struct SystemInfo
    {
        std::string serial;
        std::optional<double> battery_voltage_v;
        std::optional<double> battery_current_a;
        std::optional<int32_t> battery_power_w;
        std::optional<double> battery_soc_pct;
        std::optional<int> battery_state;
    };

    explicit CerboGX(std::string ip, int port = 502, int mqtt_port = 1883)
        : ip_(std::move(ip)), port_(port), mqtt_port_(mqtt_port)
    {
    }

    ~CerboGX()
    {
        disconnect();
    }

    CerboGX(const CerboGX&) = delete;
    CerboGX& operator=(const CerboGX&) = delete;

    bool connect()
    {
        disconnect();

        mosquitto_lib_init();

        mosq_ = mosquitto_new(nullptr, true, this);
        if (!mosq_)
        {
            last_error_ = "mosquitto_new() failed";
            return false;
        }

        mosquitto_message_callback_set(mosq_, &CerboGX::on_message_static);

        const int rc = mosquitto_connect(mosq_, ip_.c_str(), mqtt_port_, 60);
        if (rc != MOSQ_ERR_SUCCESS)
        {
            last_error_ = mosquitto_strerror(rc);
            disconnect();
            return false;
        }

        const int sub_rc = mosquitto_subscribe(mosq_, nullptr, "N/#", 0);
        if (sub_rc != MOSQ_ERR_SUCCESS)
        {
            last_error_ = mosquitto_strerror(sub_rc);
            disconnect();
            return false;
        }

        const int loop_rc = mosquitto_loop_start(mosq_);
        if (loop_rc != MOSQ_ERR_SUCCESS)
        {
            last_error_ = mosquitto_strerror(loop_rc);
            disconnect();
            return false;
        }

        connected_ = true;

        if (!wait_for_portal_id(2000))
        {
            last_error_ = "MQTT connected, but portal ID was not observed on N/#";
            return false;
        }

        request_full_publish(false);
        wait_for_full_publish(3000);

        return true;
    }

    void disconnect()
    {
        multiplus_devices_.clear();
        smartshunt_devices_.clear();

        if (mosq_)
        {
            mosquitto_loop_stop(mosq_, true);
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            topic_cache_.clear();
            discovered_services_.clear();
            portal_id_.clear();
            full_publish_completed_ = false;
        }

        connected_ = false;
        mosquitto_lib_cleanup();
    }

    bool is_connected() const noexcept
    {
        return connected_;
    }

    const std::string& ip() const noexcept
    {
        return ip_;
    }

    int port() const noexcept
    {
        return port_;
    }

    int mqtt_port() const noexcept
    {
        return mqtt_port_;
    }

    std::string last_error() const
    {
        return last_error_;
    }

    std::optional<SystemInfo> info() const
    {
        if (!connected_)
            return std::nullopt;

        SystemInfo info;

        info.serial = first_string("system", 0, {"Serial", "serial"}).value_or("");
        info.battery_voltage_v = first_double("system", 0, {"Dc/Battery/Voltage"});
        info.battery_current_a = first_double("system", 0, {"Dc/Battery/Current"});

        if (const auto p = first_double("system", 0, {"Dc/Battery/Power"}))
            info.battery_power_w = static_cast<int32_t>(std::llround(*p));

        info.battery_soc_pct = first_double("system", 0, {"Dc/Battery/Soc", "Soc"});
        info.battery_state = first_int("system", 0, {"Dc/Battery/State", "State"});

        if (info.serial.empty() &&
            !info.battery_voltage_v &&
            !info.battery_current_a &&
            !info.battery_power_w &&
            !info.battery_soc_pct &&
            !info.battery_state)
        {
            return std::nullopt;
        }

        return info;
    }

    void scan()
    {
        multiplus_devices_.clear();
        smartshunt_devices_.clear();

        if (!connected_)
            return;

        request_full_publish(true);
        wait_for_full_publish(2000);

        std::set<int> mp_seen;
        std::set<int> bat_seen;

        std::vector<std::pair<std::string, int>> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& item : discovered_services_)
                snapshot.push_back(item);
        }

        for (const auto& [service, instance] : snapshot)
        {
            if (service == "vebus")
            {
                if (!mp_seen.count(instance))
                {
                    multiplus_devices_.emplace_back(this, instance);
                    mp_seen.insert(instance);
                }
            }
            else if (service == "battery")
            {
                if (!bat_seen.count(instance))
                {
                    smartshunt_devices_.emplace_back(this, instance);
                    bat_seen.insert(instance);
                }
            }
        }
    }

    const std::vector<MultiPlusDevice>& multiplus_devices() const noexcept
    {
        return multiplus_devices_;
    }

    const std::vector<SmartShuntDevice>& smartshunt_devices() const noexcept
    {
        return smartshunt_devices_;
    }

private:
    friend class Device;
    friend class SmartShuntDevice;
    friend class MultiPlusDevice;

    static void on_message_static(struct mosquitto*, void* obj, const struct mosquitto_message* msg)
    {
        if (!obj || !msg || !msg->topic)
            return;

        static_cast<CerboGX*>(obj)->on_message(msg);
    }

    void on_message(const struct mosquitto_message* msg)
    {
        const std::string topic = msg->topic ? msg->topic : "";
        const std::string payload =
            (msg->payload && msg->payloadlen > 0)
                ? std::string(static_cast<const char*>(msg->payload),
                              static_cast<std::size_t>(msg->payloadlen))
                : std::string();

        const auto parts = split_topic(topic);
        if (parts.size() < 2 || parts[0] != "N")
            return;

        std::lock_guard<std::mutex> lock(mutex_);

        if (portal_id_.empty())
            portal_id_ = parts[1];

        topic_cache_[topic] = payload;

        if (parts.size() >= 4)
        {
            try
            {
                const int instance = std::stoi(parts[3]);
                discovered_services_.insert({parts[2], instance});
            }
            catch (...)
            {
            }
        }

        if (parts.size() == 3 && parts[2] == "full_publish_completed")
            full_publish_completed_ = true;
    }

    bool wait_for_portal_id(int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

        while (std::chrono::steady_clock::now() < deadline)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!portal_id_.empty())
                    return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        return false;
    }

    bool wait_for_full_publish(int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

        while (std::chrono::steady_clock::now() < deadline)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (full_publish_completed_)
                    return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        return false;
    }

    void request_full_publish(bool reset_flag)
    {
        std::string portal;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            portal = portal_id_;
            if (reset_flag)
                full_publish_completed_ = false;
        }

        if (portal.empty() || !mosq_)
            return;

        const std::string topic = "R/" + portal + "/keepalive";
        const int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), 0, nullptr, 0, false);
        if (rc != MOSQ_ERR_SUCCESS)
            last_error_ = mosquitto_strerror(rc);
    }

    std::optional<std::string> raw_topic_value(const std::string& topic) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = topic_cache_.find(topic);
        if (it == topic_cache_.end())
            return std::nullopt;
        return it->second;
    }

    std::optional<std::string> first_string(const std::string& service,
                                            int instance,
                                            const std::vector<std::string>& suffixes) const
    {
        for (const auto& suffix : suffixes)
        {
            const auto topic = make_topic(service, instance, suffix);
            const auto raw = raw_topic_value(topic);
            const auto tok = extract_json_value_token(raw.value_or(""));
            if (tok)
                return tok;
        }

        return std::nullopt;
    }

    std::optional<double> first_double(const std::string& service,
                                       int instance,
                                       const std::vector<std::string>& suffixes) const
    {
        for (const auto& suffix : suffixes)
        {
            const auto topic = make_topic(service, instance, suffix);
            const auto raw = raw_topic_value(topic);
            const auto tok = extract_json_value_token(raw.value_or(""));
            const auto val = token_to_double(tok);
            if (val)
                return val;
        }

        return std::nullopt;
    }

    std::optional<int> first_int(const std::string& service,
                                 int instance,
                                 const std::vector<std::string>& suffixes) const
    {
        for (const auto& suffix : suffixes)
        {
            const auto topic = make_topic(service, instance, suffix);
            const auto raw = raw_topic_value(topic);
            const auto tok = extract_json_value_token(raw.value_or(""));
            const auto val = token_to_int(tok);
            if (val)
                return val;
        }

        return std::nullopt;
    }

    std::string make_topic(const std::string& service, int instance, const std::string& suffix) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (portal_id_.empty())
            return {};
        return "N/" + portal_id_ + "/" + service + "/" + std::to_string(instance) + "/" + suffix;
    }

    static std::vector<std::string> split_topic(const std::string& s, char delim = '/')
    {
        std::vector<std::string> parts;
        std::string cur;

        for (char c : s)
        {
            if (c == delim)
            {
                parts.push_back(cur);
                cur.clear();
            }
            else
            {
                cur.push_back(c);
            }
        }

        parts.push_back(cur);
        return parts;
    }

    static std::optional<std::string> extract_json_value_token(const std::string& payload)
    {
        const std::string needle = "\"value\"";
        std::size_t p = payload.find(needle);
        if (p == std::string::npos)
            return std::nullopt;

        p = payload.find(':', p + needle.size());
        if (p == std::string::npos)
            return std::nullopt;
        ++p;

        while (p < payload.size() && std::isspace(static_cast<unsigned char>(payload[p])))
            ++p;

        if (p >= payload.size())
            return std::nullopt;

        if (payload[p] == '"')
        {
            ++p;
            std::size_t end = p;
            while (end < payload.size())
            {
                if (payload[end] == '"' && end > p && payload[end - 1] != '\\')
                    break;
                ++end;
            }

            if (end >= payload.size())
                return std::nullopt;

            return payload.substr(p, end - p);
        }

        std::size_t end = p;
        while (end < payload.size() && payload[end] != ',' && payload[end] != '}')
            ++end;

        std::string token = payload.substr(p, end - p);

        std::size_t first = 0;
        while (first < token.size() && std::isspace(static_cast<unsigned char>(token[first])))
            ++first;

        std::size_t last = token.size();
        while (last > first && std::isspace(static_cast<unsigned char>(token[last - 1])))
            --last;

        if (first >= last)
            return std::nullopt;

        token = token.substr(first, last - first);
        if (token == "null")
            return std::nullopt;

        return token;
    }

    static std::optional<double> token_to_double(const std::optional<std::string>& tok)
    {
        if (!tok)
            return std::nullopt;

        try
        {
            return std::stod(*tok);
        }
        catch (...)
        {
            return std::nullopt;
        }
    }

    static std::optional<int> token_to_int(const std::optional<std::string>& tok)
    {
        if (!tok)
            return std::nullopt;

        try
        {
            return std::stoi(*tok);
        }
        catch (...)
        {
            return std::nullopt;
        }
    }

    std::string ip_;
    int port_ = 502;
    int mqtt_port_ = 1883;

    mutable std::mutex mutex_;
    struct mosquitto* mosq_ = nullptr;
    bool connected_ = false;
    std::string last_error_;

    std::string portal_id_;
    bool full_publish_completed_ = false;
    std::map<std::string, std::string> topic_cache_;
    std::set<std::pair<std::string, int>> discovered_services_;

    std::vector<MultiPlusDevice> multiplus_devices_;
    std::vector<SmartShuntDevice> smartshunt_devices_;
};

inline std::optional<SmartShuntDevice::Info> SmartShuntDevice::read() const
{
    if (!parent_ || !parent_->is_connected())
        return std::nullopt;

    Info d;
    d.instance = instance_;

    d.connected = parent_->first_int("battery", instance_, {"Connected"});
    d.consumed_amphours = parent_->first_double("battery", instance_, {"ConsumedAmphours"});
    d.custom_name = parent_->first_string("battery", instance_, {"CustomName"});

    d.voltage_v = parent_->first_double("battery", instance_, {"Dc/0/Voltage"});
    d.current_a = parent_->first_double("battery", instance_, {"Dc/0/Current"});
    d.power_w = parent_->first_double("battery", instance_, {"Dc/0/Power"});
    d.midpoint_voltage_v = parent_->first_double("battery", instance_, {"Dc/0/MidVoltage"});
    d.midpoint_deviation_pct = parent_->first_double("battery", instance_, {"Dc/0/MidVoltageDeviation"});
    d.temperature_c = parent_->first_double("battery", instance_, {"Dc/0/Temperature"});
    d.aux_voltage_v = parent_->first_double("battery", instance_, {"Dc/1/Voltage"});

    d.device_instance = parent_->first_int("battery", instance_, {"DeviceInstance"});
    d.firmware_version = parent_->first_int("battery", instance_, {"FirmwareVersion"});
    d.group_id = parent_->first_int("battery", instance_, {"GroupId"});
    d.hardware_version = parent_->first_int("battery", instance_, {"HardwareVersion"});

    d.device0.custom_name = parent_->first_string("battery", instance_, {"Devices/0/CustomName"});
    d.device0.device_instance = parent_->first_int("battery", instance_, {"Devices/0/DeviceInstance"});
    d.device0.firmware_version = parent_->first_int("battery", instance_, {"Devices/0/FirmwareVersion"});
    d.device0.product_id = parent_->first_int("battery", instance_, {"Devices/0/ProductId"});
    d.device0.product_name = parent_->first_string("battery", instance_, {"Devices/0/ProductName"});
    d.device0.service_name = parent_->first_string("battery", instance_, {"Devices/0/ServiceName"});
    d.device0.vreg_link = parent_->first_string("battery", instance_, {"Devices/0/VregLink"});

    d.history.automatic_syncs = parent_->first_int("battery", instance_, {"History/AutomaticSyncs"});
    d.history.average_discharge = parent_->first_double("battery", instance_, {"History/AverageDischarge"});
    d.history.charge_cycles = parent_->first_int("battery", instance_, {"History/ChargeCycles"});
    d.history.charged_energy = parent_->first_double("battery", instance_, {"History/ChargedEnergy"});
    d.history.deepest_discharge = parent_->first_double("battery", instance_, {"History/DeepestDischarge"});
    d.history.discharged_energy = parent_->first_double("battery", instance_, {"History/DischargedEnergy"});
    d.history.full_discharges = parent_->first_int("battery", instance_, {"History/FullDischarges"});
    d.history.high_starter_voltage_alarms = parent_->first_int("battery", instance_, {"History/HighStarterVoltageAlarms"});
    d.history.high_voltage_alarms = parent_->first_int("battery", instance_, {"History/HighVoltageAlarms"});
    d.history.last_discharge = parent_->first_double("battery", instance_, {"History/LastDischarge"});
    d.history.low_starter_voltage_alarms = parent_->first_int("battery", instance_, {"History/LowStarterVoltageAlarms"});
    d.history.low_voltage_alarms = parent_->first_int("battery", instance_, {"History/LowVoltageAlarms"});
    d.history.maximum_starter_voltage = parent_->first_double("battery", instance_, {"History/MaximumStarterVoltage"});
    d.history.maximum_voltage = parent_->first_double("battery", instance_, {"History/MaximumVoltage"});
    d.history.minimum_starter_voltage = parent_->first_double("battery", instance_, {"History/MinimumStarterVoltage"});
    d.history.minimum_voltage = parent_->first_double("battery", instance_, {"History/MinimumVoltage"});
    d.history.time_since_last_full_charge = parent_->first_int("battery", instance_, {"History/TimeSinceLastFullCharge"});
    d.history.total_ah_drawn = parent_->first_double("battery", instance_, {"History/TotalAhDrawn"});

    d.mgmt_connection = parent_->first_string("battery", instance_, {"Mgmt/Connection"});
    d.mgmt_process_name = parent_->first_string("battery", instance_, {"Mgmt/ProcessName"});
    d.mgmt_process_version = parent_->first_string("battery", instance_, {"Mgmt/ProcessVersion"});

    d.product_id = parent_->first_int("battery", instance_, {"ProductId"});
    d.product_name = parent_->first_string("battery", instance_, {"ProductName"});
    d.relay_state = parent_->first_int("battery", instance_, {"Relay/0/State"});
    d.serial = parent_->first_string("battery", instance_, {"Serial"});

    d.settings.has_mid_voltage = parent_->first_int("battery", instance_, {"Settings/HasMidVoltage"});
    d.settings.has_starter_voltage = parent_->first_int("battery", instance_, {"Settings/HasStarterVoltage"});
    d.settings.has_temperature = parent_->first_int("battery", instance_, {"Settings/HasTemperature"});
    d.settings.relay_mode = parent_->first_int("battery", instance_, {"Settings/RelayMode"});

    d.soc_pct = parent_->first_double("battery", instance_, {"Soc"});
    d.time_to_go_s = parent_->first_double("battery", instance_, {"TimeToGo"});

    d.alarms.alarm = parent_->first_int("battery", instance_, {"Alarms/Alarm"});
    d.alarms.high_starter_voltage = parent_->first_int("battery", instance_, {"Alarms/HighStarterVoltage"});
    d.alarms.high_temperature = parent_->first_int("battery", instance_, {"Alarms/HighTemperature"});
    d.alarms.high_voltage = parent_->first_int("battery", instance_, {"Alarms/HighVoltage"});
    d.alarms.low_soc = parent_->first_int("battery", instance_, {"Alarms/LowSoc"});
    d.alarms.low_starter_voltage = parent_->first_int("battery", instance_, {"Alarms/LowStarterVoltage"});
    d.alarms.low_temperature = parent_->first_int("battery", instance_, {"Alarms/LowTemperature"});
    d.alarms.low_voltage = parent_->first_int("battery", instance_, {"Alarms/LowVoltage"});
    d.alarms.mid_voltage = parent_->first_int("battery", instance_, {"Alarms/MidVoltage"});

    d.vedirect.hex_checksum_errors = parent_->first_int("battery", instance_, {"VEDirect/HexChecksumErrors"});
    d.vedirect.hex_invalid_character_errors = parent_->first_int("battery", instance_, {"VEDirect/HexInvalidCharacterErrors"});
    d.vedirect.hex_unfinished_errors = parent_->first_int("battery", instance_, {"VEDirect/HexUnfinishedErrors"});
    d.vedirect.text_checksum_errors = parent_->first_int("battery", instance_, {"VEDirect/TextChecksumErrors"});
    d.vedirect.text_parse_error = parent_->first_int("battery", instance_, {"VEDirect/TextParseError"});
    d.vedirect.text_unfinished_errors = parent_->first_int("battery", instance_, {"VEDirect/TextUnfinishedErrors"});

    if (!d.connected &&
        !d.consumed_amphours &&
        !d.voltage_v &&
        !d.current_a &&
        !d.power_w &&
        !d.soc_pct &&
        !d.serial &&
        !d.product_name)
    {
        return std::nullopt;
    }

    return d;
}

inline std::optional<MultiPlusDevice::Info> MultiPlusDevice::read() const
{
    if (!parent_ || !parent_->is_connected())
        return std::nullopt;

    Info d;
    d.instance = instance_;
    d.dc_voltage_v = parent_->first_double("vebus", instance_, {"Dc/0/Voltage", "Dc/Voltage", "Voltage"});
    d.dc_current_a = parent_->first_double("vebus", instance_, {"Dc/0/Current", "Dc/Current", "Current"});
    d.soc_pct = parent_->first_double("vebus", instance_, {"Soc"});
    d.state = parent_->first_int("vebus", instance_, {"State"});
    d.mode = parent_->first_int("vebus", instance_, {"Mode"});
    d.out_l1_power_w = parent_->first_double("vebus", instance_, {"Ac/Out/L1/P", "Ac/Out/L1/Power"});
    d.out_l2_power_w = parent_->first_double("vebus", instance_, {"Ac/Out/L2/P", "Ac/Out/L2/Power"});
    d.out_l3_power_w = parent_->first_double("vebus", instance_, {"Ac/Out/L3/P", "Ac/Out/L3/Power"});

    int phases = 0;
    if (d.out_l1_power_w) ++phases;
    if (d.out_l2_power_w) ++phases;
    if (d.out_l3_power_w) ++phases;
    if (phases > 0)
        d.phase_count = phases;

    if (!d.dc_voltage_v && !d.dc_current_a && !d.soc_pct &&
        !d.state && !d.mode && !d.out_l1_power_w && !d.out_l2_power_w && !d.out_l3_power_w)
    {
        return std::nullopt;
    }

    return d;
}

} // namespace cerbo