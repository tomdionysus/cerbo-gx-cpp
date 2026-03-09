#pragma once

#include <mosquitto.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace cerbo
{

class CerboGX;

inline double scale_double(double v, double factor)
{
    return v / factor;
}

inline bool plausible_voltage(double v)
{
    return v > 1.0 && v < 100.0;
}

inline bool plausible_soc(double soc)
{
    return soc >= 0.0 && soc <= 100.0;
}

inline bool plausible_current(double a)
{
    return std::fabs(a) < 5000.0;
}

inline std::string battery_state_to_string(int state)
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

inline std::string vebus_mode_to_string(int mode)
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

inline std::string vebus_state_to_string(int state)
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

inline std::vector<std::string> split_topic(const std::string& s, char delim = '/')
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

inline std::optional<std::string> extract_json_value_token(const std::string& payload)
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
            if (payload[end] == '"' && payload[end - 1] != '\\')
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

inline std::optional<double> token_to_double(const std::optional<std::string>& tok)
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

inline std::optional<int> token_to_int(const std::optional<std::string>& tok)
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

struct SystemInfo
{
    std::string serial;
    std::optional<double> battery_voltage_v;
    std::optional<double> battery_current_a;
    std::optional<int32_t> battery_power_w;
    std::optional<double> battery_soc_pct;
    std::optional<int> battery_state;
};

class SmartShuntDevice
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
    SmartShuntDevice(const CerboGX* parent, int instance);

    int instance() const { return instance_; }
    std::optional<Info> read() const;

private:
    const CerboGX* parent_ = nullptr;
    int instance_ = -1;
};

class MultiPlusDevice
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
    MultiPlusDevice(const CerboGX* parent, int instance);

    int instance() const { return instance_; }
    std::optional<Info> read() const;

private:
    const CerboGX* parent_ = nullptr;
    int instance_ = -1;
};

class CerboGX
{
public:
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
            return false;

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

    bool is_connected() const
    {
        return connected_;
    }

    const std::string& ip() const
    {
        return ip_;
    }

    int port() const
    {
        return port_;
    }

    int mqtt_port() const
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
        if (auto p = first_double("system", 0, {"Dc/Battery/Power"}))
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

    const std::vector<MultiPlusDevice>& multiplus_devices() const
    {
        return multiplus_devices_;
    }

    const std::vector<SmartShuntDevice>& smartshunt_devices() const
    {
        return smartshunt_devices_;
    }

private:
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
        auto it = topic_cache_.find(topic);
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

    std::string ip_;
    int port_ = 502;          // kept for API compatibility
    int mqtt_port_ = 1883;    // actual transport used in this refactor

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

inline SmartShuntDevice::SmartShuntDevice(const CerboGX* parent, int instance)
    : parent_(parent), instance_(instance)
{
}

inline std::optional<SmartShuntDevice::Info> SmartShuntDevice::read() const
{
    if (!parent_ || !parent_->is_connected())
        return std::nullopt;

    Info d;
    d.instance = instance_;
    d.voltage_v = parent_->first_double("battery", instance_, {"Dc/0/Voltage", "Dc/Voltage", "Voltage"});
    d.current_a = parent_->first_double("battery", instance_, {"Dc/0/Current", "Dc/Current", "Current"});
    d.soc_pct = parent_->first_double("battery", instance_, {"Soc"});
    if (auto p = parent_->first_double("battery", instance_, {"Dc/0/Power", "Dc/Power", "Power"}))
        d.power_w = static_cast<int32_t>(std::llround(*p));

    if (!d.voltage_v && !d.current_a && !d.soc_pct && !d.power_w)
        return std::nullopt;

    return d;
}

inline MultiPlusDevice::MultiPlusDevice(const CerboGX* parent, int instance)
    : parent_(parent), instance_(instance)
{
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