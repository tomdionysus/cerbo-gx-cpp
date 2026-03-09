#pragma once

#include <modbus/modbus.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace cerbo
{

inline double scale_u16(uint16_t v, double factor)
{
    return static_cast<double>(v) / factor;
}

inline double scale_i16(int16_t v, double factor)
{
    return static_cast<double>(v) / factor;
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

inline std::string trim_right_nul_and_space(std::string s)
{
    while (!s.empty() &&
           (s.back() == '\0' || s.back() == ' ' || s.back() == '\t' ||
            s.back() == '\r' || s.back() == '\n'))
    {
        s.pop_back();
    }
    return s;
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

struct SystemInfo
{
    std::string serial;
    std::optional<double> battery_voltage_v;
    std::optional<double> battery_current_a;
    std::optional<int32_t> battery_power_w;
    std::optional<double> battery_soc_pct;
    std::optional<int> battery_state;
};

class ModbusConnection
{
public:
    ModbusConnection(std::string ip, int port)
        : ip_(std::move(ip)), port_(port)
    {
    }

    ~ModbusConnection()
    {
        disconnect();
    }

    ModbusConnection(const ModbusConnection&) = delete;
    ModbusConnection& operator=(const ModbusConnection&) = delete;

    bool connect()
    {
        disconnect();

        ctx_ = modbus_new_tcp(ip_.c_str(), port_);
        if (!ctx_)
            return false;

        if (modbus_connect(ctx_) != 0)
        {
            disconnect();
            return false;
        }

        return true;
    }

    void disconnect()
    {
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
    }

    bool is_connected() const
    {
        return ctx_ != nullptr;
    }

    const std::string& ip() const
    {
        return ip_;
    }

    int port() const
    {
        return port_;
    }

    std::string last_error() const
    {
        return modbus_strerror(errno);
    }

    bool read_u16(int unit_id, int addr, uint16_t& value) const
    {
        std::vector<uint16_t> regs;
        if (!read_holding(unit_id, addr, 1, regs))
            return false;
        value = regs[0];
        return true;
    }

    bool read_i16(int unit_id, int addr, int16_t& value) const
    {
        uint16_t raw = 0;
        if (!read_u16(unit_id, addr, raw))
            return false;
        value = static_cast<int16_t>(raw);
        return true;
    }

    bool read_i32_be_words(int unit_id, int addr, int32_t& value) const
    {
        std::vector<uint16_t> regs;
        if (!read_holding(unit_id, addr, 2, regs))
            return false;

        uint32_t u =
            (static_cast<uint32_t>(regs[0]) << 16) |
            static_cast<uint32_t>(regs[1]);

        value = static_cast<int32_t>(u);
        return true;
    }

    bool read_string_regs(int unit_id, int addr, int reg_count, std::string& out) const
    {
        std::vector<uint16_t> regs;
        if (!read_holding(unit_id, addr, reg_count, regs))
            return false;

        std::string s;
        s.reserve(static_cast<size_t>(reg_count) * 2);

        for (uint16_t r : regs)
        {
            char hi = static_cast<char>((r >> 8) & 0xFF);
            char lo = static_cast<char>(r & 0xFF);

            if (hi != '\0') s.push_back(hi);
            if (lo != '\0') s.push_back(lo);
        }

        out = trim_right_nul_and_space(s);
        return true;
    }

private:
    bool read_holding(int unit_id, int addr, int count, std::vector<uint16_t>& out) const
    {
        if (!ctx_)
            return false;

        if (modbus_set_slave(ctx_, unit_id) != 0)
            return false;

        out.assign(static_cast<size_t>(count), 0);
        int rc = modbus_read_registers(ctx_, addr, count, out.data());
        if (rc != count)
        {
            out.clear();
            return false;
        }

        return true;
    }

    std::string ip_;
    int port_ = 502;
    modbus_t* ctx_ = nullptr;
};

class SmartShuntDevice
{
public:
    struct Info
    {
        int unit_id = -1;
        std::optional<double> voltage_v;
        std::optional<double> current_a;
        std::optional<double> soc_pct;
        std::optional<int32_t> power_w;
    };

    SmartShuntDevice() = default;

    SmartShuntDevice(const ModbusConnection* conn, int unit_id)
        : conn_(conn), unit_id_(unit_id)
    {
    }

    int unit_id() const
    {
        return unit_id_;
    }

    std::optional<Info> read() const
    {
        if (!conn_)
            return std::nullopt;

        Info d;
        d.unit_id = unit_id_;

        uint16_t u16 = 0;
        int16_t i16 = 0;
        int32_t i32 = 0;
        bool got_any = false;

        if (conn_->read_i32_be_words(unit_id_, 256, i32))
        {
            d.power_w = i32;
            got_any = true;
        }
        else if (conn_->read_i16(unit_id_, 258, i16))
        {
            d.power_w = static_cast<int32_t>(i16);
            got_any = true;
        }

        if (conn_->read_u16(unit_id_, 259, u16))
        {
            d.voltage_v = scale_u16(u16, 100.0);
            got_any = true;
        }

        if (conn_->read_i16(unit_id_, 261, i16))
        {
            d.current_a = scale_i16(i16, 10.0);
            got_any = true;
        }

        if (conn_->read_u16(unit_id_, 266, u16))
        {
            d.soc_pct = static_cast<double>(u16) / 10.0;
            got_any = true;
        }

        if (!got_any)
            return std::nullopt;

        return d;
    }

    bool matches_signature() const
    {
        auto info = read();
        if (!info)
            return false;

        int score = 0;
        if (info->voltage_v && plausible_voltage(*info->voltage_v)) ++score;
        if (info->current_a && plausible_current(*info->current_a)) ++score;
        if (info->soc_pct && plausible_soc(*info->soc_pct)) ++score;

        return score >= 2;
    }

private:
    const ModbusConnection* conn_ = nullptr;
    int unit_id_ = -1;
};

class MultiPlusDevice
{
public:
    struct Info
    {
        int unit_id = -1;
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

    MultiPlusDevice(const ModbusConnection* conn, int unit_id)
        : conn_(conn), unit_id_(unit_id)
    {
    }

    int unit_id() const
    {
        return unit_id_;
    }

    std::optional<Info> read() const
    {
        if (!conn_)
            return std::nullopt;

        Info d;
        d.unit_id = unit_id_;

        uint16_t u16 = 0;
        int16_t i16 = 0;
        bool got_any = false;

        if (conn_->read_u16(unit_id_, 26, u16))
        {
            d.dc_voltage_v = scale_u16(u16, 100.0);
            got_any = true;
        }

        if (conn_->read_i16(unit_id_, 27, i16))
        {
            d.dc_current_a = scale_i16(i16, 10.0);
            got_any = true;
        }

        if (conn_->read_u16(unit_id_, 30, u16))
        {
            d.soc_pct = static_cast<double>(u16) / 10.0;
            got_any = true;
        }

        if (conn_->read_u16(unit_id_, 31, u16))
        {
            d.state = static_cast<int>(u16);
            got_any = true;
        }

        if (conn_->read_u16(unit_id_, 33, u16))
        {
            d.mode = static_cast<int>(u16);
            got_any = true;
        }

        if (conn_->read_u16(unit_id_, 28, u16))
        {
            d.phase_count = static_cast<int>(u16);
            got_any = true;
        }

        if (conn_->read_i16(unit_id_, 23, i16))
        {
            d.out_l1_power_w = static_cast<double>(i16);
            got_any = true;
        }

        if (conn_->read_i16(unit_id_, 24, i16))
        {
            d.out_l2_power_w = static_cast<double>(i16);
            got_any = true;
        }

        if (conn_->read_i16(unit_id_, 25, i16))
        {
            d.out_l3_power_w = static_cast<double>(i16);
            got_any = true;
        }

        if (!got_any)
            return std::nullopt;

        return d;
    }

    bool matches_signature() const
    {
        auto info = read();
        if (!info)
            return false;

        int score = 0;
        if (info->dc_voltage_v && plausible_voltage(*info->dc_voltage_v)) ++score;
        if (info->dc_current_a && plausible_current(*info->dc_current_a)) ++score;
        if (info->soc_pct && plausible_soc(*info->soc_pct)) ++score;
        if (info->mode && *info->mode >= 1 && *info->mode <= 4) ++score;

        if (info->state)
        {
            int s = *info->state;
            if (s == 0 || s == 1 || s == 2 || s == 3 || s == 4 || s == 5 ||
                s == 6 || s == 7 || s == 8 || s == 9 || s == 10 || s == 11 ||
                s == 244 || s == 252)
            {
                ++score;
            }
        }

        return score >= 3;
    }

private:
    const ModbusConnection* conn_ = nullptr;
    int unit_id_ = -1;
};

class CerboGX
{
public:
    explicit CerboGX(std::string ip, int port = 502)
        : conn_(std::move(ip), port)
    {
    }

    bool connect()
    {
        return conn_.connect();
    }

    void disconnect()
    {
        multiplus_devices_.clear();
        smartshunt_devices_.clear();
        conn_.disconnect();
    }

    bool is_connected() const
    {
        return conn_.is_connected();
    }

    const std::string& ip() const
    {
        return conn_.ip();
    }

    int port() const
    {
        return conn_.port();
    }

    std::string last_error() const
    {
        return conn_.last_error();
    }

    std::optional<SystemInfo> info() const
    {
        if (!conn_.is_connected())
            return std::nullopt;

        SystemInfo info;

        std::string serial;
        if (conn_.read_string_regs(100, 800, 6, serial))
            info.serial = serial;

        uint16_t u16 = 0;
        int16_t i16 = 0;

        if (conn_.read_u16(100, 840, u16))
            info.battery_voltage_v = scale_u16(u16, 10.0);

        if (conn_.read_i16(100, 841, i16))
            info.battery_current_a = scale_i16(i16, 10.0);

        if (conn_.read_i16(100, 842, i16))
            info.battery_power_w = static_cast<int32_t>(i16);

        if (conn_.read_u16(100, 843, u16))
            info.battery_soc_pct = static_cast<double>(u16);

        if (conn_.read_u16(100, 844, u16))
            info.battery_state = static_cast<int>(u16);

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

        if (!conn_.is_connected())
            return;

        std::set<int> seen_multiplus;
        std::set<int> seen_smartshunt;

        for (int unit = 1; unit <= 247; ++unit)
        {
            if (unit == 100)
                continue;

            MultiPlusDevice mp(&conn_, unit);
            if (mp.matches_signature() && !seen_multiplus.count(unit))
            {
                multiplus_devices_.push_back(mp);
                seen_multiplus.insert(unit);
            }

            SmartShuntDevice ss(&conn_, unit);
            if (ss.matches_signature() && !seen_smartshunt.count(unit))
            {
                smartshunt_devices_.push_back(ss);
                seen_smartshunt.insert(unit);
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
    ModbusConnection conn_;
    std::vector<MultiPlusDevice> multiplus_devices_;
    std::vector<SmartShuntDevice> smartshunt_devices_;
};

} // namespace cerbo