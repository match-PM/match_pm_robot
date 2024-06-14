#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using double_limits = std::numeric_limits<double>;

/**
 * Check if `arr` contains a value that is not NaN.
 */
bool has_number(const std::array<double, 4> &arr)
{
    for (const auto val : arr)
    {
        if (!std::isnan(val))
        {
            return true;
        }
    }
    return false;
}

/**
 * Copy values from `src` into `dst` where `dst` has NaN.
 */
void merge(std::array<double, 4> &dst, const std::array<double, 4> &src)
{
    for (auto i = 0; i < 4; ++i)
    {
        if (std::isnan(dst[i]))
        {
            dst[i] = src[i];
        }
    }
}

/**
 * Set all elements of `arr` to NaN.
 */
void fill_nan(std::array<double, 4> &arr)
{
    std::fill(std::begin(arr), std::end(arr), double_limits::quiet_NaN());
}

struct HoenleUVState
{
    std::string name{"HoenleUV"};

    std::array<double, 4> on_off_state{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> on_off_cmd{
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN()
    };
    std::array<double, 4> power_state{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> power_cmd{
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN()
    };
    std::array<double, 4> time_state{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> time_cmd{
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN(),
        double_limits::quiet_NaN()
    };

    void add_state_interfaces(std::vector<StateInterface> &interfaces)
    {
        for (auto i = 0; i < 4; i++)
        {
            interfaces.emplace_back(
                StateInterface(this->name, "OnOff_" + std::to_string(i), &this->on_off_state[i])
            );
            interfaces.emplace_back(
                StateInterface(this->name, "Power_" + std::to_string(i), &this->power_state[i])
            );
            interfaces.emplace_back(
                StateInterface(this->name, "Time_" + std::to_string(i), &this->time_state[i])
            );
        }
    }

    void add_command_interfaces(std::vector<CommandInterface> &interfaces)
    {

        for (auto i = 0; i < 4; i++)
        {
            interfaces.emplace_back(
                CommandInterface(this->name, "OnOff_" + std::to_string(i), &this->on_off_cmd[i])
            );
            interfaces.emplace_back(
                CommandInterface(this->name, "Power_" + std::to_string(i), &this->power_cmd[i])
            );
            interfaces.emplace_back(
                CommandInterface(this->name, "Time_" + std::to_string(i), &this->time_cmd[i])
            );
        }
    }

    void read(PMClient::Robot &robot)
    {
        auto on_off_state = robot.hoenle_uv->get_on_off();
        auto power_state = robot.hoenle_uv->get_power();
        auto time_state = robot.hoenle_uv->get_time();
        for (auto i = 0; i < 4; i++)
        {
            this->on_off_state[i] = static_cast<double>(on_off_state[i]);
            this->power_state[i] = static_cast<double>(power_state[i]);
            this->time_state[i] = static_cast<double>(time_state[i]);
        }
    }

    void write(PMClient::Robot &robot)
    {
        if (has_number(this->on_off_cmd))
        {
            merge(this->on_off_cmd, this->on_off_state);

            std::array<bool, 4> b_on_off_cmd;
            for (auto i = 0; i < 4; i++)
            {
                b_on_off_cmd[i] = static_cast<bool>(this->on_off_cmd[i]);
            }
            robot.hoenle_uv->set_on_off(b_on_off_cmd);

            fill_nan(this->on_off_cmd);
        }

        if (has_number(this->power_cmd))
        {
            merge(this->power_cmd, this->power_state);

            std::array<int, 4> i_power_cmd;
            for (auto i = 0; i < 4; i++)
            {
                i_power_cmd[i] = static_cast<int>(this->power_cmd[i]);
            }
            robot.hoenle_uv->set_power(i_power_cmd);

            fill_nan(this->power_cmd);
        }

        if (has_number(this->time_cmd))
        {
            merge(this->time_cmd, this->time_state);

            robot.hoenle_uv->set_time(this->time_cmd);

            fill_nan(this->time_cmd);
        }
    }
};

} // namespace pm_hardware_interface
