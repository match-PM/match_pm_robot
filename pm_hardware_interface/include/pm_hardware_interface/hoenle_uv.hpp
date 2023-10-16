#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

struct HoenleUVState
{
    std::string name{"HoenleUV"};

    std::array<double, 4> on_off_state{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> on_off_cmd{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> power_cmd{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> time_cmd{0.0, 0.0, 0.0, 0.0};

    explicit HoenleUVState()
    {
    }

    void add_state_interfaces(std::vector<StateInterface> &interfaces)
    {
        for (auto i = 0; i < 4; i++)
        {
            interfaces.emplace_back(
                StateInterface(this->name, "OnOff_" + std::to_string(i), &this->on_off_state[i])
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
        auto state = robot.hoenle_uv->get_on_off();
        for (auto i = 0; i < 4; i++)
        {
            this->on_off_state[i] = static_cast<double>(state[i]);
        }
    }

    void write(PMClient::Robot &robot)
    {
        std::array<bool, 4> b_on_off_cmd;
        std::array<int, 4> i_power_cmd;

        for (auto i = 0; i < 4; i++)
        {
            b_on_off_cmd[i] = static_cast<bool>(this->on_off_cmd[i]);
            i_power_cmd[i] = static_cast<bool>(this->power_cmd[i]);
        }

        robot.hoenle_uv->set_on_off(b_on_off_cmd);
        robot.hoenle_uv->set_power(i_power_cmd);
        robot.hoenle_uv->set_time(this->time_cmd);
    }
};

} // namespace pm_hardware_interface
