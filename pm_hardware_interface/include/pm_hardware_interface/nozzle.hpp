#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using NozzleId = PMClient::NozzleId;

using double_limits = std::numeric_limits<double>;

struct NozzleState
{
    NozzleId id;
    std::string name;

    double pressure = 0.0;
    double pressure_cmd = double_limits::quiet_NaN();

    bool initialized = false;

    explicit NozzleState(NozzleId my_id) : id(my_id)
    {
        switch (my_id)
        {
            case NozzleId::Head:
                name = "Head_Nozzle";
                break;
            case NozzleId::Gonio:
                name = "Gonio_Nozzle";
                break;
            case NozzleId::Nest:
                name = "Nest_Nozzle";
                break;
            case NozzleId::DoseurGlue:
                name = "Doseur_Nozzle";
                break;
            case NozzleId::DoseurGlue2K:
                name = "Glue_2K_Doseur";
                break;
        }
    }

    void add_state_interfaces(std::vector<StateInterface> &interfaces)
    {
        interfaces.emplace_back(StateInterface(this->name, "State", &this->pressure));
    }

    void add_command_interfaces(std::vector<CommandInterface> &interfaces)
    {
        interfaces.emplace_back(CommandInterface(this->name, "State", &this->pressure_cmd));
    }

    void read(PMClient::Robot &robot)
    {
        auto &pm_nozzle = robot.get_nozzle(this->id);
        auto state = pm_nozzle.get_state();
        switch (state)
        {
            case PMClient::NozzleState::Air:
                this->pressure = 1.0;
                break;
            case PMClient::NozzleState::Off:
                this->pressure = 0.0;
                break;
            case PMClient::NozzleState::Vacuum:
                this->pressure = -1.0;
                break;
        }

        if (!initialized)
        {
            this->pressure_cmd = this->pressure;
            initialized = true;
        }
    }

    void write(PMClient::Robot &robot)
    {
        if (!std::isnan(this->pressure_cmd))
        {
            auto &pm_nozzle = robot.get_nozzle(this->id);
            if (this->pressure_cmd > 0.0)
            {
                pm_nozzle.set_state(PMClient::NozzleState::Air);
            }
            else if (this->pressure_cmd < 0.0)
            {
                pm_nozzle.set_state(PMClient::NozzleState::Vacuum);
            }
            else
            {
                pm_nozzle.set_state(PMClient::NozzleState::Off);
            }

            this->pressure_cmd = double_limits::quiet_NaN();
        }
    }
};

} // namespace pm_hardware_interface
