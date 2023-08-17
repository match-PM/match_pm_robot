#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using PneumaticId = PMClient::PneumaticId;

struct PneumaticState
{
    PneumaticId id;
    std::string name;

    double move_forward_cmd = 0.0;
    double move_backward_cmd = 0.0;
    double is_forward = 0.0;
    double is_backward = 0.0;

    explicit PneumaticState(PneumaticId my_id) : id(my_id)
    {
        switch (my_id)
        {
            case PneumaticId::UV1:
                name = "UV1_Pneumatic";
                break;
            case PneumaticId::UV2:
                name = "UV2_Pneumatic";
                break;
            case PneumaticId::Glue:
                name = "Glue_Pneumatic";
                break;
            case PneumaticId::Glue2K:
                name = "Glue_2K_Pneumatic";
                break;
            case PneumaticId::CameraMire:
                name = "Camera_Mire_Pneumatic";
                break;
            case PneumaticId::ProtectDoseur:
                name = "Protect_Doseur_Pneumatic";
                break;
        }
    }

    void add_state_interfaces(std::vector<StateInterface> &interfaces)
    {
        interfaces.emplace_back(StateInterface(this->name, "Is_Forward", &this->is_forward));
        interfaces.emplace_back(StateInterface(this->name, "Is_Backward", &this->is_backward));
    }

    void add_command_interfaces(std::vector<CommandInterface> &interfaces)
    {
        interfaces.emplace_back(
            CommandInterface(this->name, "Move_Forward", &this->move_forward_cmd)
        );
        interfaces.emplace_back(
            CommandInterface(this->name, "Move_Backward", &this->move_backward_cmd)
        );
    }

    void read(PMClient::Robot &robot)
    {
        auto &pm_pneumatic = robot.get_pneumatic(this->id);
        this->is_forward = static_cast<double>(pm_pneumatic.get_is_forward());
        this->is_backward = static_cast<double>(pm_pneumatic.get_is_backward());
    }

    void write(PMClient::Robot &robot)
    {
        auto &pm_pneumatic = robot.get_pneumatic(this->id);
        if (this->move_forward_cmd && !this->move_backward_cmd)
            pm_pneumatic.move_forward();
        if (!this->move_forward_cmd && this->move_backward_cmd)
            pm_pneumatic.move_backward();
    }
};

} // namespace pm_hardware_interface
