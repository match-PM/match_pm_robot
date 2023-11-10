#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using PneumaticId = PMClient::PneumaticId;

using Position = PMClient::Position;

struct PneumaticState
{
    PneumaticId id;
    std::string name;

    double position = -1.0;
    double move_command = -1.0;

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
        interfaces.emplace_back(StateInterface(this->name, "Position", &this->position));
    }

    void add_command_interfaces(std::vector<CommandInterface> &interfaces)
    {
        interfaces.emplace_back(CommandInterface(this->name, "Move_Command", &this->move_command));
    }

    void read(PMClient::Robot &robot)
    {
        auto &pm_pneumatic = robot.get_pneumatic(this->id);

        switch (pm_pneumatic.get_position())
        {
            case Position::Forward:
                this->position = 1.0;
                break;
            case Position::Neutral:
                this->position = 0.0;
                break;
            case Position::Backward:
                this->position = -1.0;
                break;
        }
    }

    void write(PMClient::Robot &robot)
    {
        auto &pm_pneumatic = robot.get_pneumatic(this->id);

        if (this->move_command > 0.0)
        {
            pm_pneumatic.move(Position::Forward);
        }
        else if (this->move_command < 0.0)
        {
            pm_pneumatic.move(Position::Backward);
        }
        else
        {
            pm_pneumatic.move(Position::Neutral);
        }
    }
};

} // namespace pm_hardware_interface
