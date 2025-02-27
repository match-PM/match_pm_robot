#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using PneumaticId = PMClient::PneumaticId;

using Position = PMClient::Position;

using double_limits = std::numeric_limits<double>;

struct PneumaticState
{
    PneumaticId id;
    std::string name;

    double upper_limit = 0.0;
    double lower_limit = 0.0;
    double physical_position = 0.0;
    double fwd_or_bwd = -1.0;
    double move_command = double_limits::quiet_NaN();

    bool initialized = false;

    explicit PneumaticState(PneumaticId my_id) : id(my_id)
    {
        switch (my_id)
        {
            case PneumaticId::UV1:
                name = "UV_LED_Front_Joint";
                break;
            case PneumaticId::UV2:
                name = "UV_LED_Back_Joint";
                break;
            case PneumaticId::Glue:
                name = "1K_Dispenser_Joint";
                break;
            case PneumaticId::Glue2K:
                name = "2K_Dispenser_Joint";
                break;
            case PneumaticId::CameraMire:
                name = "Camera_Calibration_Platelet_Joint";
                break;
            case PneumaticId::ProtectDoseur:
                name = "1K_Dispenser_Protection_Joint";
                break;
        }
    }

    void add_state_interfaces(std::vector<StateInterface> &interfaces)
    {
        interfaces.emplace_back(StateInterface(this->name, "position", &this->physical_position));
        interfaces.emplace_back(StateInterface(this->name, "FwdOrBwd", &this->fwd_or_bwd));
    }

    void add_command_interfaces(std::vector<CommandInterface> &interfaces)
    {
        interfaces.emplace_back(CommandInterface(this->name, "UpperLimit", &this->upper_limit));
        interfaces.emplace_back(CommandInterface(this->name, "LowerLimit", &this->lower_limit));
        interfaces.emplace_back(CommandInterface(this->name, "Move_Command", &this->move_command));
    }

    void read(PMClient::Robot &robot)
    {
        auto &pm_pneumatic = robot.get_pneumatic(this->id);

        bool invert_physical_pos = this->id == PneumaticId::ProtectDoseur ||
                                   this->id == PneumaticId::Glue || this->id == PneumaticId::UV1;

        switch (pm_pneumatic.get_position())
        {
            case Position::Forward:
                this->fwd_or_bwd = 1.0;
                this->physical_position = invert_physical_pos ? upper_limit : lower_limit;
                break;
            case Position::Neutral:
                this->fwd_or_bwd = 0.0;
                this->physical_position = 0.0;
                break;
            case Position::Backward:
                this->fwd_or_bwd = -1.0;
                this->physical_position = invert_physical_pos ? lower_limit : upper_limit;
                break;
        }

        if (!initialized)
        {
            this->move_command = this->fwd_or_bwd;
            initialized = true;
        }
    }

    void write(PMClient::Robot &robot)
    {
        auto &pm_pneumatic = robot.get_pneumatic(this->id);

        if (!std::isnan(this->move_command))
        {
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

            this->move_command = double_limits::quiet_NaN();
        }
    }
};

} // namespace pm_hardware_interface
