#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

#include "pm_client/aerotech_axis.hpp"

namespace pm_hardware_interface
{

using StateInterface = hardware_interface::StateInterface;

using CommandInterface = hardware_interface::CommandInterface;

using AxisId = PMClient::AxisId;

static double increments_to_meters(PMClient::AerotechAxis &axis, int increments)
{
    return axis.increments_to_units(increments) / 1e6;
}

static int meters_to_increments(PMClient::AerotechAxis &axis, double units)
{
    return axis.units_to_increments(units * 1e6);
}

struct AxisState
{
    AxisId id;
    std::string name;

    double current_position = 0.0;
    double target_position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;

    explicit AxisState(AxisId my_id) : id(my_id)
    {
        switch (my_id)
        {
            case AxisId::X:
                name = "X_Axis_Joint";
                break;
            case AxisId::Y:
                name = "Y_Axis_Joint";
                break;
            case AxisId::Z:
                name = "Z_Axis_Joint";
                break;
            case AxisId::T:
                name = "T_Axis_Joint";
                break;
        }
    }

    void add_state_interfaces(std::vector<StateInterface> &interfaces)
    {
        interfaces.emplace_back(
            StateInterface(this->name, hardware_interface::HW_IF_POSITION, &this->current_position)
        );

        interfaces.emplace_back(
            StateInterface(this->name, hardware_interface::HW_IF_VELOCITY, &this->velocity)
        );
    }

    void add_command_interfaces(std::vector<CommandInterface> &interfaces)
    {
        interfaces.emplace_back(
            CommandInterface(this->name, hardware_interface::HW_IF_POSITION, &this->target_position)
        );

        interfaces.emplace_back(
            CommandInterface(this->name, hardware_interface::HW_IF_VELOCITY, &this->velocity)
        );

        interfaces.emplace_back(CommandInterface(
            this->name,
            hardware_interface::HW_IF_ACCELERATION,
            &this->acceleration
        ));
    }

    void read(PMClient::Robot &robot)
    {
        auto &pm_axis = robot.get_axis(this->id);
        this->current_position = increments_to_meters(pm_axis, pm_axis.get_position());
        this->velocity = increments_to_meters(pm_axis, pm_axis.get_speed());
        this->target_position = increments_to_meters(pm_axis, pm_axis.get_target());
        this->acceleration = increments_to_meters(pm_axis, pm_axis.get_acceleration());
    }

    void write(PMClient::Robot &robot)
    {
        auto &pm_axis = robot.get_axis(this->id);
        pm_axis.move(meters_to_increments(pm_axis, this->target_position));
        pm_axis.set_speed(meters_to_increments(pm_axis, this->velocity));
        // pm_axis->set_acceleration(meters_to_increments(*pm_axis, ros_axis.acceleration));
    }
};

} // namespace pm_hardware_interface
