#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

#include "pm_client/aerotech_axis.hpp"

#define PI 3.14159265359

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

static double increments_to_rad(PMClient::AerotechAxis &axis, int increments)
{
    return axis.increments_to_units(increments) / 180 * PI;
}

static int rad_to_increments(PMClient::AerotechAxis &axis, double rad)
{
    return axis.units_to_increments(rad * 180 / PI);
}

enum class Unit
{
    Meters,
    Degrees,
};

static std::string unitToString(Unit unit)
{
    switch (unit)
    {
        case Unit::Meters:
            return "Meters";
        case Unit::Degrees:
            return "Degrees";
        default:
            return "Unknown";
    }
}

struct AxisState
{
    AxisId id;
    std::string name;
    Unit unit;

    double current_position = 0.0;
    double target_position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;

    explicit AxisState(AxisId my_id, Unit my_unit) : id(my_id), unit(my_unit)
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
            case AxisId::R:
                name = "Gonio_Right_Stage_1_Joint";
                RCLCPP_ERROR(
                    rclcpp::get_logger("PMSystem"),
                    "Unit: %s",
                    unitToString(my_unit).c_str()
                );
                break;
            case AxisId::Q:
                name = "Gonio_Right_Stage_2_Joint";
                break;
            case AxisId::U:
                name = "Gonio_Left_Stage_1_Joint";
                break;
            case AxisId::V:
                name = "Gonio_Left_Stage_2_Joint";
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
        if (unit == Unit::Meters)
        {
            this->current_position = increments_to_meters(pm_axis, pm_axis.get_position());
            this->velocity = increments_to_meters(pm_axis, pm_axis.get_speed());
            this->target_position = increments_to_meters(pm_axis, pm_axis.get_target());
            this->acceleration = increments_to_meters(pm_axis, pm_axis.get_acceleration());
        }
        else if (unit == Unit::Degrees)
        {
            this->current_position = increments_to_rad(pm_axis, pm_axis.get_position());
            this->velocity = increments_to_rad(pm_axis, pm_axis.get_speed());
            this->target_position = increments_to_rad(pm_axis, pm_axis.get_target());
            this->acceleration = increments_to_rad(pm_axis, pm_axis.get_acceleration());
        }

        if (this->id == AxisId::Q || this->id == AxisId::R)
            this->current_position = -this->current_position;
    }

    void write(PMClient::Robot &robot)
    {
        auto target = this->target_position;
        if (this->id == AxisId::Q || this->id == AxisId::R)
            target = -target;

        try
        {
            auto &pm_axis = robot.get_axis(this->id);
            if (unit == Unit::Meters)
            {
                pm_axis.move(meters_to_increments(pm_axis, target));
                pm_axis.set_speed(meters_to_increments(pm_axis, this->velocity));
                // pm_axis.set_acceleration(meters_to_increments(pm_axis, this->acceleration));
            }
            else if (unit == Unit::Degrees)
            {
                pm_axis.move(rad_to_increments(pm_axis, target));
                pm_axis.set_speed(rad_to_increments(pm_axis, this->velocity));
                // pm_axis.set_acceleration(rad_to_increments(pm_axis, this->acceleration));
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("PMSystem"),
                "Failed to write position: %f m, or velocity %f m/s\n",
                target,
                this->velocity
            );
        }
    }
};

} // namespace pm_hardware_interface
