#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
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

enum class Unit
{
    Meters,
    Degrees,
};

struct AxisState
{
    AxisId id;
    std::string name;
    Unit unit;

    double current_position = 0.0;
    double target_position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    // Conversion factors are invariant for a running machine and are loaded
    // once instead of being requested from OPC UA every control cycle.
    double units_per_increment = std::numeric_limits<double>::quiet_NaN();

    // Last successfully written increments support duplicate suppression.
    bool has_last_target_increment = false;
    bool has_last_speed_increment = false;
    int last_target_increment = 0;
    int last_speed_increment = 0;

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
                break;
            case AxisId::Q:
                name = "Gonio_Right_Stage_2_Joint";
                break;
            case AxisId::V:
                name = "Gonio_Left_Stage_1_Joint";
                break;
            case AxisId::U:
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

    double get_units_per_increment(PMClient::AerotechAxis &pm_axis)
    {
        if (std::isnan(this->units_per_increment))
        {
            const double conversion_factor = pm_axis.increments_to_units(1);
            if (!std::isfinite(conversion_factor) || conversion_factor == 0.0)
            {
                throw std::runtime_error(
                    "Invalid units-per-increment value for axis " + this->name
                );
            }
            this->units_per_increment = conversion_factor;
        }

        return this->units_per_increment;
    }

    double increments_to_ros_units(PMClient::AerotechAxis &pm_axis, int increments)
    {
        const double axis_units = increments * this->get_units_per_increment(pm_axis);

        if (unit == Unit::Meters)
            return axis_units / 1e6;

        return axis_units / 180 * PI;
    }

    int ros_units_to_increments(PMClient::AerotechAxis &pm_axis, double ros_units)
    {
        const double axis_units = unit == Unit::Meters ? ros_units * 1e6 : ros_units * 180 / PI;
        return static_cast<int>(axis_units / this->get_units_per_increment(pm_axis));
    }

    int command_target_to_increments(PMClient::AerotechAxis &pm_axis)
    {
        auto target = this->target_position;
        if (this->id == AxisId::Q || this->id == AxisId::R)
            target = -target;

        return this->ros_units_to_increments(pm_axis, target);
    }

    int command_speed_to_increments(PMClient::AerotechAxis &pm_axis)
    {
        return this->ros_units_to_increments(pm_axis, this->velocity);
    }

    void initialize(PMClient::Robot &robot)
    {
        auto &pm_axis = robot.get_axis(this->id);
        const int position_increment = pm_axis.get_position();
        const int speed_increment = pm_axis.get_speed();
        const int target_increment = pm_axis.get_target();

        this->current_position = this->increments_to_ros_units(pm_axis, position_increment);
        this->velocity = this->increments_to_ros_units(pm_axis, speed_increment);
        this->target_position = this->increments_to_ros_units(pm_axis, target_increment);
        this->acceleration = this->increments_to_ros_units(pm_axis, pm_axis.get_acceleration());

        if (this->id == AxisId::Q || this->id == AxisId::R)
        {
            this->current_position = -this->current_position;
            this->target_position = -this->target_position;
        }

        this->last_target_increment = target_increment;
        this->last_speed_increment = speed_increment;
        this->has_last_target_increment = true;
        this->has_last_speed_increment = true;
    }

    void update(PMClient::AerotechAxis &pm_axis, const PMClient::AxisMotionState &state)
    {
        this->current_position = this->increments_to_ros_units(pm_axis, state.position);
        this->velocity = this->increments_to_ros_units(pm_axis, state.speed);

        if (this->id == AxisId::Q || this->id == AxisId::R)
        {
            this->current_position = -this->current_position;
        }

        // Velocity is both a state and an exported command in the current interface.
        // Keep the command cache synchronized unless a controller changes it after read().
        this->last_speed_increment = state.speed;
        this->has_last_speed_increment = true;
    }

    PMClient::AxisMotionCommand prepare_command(PMClient::AerotechAxis &pm_axis)
    {
        // Comparing in integer controller units avoids floating-point noise
        // creating writes for an otherwise unchanged trajectory sample.
        PMClient::AxisMotionCommand command{};
        command.target = this->command_target_to_increments(pm_axis);
        command.speed = this->command_speed_to_increments(pm_axis);
        command.has_target =
            !this->has_last_target_increment || command.target != this->last_target_increment;
        command.has_speed =
            !this->has_last_speed_increment || command.speed != this->last_speed_increment;
        return command;
    }

    void mark_command_written(const PMClient::AxisMotionCommand &command)
    {
        if (command.has_target)
        {
            this->last_target_increment = command.target;
            this->has_last_target_increment = true;
        }
        if (command.has_speed)
        {
            this->last_speed_increment = command.speed;
            this->has_last_speed_increment = true;
        }
    }

    void read(PMClient::Robot &robot)
    {
        auto &pm_axis = robot.get_axis(this->id);
        const PMClient::AxisMotionState state{pm_axis.get_position(), pm_axis.get_speed()};
        this->update(pm_axis, state);
    }

    void write(PMClient::Robot &robot)
    {
        try
        {
            auto &pm_axis = robot.get_axis(this->id);
            const auto command = this->prepare_command(pm_axis);
            if (command.has_speed)
            {
                pm_axis.set_speed(command.speed);
            }
            if (command.has_target)
            {
                pm_axis.move(command.target);
            }
            this->mark_command_written(command);
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("PMSystem"),
                "Failed to write position: %f m, or velocity %f m/s\n",
                this->target_position,
                this->velocity
            );
        }
    }
};

} // namespace pm_hardware_interface
