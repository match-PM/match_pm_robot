#include "pm_client/aerotech_axis.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool AerotechAxis::is_ok() const
{
    auto node_ids = {
        &speed_node_id,
        &max_speed_node_id,
        &acceleration_node_id,
        &max_acceleration_node_id,
        &servo_node_id,
        &tolerance_node_id,
        &end_move_node_id,
        &has_error_node_id,
        &error_id_node_id,
        &actual_position_node_id,
        &target_position_node_id,
        &min_position_node_id,
        &max_position_node_id,
        &is_initialized_node_id,
        &units_per_increment_node_id,
    };

    for (const auto &node_id : node_ids)
    {
        if (UA_NodeId_isNull(node_id))
        {
            return false;
        }
    }

    return true;
}

void AerotechAxis::set_speed(int speed)
{
    m_client->write_node_value<int, UA_TYPES_INT32>(this->speed_node_id, speed);
}

[[nodiscard]] int AerotechAxis::get_speed() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->speed_node_id);
}

[[nodiscard]] int AerotechAxis::get_max_speed() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->max_speed_node_id);
}

void AerotechAxis::set_acceleration(int acceleration)
{
    m_client->write_node_value<int, UA_TYPES_INT32>(this->acceleration_node_id, acceleration);
}

[[nodiscard]] int AerotechAxis::get_acceleration() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->acceleration_node_id);
}

[[nodiscard]] int AerotechAxis::get_max_acceleration() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->max_acceleration_node_id);
}

void AerotechAxis::set_servo(bool enable)
{
    m_client->write_node_value<bool, UA_TYPES_BOOLEAN>(this->servo_node_id, enable);
}

[[nodiscard]] bool AerotechAxis::get_servo() const
{
    return m_client->read_node_value<bool, UA_TYPES_BOOLEAN>(this->servo_node_id);
}

void AerotechAxis::set_tolerance(AxisTolerance tolerance)
{
    std::uint8_t raw_tolerance;
    switch (tolerance)
    {
        case AxisTolerance::Small:
            raw_tolerance = 0;
            break;
        case AxisTolerance::Medium:
            raw_tolerance = 1;
            break;
        case AxisTolerance::Big:
            raw_tolerance = 2;
            break;
        case AxisTolerance::Huge:
            raw_tolerance = 3;
            break;
        default:
            throw std::runtime_error{"Unreachable."};
    }
    m_client->write_node_value<std::uint8_t, UA_TYPES_BYTE>(this->tolerance_node_id, raw_tolerance);
}

[[nodiscard]] AxisTolerance AerotechAxis::get_tolerance() const
{
    auto raw_tolerance =
        m_client->read_node_value<std::uint8_t, UA_TYPES_BYTE>(this->tolerance_node_id);
    switch (raw_tolerance)
    {
        case 0:
            return AxisTolerance::Small;
        case 1:
            return AxisTolerance::Medium;
        case 2:
            return AxisTolerance::Big;
        case 3:
            return AxisTolerance::Huge;
        default:
            throw std::runtime_error{"Invalid tolerance value received from server."};
    }
}

void AerotechAxis::move(int target)
{
    m_client->write_node_value<int, UA_TYPES_INT32>(this->target_position_node_id, target);
}

void AerotechAxis::brake()
{
    // TODO: Figure out how to send a brake command.
    //       Maybe use OPCUA methods?
    throw std::runtime_error{"AerotechAxis::brake is not implemented."};
}

[[nodiscard]] bool AerotechAxis::get_end_move() const
{
    return m_client->read_node_value<bool, UA_TYPES_BOOLEAN>(this->end_move_node_id);
}

[[nodiscard]] bool AerotechAxis::get_error() const
{
    return m_client->read_node_value<bool, UA_TYPES_BOOLEAN>(this->has_error_node_id);
}

[[nodiscard]] int AerotechAxis::get_error_id() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->error_id_node_id);
}

void AerotechAxis::clear_error() const
{
    // TODO: See AerotechAxis::brake.
    throw std::runtime_error{"AerotechAxis::clear_error is not implemented."};
}

std::string AerotechAxis::get_error_message() const
{
    // TODO: Either manually map error ids to a bunch of strings,
    //       or get the error message straight from the OPCUA server.
    throw std::runtime_error("AerotechAxis::get_error_message is not implemented.");
}

[[nodiscard]] int AerotechAxis::get_position() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->actual_position_node_id);
}

[[nodiscard]] int AerotechAxis::get_target() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->target_position_node_id);
}

void AerotechAxis::set_min_position(int position)
{
    m_client->write_node_value<int, UA_TYPES_INT32>(this->min_position_node_id, position);
}

void AerotechAxis::set_max_position(int position)
{
    m_client->write_node_value<int, UA_TYPES_INT32>(this->max_position_node_id, position);
}

[[nodiscard]] int AerotechAxis::get_min_position() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->min_position_node_id);
}

[[nodiscard]] int AerotechAxis::get_max_position() const
{
    return m_client->read_node_value<int, UA_TYPES_INT32>(this->max_position_node_id);
}

[[nodiscard]] bool AerotechAxis::get_initialized() const
{
    return m_client->read_node_value<bool, UA_TYPES_BOOLEAN>(this->is_initialized_node_id);
}

void AerotechAxis::initialize()
{
    // TODO: See AerotechAxis::brake.
    throw std::runtime_error{"AerotechAxis::initialize is not implemented."};
}

[[nodiscard]] double AerotechAxis::increments_to_units(int increments) const
{
    auto units_per_increment =
        m_client->read_node_value<double, UA_TYPES_DOUBLE>(this->units_per_increment_node_id);
    return increments * units_per_increment;
}

[[nodiscard]] int AerotechAxis::units_to_increments(double units) const
{
    auto units_per_increment =
        m_client->read_node_value<double, UA_TYPES_DOUBLE>(this->units_per_increment_node_id);
    return static_cast<int>(units / units_per_increment);
}

} // namespace PMClient
