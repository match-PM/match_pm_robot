#pragma once

#include <string>

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

/**
 * Auflistung der Roboterachsen.
 */
enum class AxisId
{
    X,
    Y,
    Z,
    T,
};

/**
 * Auflistung der möglichen Toleranzen einer Achse.
 */
enum class AxisTolerance
{
    Small,
    Medium,
    Big,
    Huge,
};

/**
 * Enthält verschiedene Daten und Einstellungen einer Achse, die über den Client
 * abgefragt wurde.
 */
class AerotechAxis
{
  private:
    Client *m_client;
    int m_status = 0;

  public:
    explicit AerotechAxis(Client *client, AxisId axis_id) : m_client(client), axis_id{axis_id}
    {
    }

    /**
     * Which axis this object belongs to.
     */
    AxisId axis_id;

    /**
     * NodeId for the speed of the axis.
     */
    UA_NodeId speed_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the max speed of the axis.
     */
    UA_NodeId max_speed_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the  acceleration of the axis.
     */
    UA_NodeId acceleration_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the max acceleration of the axis.
     */
    UA_NodeId max_acceleration_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the servo state of the axis.
     */
    UA_NodeId servo_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the axis tolerance.
     */
    UA_NodeId tolerance_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the "get_end_move" function.
     */
    UA_NodeId end_move_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the whether there is an error.
     */
    UA_NodeId has_error_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the error id.
     */
    UA_NodeId error_id_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the current position of the axis.
     */
    UA_NodeId actual_position_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the target position of the axis.
     */
    UA_NodeId target_position_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the minimum allowed position of the axis.
     */
    UA_NodeId min_position_node_id = UA_NODEID_NULL;

    /**
     * NodeId for the maximum allowed position of the axis.
     */
    UA_NodeId max_position_node_id = UA_NODEID_NULL;

    /**
     * NodeId for whether or not the axis is initialized.
     */
    UA_NodeId is_initialized_node_id = UA_NODEID_NULL;

    /**
     * Check if all node ids are properly set.
     */
    [[nodiscard]] bool is_ok() const;

    /**
     * Set the speed of the axis, measured in TODO.
     */
    void set_speed(long speed);

    /**
     * The current speed of the axis, measured in TODO.
     */
    [[nodiscard]] long get_speed() const;

    /**
     * Get the maximum speed of the axis, measured in TODO.
     */
    [[nodiscard]] long get_max_speed() const;

    /**
     * Set the acceleration of the axis, measured in TODO.
     */
    void set_acceleration(long acceleration);

    /**
     * The current acceleration of the axis, measured in TODO.
     */
    [[nodiscard]] long get_acceleration() const;

    /**
     * Get the maximum acceleration of the axis, measured in TODO.
     */
    [[nodiscard]] long get_max_acceleration() const;

    /**
     * Enable or disable the servo of the axis.
     */
    void set_servo(bool enable);

    /**
     * Whether the servo of the axis is enabled.
     */
    [[nodiscard]] bool get_servo() const;

    /**
     * Set the tolerance of the axis.
     */
    void set_tolerance(AxisTolerance tolerance);

    /**
     * The current tolerance of the axis.
     */
    [[nodiscard]] AxisTolerance get_tolerance() const;

    /**
     * Set a new target position of the axis, measured in increments.
     */
    void move(long target);

    /**
     * Brake the axis.
     */
    void brake();

    /**
     * Whether or not the axis has reached its target position.
     */
    [[nodiscard]] bool get_end_move() const;

    /**
     * Whether or not the axis has encountered an error.
     */
    [[nodiscard]] bool get_error() const;

    /**
     * If the axis has encountered an error, get the error id.
     */
    [[nodiscard]] long get_error_id() const;

    /**
     * If the axis has encountered an error, clear it.
     */
    void clear_error() const;

    /**
     * If the axis has encountered an error, get the error message.
     */
    std::string get_error_message() const;
    /**
     * The current, actual position of the axis, measured in increments.
     */
    [[nodiscard]] long get_position() const;

    /**
     * The target position of the axis, measured in increments.
     */
    [[nodiscard]] long get_target() const;

    /**
     * Set the minimum allowed position for this axis.
     */
    void set_min_position(long position);

    /**
     * Set the maximum allowed position for this axis.
     */
    void set_max_position(long position);

    /**
     * Get the minimum allowed position for this axis.
     */
    [[nodiscard]] long get_min_position() const;

    /**
     * Get the maximum allowed position for this axis.
     */
    [[nodiscard]] long get_max_position() const;

    /**
     * Whether the axis has been initialized.
     */
    [[nodiscard]] bool get_initialized() const;

    /**
     * Initialize the axis (non-blocking).
     */
    void initialize();
};

} // namespace PMClient
