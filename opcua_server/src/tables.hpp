#ifndef OPCUA_SERVER_TABLES_HPP
#define OPCUA_SERVER_TABLES_HPP

/**
 * This file uses X Macros (https://en.wikipedia.org/wiki/X_Macro) to define tables that
 * contain all relevant information for each axis and each axis variable.
 * This way all information is located in one place instead of being spread across the
 * entire code base.
 * Editing the entries in these tables will automatically generate the correct code in
 * the other files.
 */

/**
 * Format: AXIS_ENTRY(display_name, axis_id, axis_descriptor)
 */
#define AXIS_TABLE                                                                                 \
    AXIS_ENTRY("RobotAxisX", AXIS_X, x_axis)                                                       \
    AXIS_ENTRY("RobotAxisY", AXIS_Y, y_axis)                                                       \
    AXIS_ENTRY("RobotAxisZ", AXIS_Z, z_axis)                                                       \
    AXIS_ENTRY("RobotAxisT", AXIS_T, t_axis)

/**
 * Format: VAR_ENTRY(display_name, description, ua_data_type, read_only, variable_id, node)
 */
#define VAR_TABLE                                                                                  \
    VAR_ENTRY("Speed", "The current speed.", UA_TYPES_INT32, false, AXIS_VAR_SPEED, speed)         \
    VAR_ENTRY(                                                                                     \
        "MaxSpeed",                                                                                \
        "The maximum allowed speed..",                                                             \
        UA_TYPES_INT32,                                                                            \
        true,                                                                                      \
        AXIS_VAR_MAX_SPEED,                                                                        \
        max_speed                                                                                  \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "Acceleration",                                                                            \
        "The current acceleration..",                                                              \
        UA_TYPES_INT32,                                                                            \
        false,                                                                                     \
        AXIS_VAR_ACCELERATION,                                                                     \
        acceleration                                                                               \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "MaxAcceleration",                                                                         \
        "The maximum allowed acceleration.",                                                       \
        UA_TYPES_INT32,                                                                            \
        true,                                                                                      \
        AXIS_VAR_MAX_ACCELERATION,                                                                 \
        max_acceleration                                                                           \
    )                                                                                              \
    VAR_ENTRY("Servo", "State of the servo.", UA_TYPES_BOOLEAN, true, AXIS_VAR_SERVO, servo)       \
    VAR_ENTRY(                                                                                     \
        "Tolerance",                                                                               \
        "The current axis movement tolerance.",                                                    \
        UA_TYPES_INT32,                                                                            \
        false,                                                                                     \
        AXIS_VAR_TOLERANCE,                                                                        \
        tolerance                                                                                  \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "EndMove",                                                                                 \
        "Whether the axis has reached its target and has stopped moving.",                         \
        UA_TYPES_BOOLEAN,                                                                          \
        true,                                                                                      \
        AXIS_VAR_END_MOVE,                                                                         \
        end_move                                                                                   \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "HasError",                                                                                \
        "Whether the axis has encountered an error.",                                              \
        UA_TYPES_BOOLEAN,                                                                          \
        true,                                                                                      \
        AXIS_VAR_HAS_ERROR,                                                                        \
        has_error                                                                                  \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "ErrorId",                                                                                 \
        "If there is an error, this gives the error id.",                                          \
        UA_TYPES_INT32,                                                                            \
        true,                                                                                      \
        AXIS_VAR_ERROR_ID,                                                                         \
        error_id                                                                                   \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "ActualPosition",                                                                          \
        "The current position.",                                                                   \
        UA_TYPES_INT32,                                                                            \
        true,                                                                                      \
        AXIS_VAR_ACTUAL_POSITION,                                                                  \
        actual_position                                                                            \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "TargetPosition",                                                                          \
        "The target position.",                                                                    \
        UA_TYPES_INT32,                                                                            \
        false,                                                                                     \
        AXIS_VAR_TARGET_POSITION,                                                                  \
        target_position                                                                            \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "MinPosition",                                                                             \
        "The minimum allowed position.",                                                           \
        UA_TYPES_INT32,                                                                            \
        true,                                                                                      \
        AXIS_VAR_MIN_POSITION,                                                                     \
        min_position                                                                               \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "MaxPosition",                                                                             \
        "The maximum allowed position.",                                                           \
        UA_TYPES_INT32,                                                                            \
        true,                                                                                      \
        AXIS_VAR_MAX_POSITION,                                                                     \
        max_position                                                                               \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "IsInitialized",                                                                           \
        "Whether the axis is initialized.",                                                        \
        UA_TYPES_BOOLEAN,                                                                          \
        true,                                                                                      \
        AXIS_VAR_IS_INITIALIZED,                                                                   \
        is_initialized                                                                             \
    )                                                                                              \
    VAR_ENTRY(                                                                                     \
        "UnitsPerIncrement",                                                                       \
        "The number of units per increment.",                                                      \
        UA_TYPES_DOUBLE,                                                                           \
        true,                                                                                      \
        AXIS_VAR_UNITS_PER_INCREMENT,                                                              \
        units_per_increment                                                                        \
    )

#endif // OPCUA_SERVER_TABLES_HPP
