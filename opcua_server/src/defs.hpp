#ifndef OPCUA_SERVER_DEFS_HPP
#define OPCUA_SERVER_DEFS_HPP

#include <array>
#include <unordered_map>
#include <vector>

#include "open62541.h"

#include "tables.hpp"

enum AxisId
{
#define AXIS_ENTRY(display_name, id, desc) id,
    AXIS_TABLE
#undef AXIS_ENTRY
};

enum AxisVariableId
{
#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node) var_id,
    VAR_TABLE
#undef VAR_ENTRY
};

struct AxisDescriptor
{
    UA_NodeId parent_id;

#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node) UA_NodeId node;
    VAR_TABLE
#undef VAR_ENTRY

    AxisDescriptor()
    {
        parent_id = UA_NODEID_NULL;

#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node)                \
    node = UA_NODEID_NULL;
        VAR_TABLE
#undef VAR_ENTRY
    }
};

struct RobotDescriptor
{
    UA_NodeId robot_axis_type_id;

    AxisDescriptor x_axis;
    AxisDescriptor y_axis;
    AxisDescriptor z_axis;
    AxisDescriptor t_axis;
};

#endif // OPCUA_SERVER_DEFS_HPP
