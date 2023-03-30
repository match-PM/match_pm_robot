#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>

#include <iostream>

#include "open62541.h"

typedef UA_StatusCode (*VariableReadFunc
)(UA_Server *server, const UA_NodeId *session_id, void *session_context, const UA_NodeId *node_id,
  void *node_context, UA_Boolean source_time_stamp, const UA_NumericRange *range,
  UA_DataValue *data_value);

typedef UA_StatusCode (*VariableWriteFunc
)(UA_Server *server, const UA_NodeId *session_id, void *session_context, const UA_NodeId *node_id,
  void *node_context, const UA_NumericRange *range, const UA_DataValue *data_value);

enum AxisId
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_T,
};

struct AxisDescriptor
{
    UA_NodeId parent_id = UA_NODEID_NULL;
    UA_NodeId speed_node_id = UA_NODEID_NULL;
    UA_NodeId max_speed_node_id = UA_NODEID_NULL;
    UA_NodeId acceleration_node_id = UA_NODEID_NULL;
    UA_NodeId max_acceleration_node_id = UA_NODEID_NULL;
    UA_NodeId servo_node_id = UA_NODEID_NULL;
    UA_NodeId tolerance_node_id = UA_NODEID_NULL;
    UA_NodeId end_move_node_id = UA_NODEID_NULL;
    UA_NodeId has_error_node_id = UA_NODEID_NULL;
    UA_NodeId error_id_node_id = UA_NODEID_NULL;
    UA_NodeId actual_position_node_id = UA_NODEID_NULL;
    UA_NodeId target_position_node_id = UA_NODEID_NULL;
    UA_NodeId min_position_node_id = UA_NODEID_NULL;
    UA_NodeId max_position_node_id = UA_NODEID_NULL;
    UA_NodeId is_initialized_node_id = UA_NODEID_NULL;
};

struct RobotDescriptor
{
    UA_NodeId robot_axis_type_id;

    AxisDescriptor x_axis;
    AxisDescriptor y_axis;
    AxisDescriptor z_axis;
    AxisDescriptor t_axis;
} g_robot_descriptor;

UA_Boolean running = true;
static void stop_handler(int sig)
{
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Received ^C");
    running = false;
}

static void
RegisterRobotAxisTypeVariable(UA_Server *server, const char *display_name, const char *description)
{
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName =
        UA_LOCALIZEDTEXT(const_cast<char *>("en-US"), const_cast<char *>(display_name));
    attr.description =
        UA_LOCALIZEDTEXT(const_cast<char *>("en-US"), const_cast<char *>(description));

    UA_NodeId node_id;
    UA_Server_addVariableNode(
        server,
        UA_NODEID_NULL,
        g_robot_descriptor.robot_axis_type_id,
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(1, const_cast<char *>(display_name)),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        attr,
        NULL,
        &node_id
    );

    UA_Server_addReference(
        server,
        node_id,
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASMODELLINGRULE),
        UA_EXPANDEDNODEID_NUMERIC(0, UA_NS0ID_MODELLINGRULE_MANDATORY),
        true
    );
}

static void RegisterRobotAxisType(UA_Server *server)
{
    UA_ObjectTypeAttributes robot_axis_attr = UA_ObjectTypeAttributes_default;
    robot_axis_attr.displayName =
        UA_LOCALIZEDTEXT(const_cast<char *>("en-US"), const_cast<char *>("RobotAxisType"));

    UA_Server_addObjectTypeNode(
        server,
        UA_NODEID_NULL,
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASSUBTYPE),
        UA_QUALIFIEDNAME(1, const_cast<char *>("RobotAxisType")),
        robot_axis_attr,
        NULL,
        &g_robot_descriptor.robot_axis_type_id
    );

    RegisterRobotAxisTypeVariable(server, "Speed", "The current speed of this axis.");
    RegisterRobotAxisTypeVariable(server, "MaxSpeed", "The maximum speed of this axis.");
    RegisterRobotAxisTypeVariable(server, "Acceleration", "The current acceleration of this axis.");
    RegisterRobotAxisTypeVariable(
        server,
        "MaxAcceleration",
        "The maximum allowed acceleration of this axis."
    );
    RegisterRobotAxisTypeVariable(server, "Servo", "Whether or not the axis servo is enabled.");
    RegisterRobotAxisTypeVariable(server, "Tolerance", "The current axis tolerance.");
    RegisterRobotAxisTypeVariable(
        server,
        "EndMove",
        "Whether or not the axis has reached its target position."
    );
    RegisterRobotAxisTypeVariable(
        server,
        "HasError",
        "Whether or not the axis has encountered an error."
    );
    RegisterRobotAxisTypeVariable(server, "ErrorId", "If there is an error, gives the error id.");
    RegisterRobotAxisTypeVariable(server, "ActualPosition", "The current position of the axis.");
    RegisterRobotAxisTypeVariable(
        server,
        "TargetPosition",
        "The current target position of the axis."
    );
    RegisterRobotAxisTypeVariable(
        server,
        "MinPosition",
        "The minimum allowed position for this axis."
    );
    RegisterRobotAxisTypeVariable(
        server,
        "MaxPosition",
        "The maximum allowed position for this axis."
    );
    RegisterRobotAxisTypeVariable(
        server,
        "IsInitialized",
        "Whether or not this axis has been initialized."
    );
}

template<typename ValueType>
void RobotAxisTypeReadVariableHelper(
    UA_DataValue *data_value_out, ValueType value, std::size_t UA_TYPE
)
{
    UA_Variant_setScalarCopy(&data_value_out->value, &value, &UA_TYPES[UA_TYPE]);
    data_value_out->hasValue = true;
    data_value->hasStatus = true;
    data_value->status = UA_STATUSCODE_GOOD;
}

static UA_StatusCode RobotAxisTypeReadVariable(
    UA_Server *server, const UA_NodeId *session_id, void *session_context, const UA_NodeId *node_id,
    void *node_context, UA_Boolean source_time_stamp, const UA_NumericRange *range,
    UA_DataValue *data_value
)
{

#define READ(elsif, axis, node, ty, uaty, value)                                                   \
    elsif(UA_NodeId_equal(node_id, &g_robot_descriptor.axis.node##_node_id))                       \
    {                                                                                              \
        RobotAxisTypeReadVariableHelper<ty>(data_value, value, uaty);                              \
        return UA_STATUSCODE_GOOD;                                                                 \
    }

    READ(if, x_axis, speed, long, UA_TYPES_INT32, 0)
    READ(else if, y_axis, speed, long, UA_TYPES_INT32, 0)
    READ(else if, z_axis, speed, long, UA_TYPES_INT32, 0)
    READ(else if, t_axis, speed, long, UA_TYPES_INT32, 0)

    READ(else if, x_axis, max_speed, long, UA_TYPES_INT32, 0)
    READ(else if, y_axis, max_speed, long, UA_TYPES_INT32, 0)
    READ(else if, z_axis, max_speed, long, UA_TYPES_INT32, 0)
    READ(else if, t_axis, max_speed, long, UA_TYPES_INT32, 0)

    READ(else if, x_axis, acceleration, long, UA_TYPES_INT32, 0)
    READ(else if, y_axis, acceleration, long, UA_TYPES_INT32, 0)
    READ(else if, z_axis, acceleration, long, UA_TYPES_INT32, 0)
    READ(else if, t_axis, acceleration, long, UA_TYPES_INT32, 0)

    READ(else if, x_axis, max_acceleration, long, UA_TYPES_INT32, 0)
    READ(else if, y_axis, max_acceleration, long, UA_TYPES_INT32, 0)
    READ(else if, z_axis, max_acceleration, long, UA_TYPES_INT32, 0)
    READ(else if, t_axis, max_acceleration, long, UA_TYPES_INT32, 0)

    READ(else if, x_axis, servo, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, y_axis, servo, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, z_axis, servo, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, t_axis, servo, bool, UA_TYPES_BOOLEAN, false)

    READ(else if, x_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)
    READ(else if, y_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)
    READ(else if, z_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)
    READ(else if, t_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)

    READ(else if, x_axis, end_move, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, y_axis, end_move, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, z_axis, end_move, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, t_axis, end_move, bool, UA_TYPES_BOOLEAN, false)

    READ(else if, x_axis, has_error, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, y_axis, has_error, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, z_axis, has_error, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, t_axis, has_error, bool, UA_TYPES_BOOLEAN, false)

    READ(else if, x_axis, error_id, long, UA_TYPES_INT32, false)
    READ(else if, y_axis, error_id, long, UA_TYPES_INT32, false)
    READ(else if, z_axis, error_id, long, UA_TYPES_INT32, false)
    READ(else if, t_axis, error_id, long, UA_TYPES_INT32, false)

    READ(else if, x_axis, target_position, long, UA_TYPES_INT32, false)
    READ(else if, y_axis, target_position, long, UA_TYPES_INT32, false)
    READ(else if, z_axis, target_position, long, UA_TYPES_INT32, false)
    READ(else if, t_axis, target_position, long, UA_TYPES_INT32, false)

    READ(else if, x_axis, min_position, long, UA_TYPES_INT32, false)
    READ(else if, y_axis, min_position, long, UA_TYPES_INT32, false)
    READ(else if, z_axis, min_position, long, UA_TYPES_INT32, false)
    READ(else if, t_axis, min_position, long, UA_TYPES_INT32, false)

    READ(else if, x_axis, max_position, long, UA_TYPES_INT32, false)
    READ(else if, y_axis, max_position, long, UA_TYPES_INT32, false)
    READ(else if, z_axis, max_position, long, UA_TYPES_INT32, false)
    READ(else if, t_axis, max_position, long, UA_TYPES_INT32, false)

    READ(else if, x_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, y_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, z_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)
    READ(else if, t_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)

    else
    {
        return UA_STATUSCODE_BADNOTFOUND;
    }

#undef READ
}

template<typename ValueType>
UA_StatusCode RobotAxisTypeWriteVariableHelper(
    const UA_DataValue *data_value, ValueType *value, std::size_t UA_TYPE
)
{
    if (!UA_Variant_hasScalarType(&data_value->value, &UA_TYPES[UA_TYPE]))
    {
        return UA_STATUSCODE_BAD;
    }
}

static UA_StatusCode RobotAxisTypeWriteVariable(
    UA_Server *server, const UA_NodeId *session_id, void *session_context, const UA_NodeId *node_id,
    void *node_context, const UA_NumericRange *range, const UA_DataValue *data_value
)
{

#define WRITE(elsif, axis, node, ty, uaty, func)                                                   \
    elsif(UA_NodeId_equal(node_id, &g_robot_descriptor.axis.node##_node_id))                       \
    {                                                                                              \
        ty value;                                                                                  \
        UA_StatusCode status = RobotAxisTypeWriteVariableHelper<ty>(data_value, &value, uaty);     \
        if (status != UA_STATUSCODE_GOOD)                                                          \
        {                                                                                          \
            return status;                                                                         \
        }                                                                                          \
        return UA_STATUSCODE_GOOD;                                                                 \
    }

    // func(value);

    WRITE(if, x_axis, speed, long, UA_TYPES_INT32, 0)
    WRITE(else if, y_axis, speed, long, UA_TYPES_INT32, 0)
    WRITE(else if, z_axis, speed, long, UA_TYPES_INT32, 0)
    WRITE(else if, t_axis, speed, long, UA_TYPES_INT32, 0)

    WRITE(else if, x_axis, max_speed, long, UA_TYPES_INT32, 0)
    WRITE(else if, y_axis, max_speed, long, UA_TYPES_INT32, 0)
    WRITE(else if, z_axis, max_speed, long, UA_TYPES_INT32, 0)
    WRITE(else if, t_axis, max_speed, long, UA_TYPES_INT32, 0)

    WRITE(else if, x_axis, acceleration, long, UA_TYPES_INT32, 0)
    WRITE(else if, y_axis, acceleration, long, UA_TYPES_INT32, 0)
    WRITE(else if, z_axis, acceleration, long, UA_TYPES_INT32, 0)
    WRITE(else if, t_axis, acceleration, long, UA_TYPES_INT32, 0)

    WRITE(else if, x_axis, max_acceleration, long, UA_TYPES_INT32, 0)
    WRITE(else if, y_axis, max_acceleration, long, UA_TYPES_INT32, 0)
    WRITE(else if, z_axis, max_acceleration, long, UA_TYPES_INT32, 0)
    WRITE(else if, t_axis, max_acceleration, long, UA_TYPES_INT32, 0)

    WRITE(else if, x_axis, servo, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, y_axis, servo, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, z_axis, servo, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, t_axis, servo, bool, UA_TYPES_BOOLEAN, false)

    WRITE(else if, x_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)
    WRITE(else if, y_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)
    WRITE(else if, z_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)
    WRITE(else if, t_axis, tolerance, unsigned char, UA_TYPES_BYTE, false)

    WRITE(else if, x_axis, end_move, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, y_axis, end_move, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, z_axis, end_move, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, t_axis, end_move, bool, UA_TYPES_BOOLEAN, false)

    WRITE(else if, x_axis, has_error, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, y_axis, has_error, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, z_axis, has_error, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, t_axis, has_error, bool, UA_TYPES_BOOLEAN, false)

    WRITE(else if, x_axis, error_id, long, UA_TYPES_INT32, false)
    WRITE(else if, y_axis, error_id, long, UA_TYPES_INT32, false)
    WRITE(else if, z_axis, error_id, long, UA_TYPES_INT32, false)
    WRITE(else if, t_axis, error_id, long, UA_TYPES_INT32, false)

    WRITE(else if, x_axis, target_position, long, UA_TYPES_INT32, false)
    WRITE(else if, y_axis, target_position, long, UA_TYPES_INT32, false)
    WRITE(else if, z_axis, target_position, long, UA_TYPES_INT32, false)
    WRITE(else if, t_axis, target_position, long, UA_TYPES_INT32, false)

    WRITE(else if, x_axis, min_position, long, UA_TYPES_INT32, false)
    WRITE(else if, y_axis, min_position, long, UA_TYPES_INT32, false)
    WRITE(else if, z_axis, min_position, long, UA_TYPES_INT32, false)
    WRITE(else if, t_axis, min_position, long, UA_TYPES_INT32, false)

    WRITE(else if, x_axis, max_position, long, UA_TYPES_INT32, false)
    WRITE(else if, y_axis, max_position, long, UA_TYPES_INT32, false)
    WRITE(else if, z_axis, max_position, long, UA_TYPES_INT32, false)
    WRITE(else if, t_axis, max_position, long, UA_TYPES_INT32, false)

    WRITE(else if, x_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, y_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, z_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)
    WRITE(else if, t_axis, is_initialized, bool, UA_TYPES_BOOLEAN, false)

    else
    {
        return UA_STATUSCODE_BADNOTFOUND;
    }

#undef WRITE
}

static UA_StatusCode RegisterReadWriteFuncForRobotAxisVariable(
    UA_Server *server, UA_NodeId *axis_id, UA_NodeId *target_variable_node_id,
    const char *target_name
)
{
    UA_RelativePathElement rpe;
    UA_RelativePathElement_init(&rpe);
    rpe.referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
    rpe.isInverse = false;
    rpe.includeSubtypes = false;
    rpe.targetName = UA_QUALIFIEDNAME(1, const_cast<char *>(target_name));

    UA_BrowsePath bp;
    UA_BrowsePath_init(&bp);
    bp.startingNode = *axis_id;
    bp.relativePath.elementsSize = 1;
    bp.relativePath.elements = &rpe;

    UA_BrowsePathResult bpr = UA_Server_translateBrowsePathToNodeIds(server, &bp);

    if (bpr.statusCode != UA_STATUSCODE_GOOD || bpr.targetsSize < 1)
        return bpr.statusCode;

    UA_NodeId target_variable_id = bpr.targets[0].targetId.nodeId;
    UA_DataSource target_variable_data_source;
    target_variable_data_source.read = RobotAxisTypeReadVariable;
    target_variable_data_source.write = RobotAxisTypeWriteVariable;
    UA_Server_setVariableNode_dataSource(server, target_variable_id, target_variable_data_source);

    UA_NodeId_copy(&target_variable_id, target_variable_node_id);

    UA_BrowsePathResult_clear(&bpr);

    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode
RegisterRobotAxisObject(UA_Server *server, AxisId axis_id, const char *display_name)
{
    UA_ObjectAttributes axis_object_attr = UA_ObjectAttributes_default;
    axis_object_attr.displayName =
        UA_LOCALIZEDTEXT(const_cast<char *>("en-US"), const_cast<char *>(display_name));

    AxisDescriptor *axis_desc;
    switch (axis_id)
    {
        case AXIS_X:
            axis_desc = &g_robot_descriptor.x_axis;
            break;
        case AXIS_Y:
            axis_desc = &g_robot_descriptor.y_axis;
            break;
        case AXIS_Z:
            axis_desc = &g_robot_descriptor.z_axis;
            break;
        case AXIS_T:
            axis_desc = &g_robot_descriptor.t_axis;
            break;
        default:
            throw std::runtime_error("RegisterRobotAxisObject: Invalid axis id");
    }

    UA_Server_addObjectNode(
        server,
        UA_NODEID_NULL,
        UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(1, const_cast<char *>(display_name)),
        g_robot_descriptor.robot_axis_type_id,
        axis_object_attr,
        NULL,
        &axis_desc->parent_id
    );

    UA_StatusCode status = RegisterReadWriteFuncForRobotAxisVariable(
        server,
        &axis_desc->parent_id,
        &axis_desc->actual_position_node_id,
        "ActualPosition"
    );
    if (status != UA_STATUSCODE_GOOD)
    {
        return status;
    }

    status = RegisterReadWriteFuncForRobotAxisVariable(
        server,
        &axis_desc->parent_id,
        &axis_desc->target_position_node_id,
        "TargetPosition"
    );
    if (status != UA_STATUSCODE_GOOD)
    {
        return status;
    }

    status = RegisterReadWriteFuncForRobotAxisVariable(
        server,
        &axis_desc->parent_id,
        &axis_desc->tolerance_node_id,
        "Tolerance"
    );
    if (status != UA_STATUSCODE_GOOD)
    {
        return status;
    }

    status = RegisterReadWriteFuncForRobotAxisVariable(
        server,
        &axis_desc->parent_id,
        &axis_desc->speed_node_id,
        "Speed"
    );
    if (status != UA_STATUSCODE_GOOD)
    {
        return status;
    }

    return UA_STATUSCODE_GOOD;
}

int main(int argc, char **argv)
{
    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);

    UA_Server *server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    RegisterRobotAxisType(server);
    RegisterRobotAxisObject(server, AXIS_X, "RobotAxisX");
    RegisterRobotAxisObject(server, AXIS_Y, "RobotAxisY");
    RegisterRobotAxisObject(server, AXIS_Z, "RobotAxisZ");
    RegisterRobotAxisObject(server, AXIS_T, "RobotAxisT");

    UA_StatusCode status = UA_Server_run(server, &running);

    UA_Server_delete(server);

    return status == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}
