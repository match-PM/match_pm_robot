#include <cassert>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>

#include <iostream>

#include "open62541.h"

#include "defs.hpp"

RobotDescriptor g_robot_descriptor;

UA_Boolean running = true;
static void stop_handler(int sig)
{
    (void)sig;
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Received ^C");
    running = false;
}

static void RegisterRobotAxisTypeVariable(
    UA_Server *server, const char *display_name, const char *description, std::size_t ua_data_type,
    bool read_only
)
{
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName =
        UA_LOCALIZEDTEXT(const_cast<char *>("en-US"), const_cast<char *>(display_name));
    attr.description =
        UA_LOCALIZEDTEXT(const_cast<char *>("en-US"), const_cast<char *>(description));
    attr.dataType = UA_TYPES[ua_data_type].typeId;
    if (!read_only)
    {
        std::cout << display_name << " allow writes !!!!!!\n";
        attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    }
    else
    {

        attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    }

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

#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node)                \
    RegisterRobotAxisTypeVariable(server, display_name, description, ua_data_type, read_only);

    VAR_TABLE
#undef VAR_ENTRY
}

UA_StatusCode RobotAxisTypeHandleReadVariable(
    UA_DataValue *data_value, AxisId axis_id, AxisVariableId variable_id, std::size_t ua_data_type
)
{
    (void)axis_id;
    (void)variable_id;

    // double value = 123.456;
    // UA_Variant_setScalarCopy(&data_value->value, nullptr, &UA_TYPES[ua_data_type]);
    // data_value->hasValue = true;
    // data_value->hasStatus = true;
    // data_value->status = UA_STATUSCODE_GOOD;

    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode RobotAxisTypeReadVariable(
    UA_Server *server, const UA_NodeId *session_id, void *session_context, const UA_NodeId *node_id,
    void *node_context, UA_Boolean source_time_stamp, const UA_NumericRange *range,
    UA_DataValue *data_value
)
{
    (void)server;
    (void)session_id;
    (void)session_context;
    (void)node_context;
    (void)source_time_stamp;
    (void)range;

    static std::vector<std::pair<const AxisDescriptor &, AxisId>> axes = {
#define AXIS_ENTRY(display_name, id, desc) {g_robot_descriptor.desc, id},
        AXIS_TABLE
#undef AXIS_ENTRY
    };

    static std::vector<std::pair<const UA_NodeId AxisDescriptor::*, AxisVariableId>> vars = {
#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node)                \
    {&AxisDescriptor::node, var_id},
        VAR_TABLE
#undef VAR_ENTRY
    };

    static std::vector<std::size_t> var_ua_data_types = {
#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node) ua_data_type,
        VAR_TABLE
#undef VAR_ENTRY
    };

    for (std::size_t axis_idx = 0; axis_idx < axes.size(); axis_idx += 1)
    {
        for (std::size_t var_idx = 0; var_idx < vars.size(); var_idx += 1)
        {
            const AxisDescriptor &axis_desc = axes[axis_idx].first;
            const AxisId axis_id = axes[axis_idx].second;

            const UA_NodeId AxisDescriptor::*node_id_ptr = vars[var_idx].first;
            const UA_NodeId *this_node_id = &(axis_desc.*node_id_ptr);

            const AxisVariableId var_id = vars[var_idx].second;

            const std::size_t ua_data_type = var_ua_data_types[var_idx];

            if (UA_NodeId_equal(node_id, this_node_id))
            {
                return RobotAxisTypeHandleReadVariable(data_value, axis_id, var_id, ua_data_type);
            }
        }
    }

    return UA_STATUSCODE_BADNOTFOUND;
}

UA_StatusCode RobotAxisTypeHandleWriteVariable(
    const UA_DataValue *data_value, AxisId axis_id, AxisVariableId variable_id
)
{
    (void)axis_id;
    (void)variable_id;

    std::cout << "WRITING\n";

    if (!UA_Variant_hasScalarType(&data_value->value, &UA_TYPES[UA_TYPES_DOUBLE]))
    {
        return UA_STATUSCODE_BAD;
    }

    double value = *(double *)data_value->value.data;
    std::cout << "WROTE VALUE = " << value << '\n';

    return UA_STATUSCODE_GOOD;
}

static UA_StatusCode RobotAxisTypeWriteVariable(
    UA_Server *server, const UA_NodeId *session_id, void *session_context, const UA_NodeId *node_id,
    void *node_context, const UA_NumericRange *range, const UA_DataValue *data_value
)
{
    (void)server;
    (void)session_id;
    (void)session_context;
    (void)node_context;
    (void)range;

    static std::vector<std::pair<const AxisDescriptor &, AxisId>> axes = {
#define AXIS_ENTRY(display_name, id, desc) {g_robot_descriptor.desc, id},
        AXIS_TABLE
#undef AXIS_ENTRY
    };

    static std::vector<std::pair<const UA_NodeId AxisDescriptor::*, AxisVariableId>> vars = {
#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node)                \
    {&AxisDescriptor::node, var_id},
        VAR_TABLE
#undef VAR_ENTRY
    };

    for (std::size_t axis_idx = 0; axis_idx < axes.size(); axis_idx += 1)
    {
        for (std::size_t var_idx = 0; var_idx < vars.size(); var_idx += 1)
        {
            const AxisDescriptor &axis_desc = axes[axis_idx].first;
            const AxisId axis_id = axes[axis_idx].second;

            const UA_NodeId AxisDescriptor::*node_id_ptr = vars[var_idx].first;
            const UA_NodeId *this_node_id = &(axis_desc.*node_id_ptr);

            const AxisVariableId var_id = vars[var_idx].second;

            if (UA_NodeId_equal(node_id, this_node_id))
            {
                return RobotAxisTypeHandleWriteVariable(data_value, axis_id, var_id);
            }
        }
    }

    return UA_STATUSCODE_BADNOTFOUND;
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
#define AXIS_ENTRY(display_name, id, desc)                                                         \
    case id:                                                                                       \
        axis_desc = &g_robot_descriptor.desc;                                                      \
        break;

        AXIS_TABLE
#undef AXIS_ENTRY

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

    UA_StatusCode status;

#define VAR_ENTRY(display_name, description, ua_data_type, read_only, var_id, node)                \
    status = RegisterReadWriteFuncForRobotAxisVariable(                                            \
        server,                                                                                    \
        &axis_desc->parent_id,                                                                     \
        &axis_desc->node,                                                                          \
        display_name                                                                               \
    );                                                                                             \
    if (status != UA_STATUSCODE_GOOD)                                                              \
    {                                                                                              \
        return status;                                                                             \
    }

    VAR_TABLE
#undef ENTRY

    return UA_STATUSCODE_GOOD;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);

    UA_Server *server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    RegisterRobotAxisType(server);

#define AXIS_ENTRY(display_name, id, desc) RegisterRobotAxisObject(server, id, display_name);
    AXIS_TABLE
#undef AXIS_ENTRY

    UA_StatusCode status = UA_Server_run(server, &running);

    UA_Server_delete(server);

    return status == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}
