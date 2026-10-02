

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "pm_client/client.hpp"
#include "pm_client/robot.hpp"

#include "open62541/open62541.h"

namespace PMClient
{

namespace
{

bool data_value_is_good(const UA_DataValue &value)
{
    return value.hasValue && (!value.hasStatus || value.status == UA_STATUSCODE_GOOD);
}

bool read_int32(const UA_DataValue &value, int &result)
{
    if (!data_value_is_good(value) ||
        !UA_Variant_hasScalarType(&value.value, &UA_TYPES[UA_TYPES_INT32]))
    {
        return false;
    }

    result = static_cast<int>(*static_cast<const UA_Int32 *>(value.value.data));
    return true;
}

bool read_boolean(const UA_DataValue &value, bool &result)
{
    if (!data_value_is_good(value) ||
        !UA_Variant_hasScalarType(&value.value, &UA_TYPES[UA_TYPES_BOOLEAN]))
    {
        return false;
    }

    result = *static_cast<const UA_Boolean *>(value.value.data) != UA_FALSE;
    return true;
}

bool read_double(const UA_DataValue &value, double &result)
{
    if (!data_value_is_good(value) ||
        !UA_Variant_hasScalarType(&value.value, &UA_TYPES[UA_TYPES_DOUBLE]))
    {
        return false;
    }

    result = static_cast<double>(*static_cast<const UA_Double *>(value.value.data));
    return true;
}

template<std::size_t Count>
bool read_boolean_array(const UA_DataValue &value, std::array<bool, Count> &result)
{
    if (!data_value_is_good(value) ||
        !UA_Variant_hasArrayType(&value.value, &UA_TYPES[UA_TYPES_BOOLEAN]) ||
        value.value.arrayLength != Count)
    {
        return false;
    }

    const auto *data = static_cast<const UA_Boolean *>(value.value.data);
    for (std::size_t i = 0; i < Count; ++i)
    {
        result[i] = data[i] != UA_FALSE;
    }
    return true;
}

template<std::size_t Count>
bool read_int32_array(const UA_DataValue &value, std::array<int, Count> &result)
{
    if (!data_value_is_good(value) ||
        !UA_Variant_hasArrayType(&value.value, &UA_TYPES[UA_TYPES_INT32]) ||
        value.value.arrayLength != Count)
    {
        return false;
    }

    const auto *data = static_cast<const UA_Int32 *>(value.value.data);
    for (std::size_t i = 0; i < Count; ++i)
    {
        result[i] = static_cast<int>(data[i]);
    }
    return true;
}

template<std::size_t Count>
bool read_double_array(const UA_DataValue &value, std::array<double, Count> &result)
{
    if (!data_value_is_good(value) ||
        !UA_Variant_hasArrayType(&value.value, &UA_TYPES[UA_TYPES_DOUBLE]) ||
        value.value.arrayLength != Count)
    {
        return false;
    }

    const auto *data = static_cast<const UA_Double *>(value.value.data);
    for (std::size_t i = 0; i < Count; ++i)
    {
        result[i] = static_cast<double>(data[i]);
    }
    return true;
}

template<typename Parser>
bool read_nodes(
    UA_Client *client, const std::vector<const UA_NodeId *> &node_ids, Parser parser
)
{
    // Build a single service request instead of paying one network round trip
    // for every node in an auxiliary snapshot.
    UA_ReadRequest request;
    UA_ReadRequest_init(&request);
    request.timestampsToReturn = UA_TIMESTAMPSTORETURN_NEITHER;
    request.nodesToReadSize = node_ids.size();
    request.nodesToRead = static_cast<UA_ReadValueId *>(
        UA_Array_new(node_ids.size(), &UA_TYPES[UA_TYPES_READVALUEID])
    );
    if (request.nodesToRead == nullptr)
    {
        return false;
    }

    bool request_valid = true;
    for (std::size_t i = 0; i < node_ids.size(); ++i)
    {
        request.nodesToRead[i].attributeId = UA_ATTRIBUTEID_VALUE;
        if (UA_NodeId_copy(node_ids[i], &request.nodesToRead[i].nodeId) != UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }
    }

    if (!request_valid)
    {
        UA_ReadRequest_clear(&request);
        return false;
    }

    UA_ReadResponse response = UA_Client_Service_read(client, request);
    bool success = response.responseHeader.serviceResult == UA_STATUSCODE_GOOD &&
                   response.resultsSize == node_ids.size();
    if (success)
    {
        success = parser(response.results);
    }

    UA_ReadResponse_clear(&response);
    UA_ReadRequest_clear(&request);
    return success;
}

struct WriteNode
{
    const UA_NodeId *node_id;
    const void *data;
    const UA_DataType *type;
    std::size_t array_length;
};

bool write_nodes(UA_Client *client, const std::vector<WriteNode> &nodes)
{
    // Scalar and array writes share one OPC UA service request.
    if (nodes.empty())
    {
        return true;
    }

    UA_WriteRequest request;
    UA_WriteRequest_init(&request);
    request.nodesToWriteSize = nodes.size();
    request.nodesToWrite = static_cast<UA_WriteValue *>(
        UA_Array_new(nodes.size(), &UA_TYPES[UA_TYPES_WRITEVALUE])
    );
    if (request.nodesToWrite == nullptr)
    {
        return false;
    }

    bool request_valid = true;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        UA_WriteValue &write_value = request.nodesToWrite[i];
        write_value.attributeId = UA_ATTRIBUTEID_VALUE;
        write_value.value.hasValue = true;
        if (UA_NodeId_copy(nodes[i].node_id, &write_value.nodeId) != UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }

        UA_StatusCode status = UA_STATUSCODE_BAD;
        if (nodes[i].array_length == 0)
        {
            status = UA_Variant_setScalarCopy(
                &write_value.value.value, nodes[i].data, nodes[i].type
            );
        }
        else
        {
            status = UA_Variant_setArrayCopy(
                &write_value.value.value,
                nodes[i].data,
                nodes[i].array_length,
                nodes[i].type
            );
        }
        if (status != UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }
    }

    if (!request_valid)
    {
        UA_WriteRequest_clear(&request);
        return false;
    }

    UA_WriteResponse response = UA_Client_Service_write(client, request);
    bool success = response.responseHeader.serviceResult == UA_STATUSCODE_GOOD &&
                   response.resultsSize == nodes.size();
    for (std::size_t i = 0; success && i < response.resultsSize; ++i)
    {
        success = response.results[i] == UA_STATUSCODE_GOOD;
    }

    UA_WriteResponse_clear(&response);
    UA_WriteRequest_clear(&request);
    return success;
}

} // namespace

bool AuxiliaryCommands::empty() const
{
    for (const bool changed : pneumatic_changed)
    {
        if (changed)
        {
            return false;
        }
    }
    for (const bool changed : nozzle_changed)
    {
        if (changed)
        {
            return false;
        }
    }

    return !camera1_coax_light_changed && !camera1_ring_light_changed &&
           !camera1_ring_light_rgb_changed && !camera2_light_changed && !uv_on_off_changed &&
           !uv_power_changed && !uv_time_changed && !force_sensor_bias;
}

Client::Client() : m_client{UA_Client_new()}, m_robot{std::make_unique<Robot>()}
{
    UA_ClientConfig *config = UA_Client_getConfig(m_client);
    UA_ClientConfig_setDefault(config);
    config->timeout = 60'000; // ms
}

Client::~Client()
{
    disconnect();
    UA_Client_delete(m_client);
}

void Client::connect(std::string endpoint)
{
    auto status = UA_Client_connect(m_client, endpoint.c_str());
    if (status != UA_STATUSCODE_GOOD)
    {
        throw std::runtime_error{UA_StatusCode_name(status)};
    }
}

void Client::disconnect()
{
    UA_Client_disconnect(m_client);
}

void Client::set_timeout(UA_UInt32 timeout_ms)
{
    UA_Client_getConfig(m_client)->timeout = timeout_ms;
}

bool Client::read_axis_motion_states(std::array<AxisMotionState, 8> &states)
{
    // ros2_control needs exactly these two values in its time-critical loop.
    // Slow lighting, UV, pneumatic and sensor nodes use the auxiliary client.
    std::array<AerotechAxis *, 8> axes{
        m_robot->x_axis.get(),
        m_robot->y_axis.get(),
        m_robot->z_axis.get(),
        m_robot->t_axis.get(),
        m_robot->q_axis.get(),
        m_robot->r_axis.get(),
        m_robot->u_axis.get(),
        m_robot->v_axis.get(),
    };

    constexpr std::size_t node_count = 16;
    UA_ReadRequest request;
    UA_ReadRequest_init(&request);
    request.timestampsToReturn = UA_TIMESTAMPSTORETURN_NEITHER;
    request.nodesToReadSize = node_count;
    request.nodesToRead = static_cast<UA_ReadValueId *>(
        UA_Array_new(node_count, &UA_TYPES[UA_TYPES_READVALUEID])
    );
    if (request.nodesToRead == nullptr)
    {
        return false;
    }

    bool request_valid = true;
    for (std::size_t i = 0; i < axes.size(); ++i)
    {
        UA_ReadValueId &position = request.nodesToRead[2 * i];
        UA_ReadValueId &speed = request.nodesToRead[2 * i + 1];
        position.attributeId = UA_ATTRIBUTEID_VALUE;
        speed.attributeId = UA_ATTRIBUTEID_VALUE;
        if (UA_NodeId_copy(&axes[i]->actual_position_node_id, &position.nodeId) !=
            UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }
        if (UA_NodeId_copy(&axes[i]->speed_node_id, &speed.nodeId) != UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }
    }

    if (!request_valid)
    {
        UA_ReadRequest_clear(&request);
        return false;
    }

    UA_ReadResponse response = UA_Client_Service_read(m_client, request);
    bool success = response.responseHeader.serviceResult == UA_STATUSCODE_GOOD &&
                   response.resultsSize == node_count;
    std::array<AxisMotionState, 8> new_states{};
    for (std::size_t i = 0; success && i < new_states.size(); ++i)
    {
        const UA_DataValue &position = response.results[2 * i];
        const UA_DataValue &speed = response.results[2 * i + 1];
        success = position.hasValue && speed.hasValue &&
                  (!position.hasStatus || position.status == UA_STATUSCODE_GOOD) &&
                  (!speed.hasStatus || speed.status == UA_STATUSCODE_GOOD) &&
                  UA_Variant_hasScalarType(&position.value, &UA_TYPES[UA_TYPES_INT32]) &&
                  UA_Variant_hasScalarType(&speed.value, &UA_TYPES[UA_TYPES_INT32]);
        if (success)
        {
            new_states[i].position = *static_cast<UA_Int32 *>(position.value.data);
            new_states[i].speed = *static_cast<UA_Int32 *>(speed.value.data);
        }
    }

    UA_ReadResponse_clear(&response);
    UA_ReadRequest_clear(&request);
    if (success)
    {
        states = new_states;
    }
    return success;
}

bool Client::write_axis_motion_commands(const std::array<AxisMotionCommand, 8> &commands)
{
    // The request is bounded to changed values, so an idle control cycle sends
    // no write and cannot build a backlog of duplicate commands.
    std::array<AerotechAxis *, 8> axes{
        m_robot->x_axis.get(),
        m_robot->y_axis.get(),
        m_robot->z_axis.get(),
        m_robot->t_axis.get(),
        m_robot->q_axis.get(),
        m_robot->r_axis.get(),
        m_robot->u_axis.get(),
        m_robot->v_axis.get(),
    };

    std::size_t node_count = 0;
    for (const auto &command : commands)
    {
        node_count += command.has_target ? 1 : 0;
        node_count += command.has_speed ? 1 : 0;
    }
    if (node_count == 0)
    {
        return true;
    }

    UA_WriteRequest request;
    UA_WriteRequest_init(&request);
    request.nodesToWriteSize = node_count;
    request.nodesToWrite = static_cast<UA_WriteValue *>(
        UA_Array_new(node_count, &UA_TYPES[UA_TYPES_WRITEVALUE])
    );
    if (request.nodesToWrite == nullptr)
    {
        return false;
    }

    bool request_valid = true;
    std::size_t node_index = 0;
    const auto add_value = [&](const UA_NodeId &node_id, int value) {
        UA_WriteValue &write_value = request.nodesToWrite[node_index++];
        write_value.attributeId = UA_ATTRIBUTEID_VALUE;
        write_value.value.hasValue = true;
        if (UA_NodeId_copy(&node_id, &write_value.nodeId) != UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }
        const UA_Int32 ua_value = static_cast<UA_Int32>(value);
        if (UA_Variant_setScalarCopy(
                &write_value.value.value, &ua_value, &UA_TYPES[UA_TYPES_INT32]
            ) != UA_STATUSCODE_GOOD)
        {
            request_valid = false;
        }
    };

    // Apply all speed changes before target changes. A target write can start
    // motion immediately on the server.
    for (std::size_t i = 0; i < commands.size(); ++i)
    {
        if (commands[i].has_speed)
        {
            add_value(axes[i]->speed_node_id, commands[i].speed);
        }
    }
    for (std::size_t i = 0; i < commands.size(); ++i)
    {
        if (commands[i].has_target)
        {
            add_value(axes[i]->target_position_node_id, commands[i].target);
        }
    }

    if (!request_valid)
    {
        UA_WriteRequest_clear(&request);
        return false;
    }

    UA_WriteResponse response = UA_Client_Service_write(m_client, request);
    bool success = response.responseHeader.serviceResult == UA_STATUSCODE_GOOD &&
                   response.resultsSize == node_count;
    for (std::size_t i = 0; success && i < response.resultsSize; ++i)
    {
        success = response.results[i] == UA_STATUSCODE_GOOD;
    }

    UA_WriteResponse_clear(&response);
    UA_WriteRequest_clear(&request);
    return success;
}

bool Client::read_auxiliary_io_state(AuxiliaryIoState &state)
{
    std::array<PneumaticCylinder *, 6> pneumatics{
        m_robot->uv1_pneumatic.get(),
        m_robot->uv2_pneumatic.get(),
        m_robot->glue_pneumatic.get(),
        m_robot->glue_2k_pneumatic.get(),
        m_robot->camera_mire_pneumatic.get(),
        m_robot->protect_doseur_pneumatic.get(),
    };
    std::array<Nozzle *, 7> nozzles{
        m_robot->head_nozzle.get(),
        m_robot->gonio_nozzle.get(),
        m_robot->nest_nozzle.get(),
        m_robot->doseur_glue.get(),
        m_robot->doseur_glue_2k.get(),
        m_robot->tool_changer.get(),
        m_robot->tool_changer_air_pressure.get(),
    };

    std::vector<const UA_NodeId *> nodes;
    nodes.reserve(pneumatics.size() + nozzles.size() + 1);
    for (const auto *pneumatic : pneumatics)
    {
        nodes.push_back(&pneumatic->position_node_id);
    }
    for (const auto *nozzle : nozzles)
    {
        nodes.push_back(&nozzle->state_node_id);
    }
    nodes.push_back(&m_robot->reference_cube->pushed);

    AuxiliaryIoState new_state{};
    const bool success = read_nodes(m_client, nodes, [&](const UA_DataValue *values) {
        std::size_t value_index = 0;
        for (std::size_t i = 0; i < pneumatics.size(); ++i)
        {
            int raw_position = 0;
            if (!read_int32(values[value_index++], raw_position) || raw_position < -1 ||
                raw_position > 1)
            {
                return false;
            }
            new_state.pneumatic_positions[i] = static_cast<Position>(raw_position);
        }
        for (std::size_t i = 0; i < nozzles.size(); ++i)
        {
            int raw_state = 0;
            if (!read_int32(values[value_index++], raw_state) || raw_state < -1 || raw_state > 1)
            {
                return false;
            }
            new_state.nozzle_states[i] = static_cast<NozzleState>(raw_state);
        }
        return read_boolean(values[value_index], new_state.reference_cube_pushed);
    });

    if (success)
    {
        state = new_state;
    }
    return success;
}

bool Client::read_auxiliary_sensor_state(AuxiliarySensorState &state)
{
    const std::vector<const UA_NodeId *> nodes{
        &m_robot->laser->measurement,
        &m_robot->force_sensor->measurements,
    };

    AuxiliarySensorState new_state{};
    const bool success = read_nodes(m_client, nodes, [&](const UA_DataValue *values) {
        return read_double(values[0], new_state.laser_measurement) &&
               read_double_array(values[1], new_state.force_measurements);
    });

    if (success)
    {
        state = new_state;
    }
    return success;
}

bool Client::read_auxiliary_lighting_state(AuxiliaryLightingState &state)
{
    const std::vector<const UA_NodeId *> nodes{
        &m_robot->camera1->coax_light,
        &m_robot->camera1->ring_light,
        &m_robot->camera1->ring_light_rgb,
        &m_robot->camera2->light,
        &m_robot->hoenle_uv->on_off,
        &m_robot->hoenle_uv->power,
        &m_robot->hoenle_uv->time,
    };

    AuxiliaryLightingState new_state{};
    const bool success = read_nodes(m_client, nodes, [&](const UA_DataValue *values) {
        return read_boolean(values[0], new_state.camera1_coax_light) &&
               read_boolean_array(values[1], new_state.camera1_ring_light) &&
               read_int32_array(values[2], new_state.camera1_ring_light_rgb) &&
               read_int32(values[3], new_state.camera2_light) &&
               read_boolean_array(values[4], new_state.uv_on_off) &&
               read_int32_array(values[5], new_state.uv_power) &&
               read_double_array(values[6], new_state.uv_time);
    });

    if (success)
    {
        state = new_state;
    }
    return success;
}

bool Client::write_auxiliary_commands(const AuxiliaryCommands &commands)
{
    std::array<PneumaticCylinder *, 6> pneumatics{
        m_robot->uv1_pneumatic.get(),
        m_robot->uv2_pneumatic.get(),
        m_robot->glue_pneumatic.get(),
        m_robot->glue_2k_pneumatic.get(),
        m_robot->camera_mire_pneumatic.get(),
        m_robot->protect_doseur_pneumatic.get(),
    };
    std::array<Nozzle *, 7> nozzles{
        m_robot->head_nozzle.get(),
        m_robot->gonio_nozzle.get(),
        m_robot->nest_nozzle.get(),
        m_robot->doseur_glue.get(),
        m_robot->doseur_glue_2k.get(),
        m_robot->tool_changer.get(),
        m_robot->tool_changer_air_pressure.get(),
    };

    std::array<UA_Int32, 6> pneumatic_values{};
    std::array<UA_Int32, 7> nozzle_values{};
    UA_Boolean camera1_coax_light = static_cast<UA_Boolean>(commands.camera1_coax_light);
    std::array<UA_Boolean, 4> camera1_ring_light{};
    std::array<UA_Int32, 3> camera1_ring_light_rgb{};
    UA_Int32 camera2_light = static_cast<UA_Int32>(commands.camera2_light);
    std::array<UA_Boolean, 4> uv_on_off{};
    std::array<UA_Int32, 4> uv_power{};
    std::array<UA_Double, 4> uv_time{};
    const UA_Boolean force_sensor_bias = UA_TRUE;

    std::vector<WriteNode> nodes;
    nodes.reserve(24);
    for (std::size_t i = 0; i < pneumatics.size(); ++i)
    {
        if (commands.pneumatic_changed[i])
        {
            pneumatic_values[i] = static_cast<UA_Int32>(commands.pneumatic_positions[i]);
            nodes.push_back(WriteNode{
                &pneumatics[i]->move_cmd_node_id,
                &pneumatic_values[i],
                &UA_TYPES[UA_TYPES_INT32],
                0,
            });
        }
    }
    for (std::size_t i = 0; i < nozzles.size(); ++i)
    {
        if (commands.nozzle_changed[i])
        {
            nozzle_values[i] = static_cast<UA_Int32>(commands.nozzle_states[i]);
            nodes.push_back(WriteNode{
                &nozzles[i]->state_node_id,
                &nozzle_values[i],
                &UA_TYPES[UA_TYPES_INT32],
                0,
            });
        }
    }
    if (commands.force_sensor_bias)
    {
        nodes.push_back(WriteNode{
            &m_robot->force_sensor->bias,
            &force_sensor_bias,
            &UA_TYPES[UA_TYPES_BOOLEAN],
            0,
        });
    }
    if (commands.camera1_coax_light_changed)
    {
        nodes.push_back(WriteNode{
            &m_robot->camera1->coax_light,
            &camera1_coax_light,
            &UA_TYPES[UA_TYPES_BOOLEAN],
            0,
        });
    }
    if (commands.camera1_ring_light_changed)
    {
        for (std::size_t i = 0; i < camera1_ring_light.size(); ++i)
        {
            camera1_ring_light[i] = static_cast<UA_Boolean>(commands.camera1_ring_light[i]);
        }
        nodes.push_back(WriteNode{
            &m_robot->camera1->ring_light,
            camera1_ring_light.data(),
            &UA_TYPES[UA_TYPES_BOOLEAN],
            camera1_ring_light.size(),
        });
    }
    if (commands.camera1_ring_light_rgb_changed)
    {
        for (std::size_t i = 0; i < camera1_ring_light_rgb.size(); ++i)
        {
            camera1_ring_light_rgb[i] =
                static_cast<UA_Int32>(commands.camera1_ring_light_rgb[i]);
        }
        nodes.push_back(WriteNode{
            &m_robot->camera1->ring_light_rgb,
            camera1_ring_light_rgb.data(),
            &UA_TYPES[UA_TYPES_INT32],
            camera1_ring_light_rgb.size(),
        });
    }
    if (commands.camera2_light_changed)
    {
        nodes.push_back(WriteNode{
            &m_robot->camera2->light,
            &camera2_light,
            &UA_TYPES[UA_TYPES_INT32],
            0,
        });
    }
    if (commands.uv_on_off_changed)
    {
        for (std::size_t i = 0; i < uv_on_off.size(); ++i)
        {
            uv_on_off[i] = static_cast<UA_Boolean>(commands.uv_on_off[i]);
        }
        nodes.push_back(WriteNode{
            &m_robot->hoenle_uv->on_off,
            uv_on_off.data(),
            &UA_TYPES[UA_TYPES_BOOLEAN],
            uv_on_off.size(),
        });
    }
    if (commands.uv_power_changed)
    {
        for (std::size_t i = 0; i < uv_power.size(); ++i)
        {
            uv_power[i] = static_cast<UA_Int32>(commands.uv_power[i]);
        }
        nodes.push_back(WriteNode{
            &m_robot->hoenle_uv->power,
            uv_power.data(),
            &UA_TYPES[UA_TYPES_INT32],
            uv_power.size(),
        });
    }
    if (commands.uv_time_changed)
    {
        for (std::size_t i = 0; i < uv_time.size(); ++i)
        {
            uv_time[i] = static_cast<UA_Double>(commands.uv_time[i]);
        }
        nodes.push_back(WriteNode{
            &m_robot->hoenle_uv->time,
            uv_time.data(),
            &UA_TYPES[UA_TYPES_DOUBLE],
            uv_time.size(),
        });
    }

    return write_nodes(m_client, nodes);
}

void Client::init()
{
    auto browse = [this](UA_NodeId start_id, auto f) {
        UA_BrowseRequest browse_request;
        UA_BrowseRequest_init(&browse_request);
        browse_request.requestedMaxReferencesPerNode = 0;
        browse_request.nodesToBrowse = UA_BrowseDescription_new();
        browse_request.nodesToBrowseSize = 1;
        browse_request.nodesToBrowse[0].nodeId = start_id;
        browse_request.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL;
        auto response = UA_Client_Service_browse(this->m_client, browse_request);

        auto return_value = f(response);

        UA_BrowseResponse_clear(&response);
        UA_BrowseRequest_clear(&browse_request);

        return return_value;
    };

    // TODO: refactor this for less code duplication

    auto browse_axis = [&](UA_NodeId axis_node_id,
                           AxisId axis_id) -> std::unique_ptr<AerotechAxis> {
        return browse(axis_node_id, [&](auto response) {
            auto axis = std::make_unique<AerotechAxis>(this, axis_id);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Speed", &axis->speed_node_id);
                    get_node_id_from_ref("MaxSpeed", &axis->max_speed_node_id);
                    get_node_id_from_ref("Acceleration", &axis->acceleration_node_id);
                    get_node_id_from_ref("MaxAcceleration", &axis->max_acceleration_node_id);
                    get_node_id_from_ref("Servo", &axis->servo_node_id);
                    get_node_id_from_ref("Tolerance", &axis->tolerance_node_id);
                    get_node_id_from_ref("EndMove", &axis->end_move_node_id);
                    get_node_id_from_ref("HasError", &axis->has_error_node_id);
                    get_node_id_from_ref("ErrorId", &axis->error_id_node_id);
                    get_node_id_from_ref("ActualPosition", &axis->actual_position_node_id);
                    get_node_id_from_ref("TargetPosition", &axis->target_position_node_id);
                    get_node_id_from_ref("MinPosition", &axis->min_position_node_id);
                    get_node_id_from_ref("MaxPosition", &axis->max_position_node_id);
                    get_node_id_from_ref("IsInitialized", &axis->is_initialized_node_id);
                    get_node_id_from_ref("UnitsPerIncrement", &axis->units_per_increment_node_id);
                }
            }

            return axis;
        });
    };

    auto browse_pneumatic = [&](UA_NodeId pneumatic_node_id,
                                PneumaticId pneumatic_id) -> std::unique_ptr<PneumaticCylinder> {
        return browse(pneumatic_node_id, [&](auto response) {
            auto pneumatic = std::make_unique<PneumaticCylinder>(this, pneumatic_id);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Position", &pneumatic->position_node_id);
                    get_node_id_from_ref("MoveCommand", &pneumatic->move_cmd_node_id);
                }
            }

            return pneumatic;
        });
    };

    auto browse_nozzle = [&](UA_NodeId nozzle_node_id,
                             NozzleId nozzle_id) -> std::unique_ptr<Nozzle> {
        return browse(nozzle_node_id, [&](auto response) {
            auto nozzle = std::make_unique<Nozzle>(this, nozzle_id);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("State", &nozzle->state_node_id);
                }
            }

            return nozzle;
        });
    };

    auto browse_camera1 = [&](UA_NodeId camera_node) -> std::unique_ptr<Camera1> {
        return browse(camera_node, [&](auto response) {
            auto camera1 = std::make_unique<Camera1>(this);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("CoaxLight", &camera1->coax_light);
                    get_node_id_from_ref("RingLight", &camera1->ring_light);
                    get_node_id_from_ref("RingLightRGB", &camera1->ring_light_rgb);
                }
            }

            return camera1;
        });
    };

    auto browse_camera2 = [&](UA_NodeId camera_node) -> std::unique_ptr<Camera2> {
        return browse(camera_node, [&](auto response) {
            auto camera2 = std::make_unique<Camera2>(this);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Light", &camera2->light);
                }
            }

            return camera2;
        });
    };

    auto browse_laser = [&](UA_NodeId laser_node) -> std::unique_ptr<Laser> {
        return browse(laser_node, [&](auto response) {
            auto laser = std::make_unique<Laser>(this);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Measurement", &laser->measurement);
                }
            }

            return laser;
        });
    };

    auto browse_force_sensor = [&](UA_NodeId force_sensor_node) -> std::unique_ptr<ForceSensor> {
        return browse(force_sensor_node, [&](auto response) {
            auto force_sensor = std::make_unique<ForceSensor>(this);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Measurements", &force_sensor->measurements);
                    get_node_id_from_ref("SetZero", &force_sensor->bias);
                }
            }

            return force_sensor;
        });
    };

    auto browse_hoenle_uv = [&](UA_NodeId hoenle_uv_node) -> std::unique_ptr<HoenleUV> {
        return browse(hoenle_uv_node, [&](auto response) {
            auto hoenle_uv = std::make_unique<HoenleUV>(this);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("OnOff", &hoenle_uv->on_off);
                    get_node_id_from_ref("Power", &hoenle_uv->power);
                    get_node_id_from_ref("Time", &hoenle_uv->time);
                }
            }

            return hoenle_uv;
        });
    };

    auto browse_reference_cube =
        [&](UA_NodeId reference_cube_node) -> std::unique_ptr<ReferenceCube> {
        return browse(reference_cube_node, [&](auto response) {
            auto reference_cube = std::make_unique<ReferenceCube>(this);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Pushed", &reference_cube->pushed);
                }
            }

            return reference_cube;
        });
    };

    auto browse_skills = [&](UA_NodeId skills_node) -> std::unique_ptr<Skills> {
        return browse(skills_node, [&](auto response) {
            auto skills = std::make_unique<Skills>(this, skills_node);

            for (size_t i = 0; i < response.resultsSize; ++i)
            {
                for (size_t j = 0; j < response.results[i].referencesSize; ++j)
                {
                    UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                    std::string_view browse_name(
                        reinterpret_cast<char *>(ref->browseName.name.data),
                        ref->browseName.name.length
                    );

                    auto get_node_id_from_ref = [&](auto bname, auto *dest) {
                        if (browse_name == bname)
                        {
                            UA_NodeId_copy(&ref->nodeId.nodeId, dest);
                        }
                    };

                    get_node_id_from_ref("Dispense", &skills->dispense_method);
                    get_node_id_from_ref("ForceSensingMove", &skills->force_sensing_move_method);
                }
            }

            return skills;
        });
    };

    browse(UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER), [&](auto response) {
        for (size_t i = 0; i < response.resultsSize; ++i)
        {
            for (size_t j = 0; j < response.results[i].referencesSize; ++j)
            {
                UA_ReferenceDescription *ref = &(response.results[i].references[j]);

                std::string_view browse_name(
                    reinterpret_cast<char *>(ref->browseName.name.data),
                    ref->browseName.name.length
                );

                if (browse_name == "RobotAxisX")
                {
                    m_robot->x_axis = browse_axis(ref->nodeId.nodeId, AxisId::X);
                }
                else if (browse_name == "RobotAxisY")
                {
                    m_robot->y_axis = browse_axis(ref->nodeId.nodeId, AxisId::Y);
                }
                else if (browse_name == "RobotAxisZ")
                {
                    m_robot->z_axis = browse_axis(ref->nodeId.nodeId, AxisId::Z);
                }
                else if (browse_name == "RobotAxisT")
                {
                    m_robot->t_axis = browse_axis(ref->nodeId.nodeId, AxisId::T);
                }
                else if (browse_name == "RobotAxisQ")
                {
                    m_robot->q_axis = browse_axis(ref->nodeId.nodeId, AxisId::Q);
                }
                else if (browse_name == "RobotAxisR")
                {
                    m_robot->r_axis = browse_axis(ref->nodeId.nodeId, AxisId::R);
                }
                else if (browse_name == "RobotAxisU")
                {
                    m_robot->u_axis = browse_axis(ref->nodeId.nodeId, AxisId::U);
                }
                else if (browse_name == "RobotAxisV")
                {
                    m_robot->v_axis = browse_axis(ref->nodeId.nodeId, AxisId::V);
                }
                else if (browse_name == "PneumaticModuleUV1")
                {
                    m_robot->uv1_pneumatic = browse_pneumatic(ref->nodeId.nodeId, PneumaticId::UV1);
                }
                else if (browse_name == "PneumaticModuleUV2")
                {
                    m_robot->uv2_pneumatic = browse_pneumatic(ref->nodeId.nodeId, PneumaticId::UV2);
                }
                else if (browse_name == "PneumaticModuleGlue")
                {
                    m_robot->glue_pneumatic =
                        browse_pneumatic(ref->nodeId.nodeId, PneumaticId::Glue);
                }
                else if (browse_name == "PneumaticModuleGlue2K")
                {
                    m_robot->glue_2k_pneumatic =
                        browse_pneumatic(ref->nodeId.nodeId, PneumaticId::Glue2K);
                }
                else if (browse_name == "PneumaticModuleCameraMire")
                {
                    m_robot->camera_mire_pneumatic =
                        browse_pneumatic(ref->nodeId.nodeId, PneumaticId::CameraMire);
                }
                else if (browse_name == "PneumaticProtectDoseur")
                {
                    m_robot->protect_doseur_pneumatic =
                        browse_pneumatic(ref->nodeId.nodeId, PneumaticId::ProtectDoseur);
                }
                else if (browse_name == "HeadNozzle")
                {
                    m_robot->head_nozzle = browse_nozzle(ref->nodeId.nodeId, NozzleId::Head);
                }
                else if (browse_name == "GoniometerNozzle")
                {
                    m_robot->gonio_nozzle = browse_nozzle(ref->nodeId.nodeId, NozzleId::Gonio);
                }
                else if (browse_name == "NestNozzle")
                {
                    m_robot->nest_nozzle = browse_nozzle(ref->nodeId.nodeId, NozzleId::Nest);
                }
                else if (browse_name == "DoseurGlue")
                {
                    m_robot->doseur_glue = browse_nozzle(ref->nodeId.nodeId, NozzleId::DoseurGlue);
                }
                else if (browse_name == "DoseurGlue2K")
                {
                    m_robot->doseur_glue_2k =
                        browse_nozzle(ref->nodeId.nodeId, NozzleId::DoseurGlue2K);
                }
                else if (browse_name == "ToolChanger")
                {
                    m_robot->tool_changer =
                        browse_nozzle(ref->nodeId.nodeId, NozzleId::ToolChanger);
                }
                else if (browse_name == "ToolChangerAirPressure")
                {
                    m_robot->tool_changer_air_pressure =
                        browse_nozzle(ref->nodeId.nodeId, NozzleId::ToolChangerAirPressure);
                }
                else if (browse_name == "Camera1")
                {
                    m_robot->camera1 = browse_camera1(ref->nodeId.nodeId);
                }
                else if (browse_name == "Camera2")
                {
                    m_robot->camera2 = browse_camera2(ref->nodeId.nodeId);
                }
                else if (browse_name == "Laser")
                {
                    m_robot->laser = browse_laser(ref->nodeId.nodeId);
                }
                else if (browse_name == "ForceSensor")
                {
                    m_robot->force_sensor = browse_force_sensor(ref->nodeId.nodeId);
                }
                else if (browse_name == "HoenleUV")
                {
                    m_robot->hoenle_uv = browse_hoenle_uv(ref->nodeId.nodeId);
                }
                else if (browse_name == "ReferenceCube")
                {
                    m_robot->reference_cube = browse_reference_cube(ref->nodeId.nodeId);
                }
                else if (browse_name == "Skills")
                {
                    m_robot->skills = browse_skills(ref->nodeId.nodeId);
                }
            }
        }

        return 0;
    });

    if (!m_robot->is_ok())
    {
        throw std::runtime_error{"Robot descriptor is not complete. Server is missing nodes."};
    }
}

// Luis alte function!!!

// void Client::call_method(
//     UA_NodeId object_id, UA_NodeId method_id, std::size_t input_size, UA_Variant *inputs,
//     std::size_t *output_size, UA_Variant **outputs
// )
// {
//     UA_StatusCode status =
//         UA_Client_call(m_client, object_id, method_id, input_size, inputs, output_size, outputs);

//     if (status != UA_STATUSCODE_GOOD)
//     {
//         throw std::runtime_error{UA_StatusCode_name(status)};
//     }
// }

// // ChatGPT revised function with retry mechanism
void Client::call_method(
    UA_NodeId object_id, UA_NodeId method_id, std::size_t input_size, UA_Variant *inputs,
    std::size_t *output_size, UA_Variant **outputs, std::string endpoint
)
{
    const int MAX_RETRIES = 6;       // number of retry attempts
    const int RETRY_DELAY_MS = 1000; // delay between retries in milliseconds

    int attempt = 0;
    UA_StatusCode status;

    while (attempt < MAX_RETRIES)
    {
        status = UA_Client_call(
            m_client,
            object_id,
            method_id,
            input_size,
            inputs,
            output_size,
            outputs
        );
        // m_config.opcua_endpoint
        if (status == UA_STATUSCODE_GOOD)
        {
            // Call succeeded
            return;
        }

        // Check if the error is connection/session related
        if (status == UA_STATUSCODE_BADCONNECTIONCLOSED || status == UA_STATUSCODE_BADSESSIONCLOSED)
        {
            std::cerr << "Connection lost (attempt " << (attempt + 1) << "/" << MAX_RETRIES
                      << "). Reconnecting...\n";

            // Disconnect and try to reconnect
            UA_Client_disconnect(m_client);

            // You may want to store the server endpoint somewhere in your Client class
            UA_StatusCode conn_status = UA_Client_connect(m_client, endpoint.c_str());
            if (conn_status != UA_STATUSCODE_GOOD)
            {
                std::cerr << "Reconnect failed: " << UA_StatusCode_name(conn_status) << "\n";
                // Increment attempt and retry after delay
                attempt++;
                UA_sleep_ms(RETRY_DELAY_MS);
                continue;
            }

            // Successfully reconnected, retry the call
            attempt++;
            continue;
        }
        else
        {
            // Other errors, do not retry
            throw std::runtime_error{UA_StatusCode_name(status)};
        }
    }

    // If we reach here, all retries failed
    throw std::runtime_error{
        "UA_Client_call failed after retries: " + std::string(UA_StatusCode_name(status))
    };
}

} // namespace PMClient
