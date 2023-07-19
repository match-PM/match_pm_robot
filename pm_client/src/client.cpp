

#include <memory>
#include <string>
#include <string_view>

#include "pm_client/client.hpp"
#include "pm_client/robot.hpp"

#include "open62541/open62541.h"

namespace PMClient
{

Client::Client() : m_client{UA_Client_new()}, m_robot{std::make_unique<Robot>()}
{
    UA_ClientConfig_setDefault(UA_Client_getConfig(m_client));
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

                    get_node_id_from_ref("IsForward", &pneumatic->is_forward_node_id);
                    get_node_id_from_ref("IsBackward", &pneumatic->is_backward_node_id);
                    get_node_id_from_ref("MoveForwardCmd", &pneumatic->move_forward_cmd_node_id);
                    get_node_id_from_ref("MoveBackwardCmd", &pneumatic->move_backward_cmd_node_id);
                }
            }

            return pneumatic;
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
                    m_robot->t_axis = browse_axis(ref->nodeId.nodeId, AxisId::X);
                }
                else if (browse_name == "RobotAxisR")
                {
                    m_robot->t_axis = browse_axis(ref->nodeId.nodeId, AxisId::X);
                }
                else if (browse_name == "RobotAxisU")
                {
                    m_robot->t_axis = browse_axis(ref->nodeId.nodeId, AxisId::X);
                }
                else if (browse_name == "RobotAxisV")
                {
                    m_robot->t_axis = browse_axis(ref->nodeId.nodeId, AxisId::X);
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
                        browse_pneumatic(ref->nodeId.nodeId, PneumaticId::Glue);
                }
                else if (browse_name == "PneumaticModuleCameraMire")
                {
                    m_robot->camera_mire_pneumatic =
                        browse_pneumatic(ref->nodeId.nodeId, PneumaticId::CameraMire);
                }
                else if (browse_name == "Camera1")
                {
                    m_robot->camera1 = browse_camera1(ref->nodeId.nodeId);
                }
                else if (browse_name == "Camera2")
                {
                    m_robot->camera2 = browse_camera2(ref->nodeId.nodeId);
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

} // namespace PMClient
