

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

    auto browse_reference_cube = [&](UA_NodeId reference_cube_node
                                 ) -> std::unique_ptr<ReferenceCube> {
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

void Client::call_method(
    UA_NodeId object_id, UA_NodeId method_id, std::size_t input_size, UA_Variant *inputs,
    std::size_t *output_size, UA_Variant **outputs
)
{
    UA_StatusCode status =
        UA_Client_call(m_client, object_id, method_id, input_size, inputs, output_size, outputs);

    if (status != UA_STATUSCODE_GOOD)
    {
        throw std::runtime_error{UA_StatusCode_name(status)};
    }
}

} // namespace PMClient
