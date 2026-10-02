#pragma once

#include <array>
#include <iostream>
#include <memory>
#include <string>

#include "open62541/open62541.h"

#include "pm_client/robot.hpp"
#include "pm_client/util.hpp"

namespace PMClient
{

struct RobotDescriptor;

// Snapshots group nodes with similar timing requirements into one OPC UA call.
struct AuxiliaryIoState
{
    std::array<Position, 6> pneumatic_positions{};
    std::array<NozzleState, 7> nozzle_states{};
    bool reference_cube_pushed = false;
};

struct AuxiliarySensorState
{
    double laser_measurement = 0.0;
    std::array<double, 7> force_measurements{};
};

struct AuxiliaryLightingState
{
    bool camera1_coax_light = false;
    std::array<bool, 4> camera1_ring_light{};
    std::array<int, 3> camera1_ring_light_rgb{};
    int camera2_light = 0;
    std::array<bool, 4> uv_on_off{};
    std::array<int, 4> uv_power{};
    std::array<double, 4> uv_time{};
};

struct AuxiliaryCommands
{
    std::array<bool, 6> pneumatic_changed{};
    std::array<Position, 6> pneumatic_positions{};
    std::array<bool, 7> nozzle_changed{};
    std::array<NozzleState, 7> nozzle_states{};

    bool camera1_coax_light_changed = false;
    bool camera1_coax_light = false;
    bool camera1_ring_light_changed = false;
    std::array<bool, 4> camera1_ring_light{};
    bool camera1_ring_light_rgb_changed = false;
    std::array<int, 3> camera1_ring_light_rgb{};
    bool camera2_light_changed = false;
    int camera2_light = 0;

    bool uv_on_off_changed = false;
    std::array<bool, 4> uv_on_off{};
    bool uv_power_changed = false;
    std::array<int, 4> uv_power{};
    bool uv_time_changed = false;
    std::array<double, 4> uv_time{};

    bool force_sensor_bias = false;

    [[nodiscard]] bool empty() const;
};

/**
 * Der Client ermöglicht die Kommunikation zwischen der PM-Zelle
 * und anderen Rechnern/Programmen.
 *
 * Über den Client kann eine Verbindung zum OPCUA-Server der PM-Zelle
 * aufgebaut werden und es können Daten und Befehle ausgetauscht werden.
 */
class Client
{
    /**
     * Internal referenc to UA Client, which provides the connection
     * to the OPCUA server.
     */
    UA_Client *m_client;

    /**
     * Internal reference to Robot object through which the robot can be controlled.
     */
    std::unique_ptr<Robot> m_robot;

  public:
    /**
     * Konstruktor.
     *
     * Nach Konstruktion muss die `Client::connect` Funktion aufgerufen werden.
     */
    explicit Client();

    Client(const Client &) = delete;
    Client &operator=(const Client &) = delete;

    ~Client();

    /**
     * Verbindet den Client mit dem gegebenen OPCUA-Server.
     *
     * \param endpoint URL des OPCUA-Servers in der Form `"opc.tcp://{host}:{port}"`.
     * \returns `1` bei Fehler, sonst `0`.
     */
    void connect(std::string endpoint);

    void disconnect();

    /**
     * Initialisiert den Client. Danach können Anfragen an den Server gestellt werden.
     *
     * \returns `1` bei Fehler, sonst `0`.
     */
    void init();

    /**
     * Get reference to robot object.
     */
    Robot &get_robot()
    {
        return *m_robot;
    }

    void set_timeout(UA_UInt32 timeout_ms);

    // One service request carries position and speed for all eight axes.
    bool read_axis_motion_states(std::array<AxisMotionState, 8> &states);

    // One service request carries only axis values that changed.
    bool write_axis_motion_commands(const std::array<AxisMotionCommand, 8> &commands);

    bool read_auxiliary_io_state(AuxiliaryIoState &state);

    bool read_auxiliary_sensor_state(AuxiliarySensorState &state);

    bool read_auxiliary_lighting_state(AuxiliaryLightingState &state);

    bool write_auxiliary_commands(const AuxiliaryCommands &commands);

    /**
     * Helper function to read scalar node values.
     */
    template<typename T>
    T read_node_value(UA_NodeId node_id)
    {
        constexpr std::size_t UA_TYPE = type_to_ua<T>::value;

        UA_Variant value;
        UA_Variant_init(&value);

        UA_StatusCode status = UA_Client_readValueAttribute(m_client, node_id, &value);

        if (status != UA_STATUSCODE_GOOD)
        {
            // throw std::runtime_error{UA_StatusCode_name(status)};
            return {};
        }

        if (!UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPE]))
        {
            throw std::runtime_error{
                "Tried to read value from node but node did not have expected type."
            };
        }

        T data = *reinterpret_cast<T *>(value.data);

        UA_Variant_clear(&value);

        return data;
    }

    /**
     * Helper function to read array node values.
     */
    template<typename T, std::size_t count>
    std::array<T, count> read_node_values(UA_NodeId node_id)
    {
        constexpr std::size_t UA_TYPE = type_to_ua<T>::value;

        UA_Variant value;
        UA_Variant_init(&value);

        UA_StatusCode status = UA_Client_readValueAttribute(m_client, node_id, &value);

        if (status != UA_STATUSCODE_GOOD)
        {
            // throw std::runtime_error{UA_StatusCode_name(status)};
            return {};
        }

        if (!UA_Variant_hasArrayType(&value, &UA_TYPES[UA_TYPE]) || value.arrayLength != count)
        {
            throw std::runtime_error{
                "Tried to read value from node but node did not have expected type."
            };
        }

        T *data = reinterpret_cast<T *>(value.data);
        std::array<T, count> return_data;
        for (std::size_t i = 0; i < count; ++i)
        {
            return_data[i] = data[i];
        }

        UA_Variant_clear(&value);

        return return_data;
    }

    /**
     * Helper function to write scalar node values.
     */
    template<typename T>
    void write_node_value(UA_NodeId node_id, T value)
    {
        constexpr std::size_t UA_TYPE = type_to_ua<T>::value;

        UA_Variant *variant = UA_Variant_new();
        UA_Variant_setScalarCopy(variant, &value, &UA_TYPES[UA_TYPE]);

        UA_StatusCode status = UA_Client_writeValueAttribute(m_client, node_id, variant);

        if (status != UA_STATUSCODE_GOOD)
        {
            // throw std::runtime_error{UA_StatusCode_name(status)};
            return;
        }

        UA_Variant_delete(variant);
    }

    /**
     * Helper function to write array node values.
     */
    template<typename T, std::size_t count>
    void write_node_values(UA_NodeId node_id, std::array<T, count> values)
    {
        constexpr std::size_t UA_TYPE = type_to_ua<T>::value;

        UA_Variant *variant = UA_Variant_new();
        UA_Variant_setArrayCopy(variant, values.data(), count, &UA_TYPES[UA_TYPE]);

        UA_StatusCode status = UA_Client_writeValueAttribute(m_client, node_id, variant);

        if (status != UA_STATUSCODE_GOOD)
        {
            // throw std::runtime_error{UA_StatusCode_name(status)};
            return;
        }

        UA_Variant_delete(variant);
    }

    void call_method(
        UA_NodeId object_id, UA_NodeId method_id, std::size_t input_size, UA_Variant *inputs,
        std::size_t *output_size, UA_Variant **outputs, std::string endpoint
    );
};

} // namespace PMClient
