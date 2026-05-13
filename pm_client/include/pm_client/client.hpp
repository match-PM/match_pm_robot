#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "open62541/open62541.h"

#include "pm_client/debug.hpp"
#include "pm_client/error_handling.hpp"
#include "pm_client/robot.hpp"
#include "pm_client/util.hpp"

namespace PMClient
{

struct RobotDescriptor;

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

    /**
     * Helper function to read scalar node values.
     * Throws OpcuaException if the read fails or node has incorrect type.
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
            DebugLogger::log_status("Client", status, "read_node_value", "scalar read failed");
            throw OpcuaException(
                status,
                "read_node_value(scalar)",
                "Failed to read value from node"
            );
        }

        if (!UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPE]))
        {
            UA_Variant_clear(&value);
            DebugLogger::error(
                "Client",
                "Node type mismatch: expected scalar type, got array or wrong type"
            );
            throw NodeTypeException("scalar", "unexpected type");
        }

        if (value.data == nullptr)
        {
            UA_Variant_clear(&value);
            DebugLogger::error("Client", "Node value data is null after successful read");
            throw std::runtime_error("Node value data is null");
        }

        T data = *reinterpret_cast<T *>(value.data);

        UA_Variant_clear(&value);
        DebugLogger::debug("Client", "Successfully read scalar node value");

        return data;
    }

    /**
     * Helper function to read array node values.
     * Throws OpcuaException if the read fails or node has incorrect type/size.
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
            DebugLogger::log_status("Client", status, "read_node_values", "array read failed");
            throw OpcuaException(
                status,
                "read_node_values(array)",
                "Failed to read array from node"
            );
        }

        if (!UA_Variant_hasArrayType(&value, &UA_TYPES[UA_TYPE]))
        {
            UA_Variant_clear(&value);
            DebugLogger::error("Client", "Node type mismatch: expected array type");
            throw NodeTypeException("array", "scalar or wrong type");
        }

        if (value.arrayLength != count)
        {
            UA_Variant_clear(&value);
            DebugLogger::error(
                "Client",
                "Array size mismatch: expected " + std::to_string(count) + ", got " +
                    std::to_string(value.arrayLength)
            );
            throw std::runtime_error(
                "Array size mismatch: expected " + std::to_string(count) + ", got " +
                std::to_string(value.arrayLength)
            );
        }

        if (value.data == nullptr)
        {
            UA_Variant_clear(&value);
            DebugLogger::error("Client", "Array data is null after successful read");
            throw std::runtime_error("Array data is null");
        }

        T *data = reinterpret_cast<T *>(value.data);
        std::array<T, count> return_data;
        for (std::size_t i = 0; i < count; ++i)
        {
            return_data[i] = data[i];
        }

        UA_Variant_clear(&value);
        DebugLogger::debug(
            "Client",
            "Successfully read array node values (size: " + std::to_string(count) + ")"
        );

        return return_data;
    }

    /**
     * Helper function to write scalar node values.
     * Throws OpcuaException if the write fails.
     */
    template<typename T>
    void write_node_value(UA_NodeId node_id, T value)
    {
        constexpr std::size_t UA_TYPE = type_to_ua<T>::value;

        UA_Variant *variant = UA_Variant_new();
        if (variant == nullptr)
        {
            DebugLogger::error("Client", "Failed to allocate memory for variant");
            throw std::runtime_error("Failed to allocate memory for variant");
        }

        UA_Variant_setScalarCopy(variant, &value, &UA_TYPES[UA_TYPE]);

        UA_StatusCode status = UA_Client_writeValueAttribute(m_client, node_id, variant);

        if (status != UA_STATUSCODE_GOOD)
        {
            UA_Variant_delete(variant);
            DebugLogger::log_status("Client", status, "write_node_value", "scalar write failed");
            throw OpcuaException(
                status,
                "write_node_value(scalar)",
                "Failed to write value to node"
            );
        }

        UA_Variant_delete(variant);
        DebugLogger::debug("Client", "Successfully wrote scalar node value");
    }

    /**
     * Helper function to write array node values.
     * Throws OpcuaException if the write fails.
     */
    template<typename T, std::size_t count>
    void write_node_values(UA_NodeId node_id, std::array<T, count> values)
    {
        constexpr std::size_t UA_TYPE = type_to_ua<T>::value;

        UA_Variant *variant = UA_Variant_new();
        if (variant == nullptr)
        {
            DebugLogger::error("Client", "Failed to allocate memory for array variant");
            throw std::runtime_error("Failed to allocate memory for array variant");
        }

        UA_Variant_setArrayCopy(variant, values.data(), count, &UA_TYPES[UA_TYPE]);

        UA_StatusCode status = UA_Client_writeValueAttribute(m_client, node_id, variant);

        if (status != UA_STATUSCODE_GOOD)
        {
            UA_Variant_delete(variant);
            DebugLogger::log_status("Client", status, "write_node_values", "array write failed");
            throw OpcuaException(
                status,
                "write_node_values(array)",
                "Failed to write array to node"
            );
        }

        UA_Variant_delete(variant);
        DebugLogger::debug(
            "Client",
            "Successfully wrote array node values (size: " + std::to_string(count) + ")"
        );
    }

    void call_method(
        UA_NodeId object_id, UA_NodeId method_id, std::size_t input_size, UA_Variant *inputs,
        std::size_t *output_size, UA_Variant **outputs, std::string endpoint
    );
};

} // namespace PMClient
