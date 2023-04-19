#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "open62541/open62541.h"

#include "pm_client/robot.hpp"

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

    /**
     * Initialisiert den Client. Danach können Anfragen an den Server gestellt werden.
     *
     * \returns `1` bei Fehler, sonst `0`.
     */
    void init();

    /**
     * Get reference to robot object.
     */
    Robot *get_robot()
    {
        return m_robot.get();
    }

    /**
     * Helper function to read node values.
     */
    template<typename T, std::size_t UA_TYPE>
    T read_node_value(UA_NodeId node_id)
    {
        UA_Variant value;
        UA_Variant_init(&value);

        UA_StatusCode status = UA_Client_readValueAttribute(m_client, node_id, &value);

        if (status != UA_STATUSCODE_GOOD)
        {
            throw std::runtime_error{UA_StatusCode_name(status)};
        }

        if (!UA_Variant_hasScalarType(&value, &UA_TYPES[UA_TYPE]))
        {
            throw std::runtime_error{
                "Tried to read value from node but node did not have expected type."};
        }

        T data = *reinterpret_cast<T *>(value.data);

        UA_Variant_clear(&value);

        return data;
    }

    /**
     * Helper function to write node values.
     */
    template<typename T, std::size_t UA_TYPE>
    void write_node_value(UA_NodeId node_id, T value)
    {
        UA_Variant *variant = UA_Variant_new();
        UA_Variant_setScalarCopy(variant, &value, &UA_TYPES[UA_TYPE]);

        UA_StatusCode status = UA_Client_writeValueAttribute(m_client, node_id, variant);

        if (status != UA_STATUSCODE_GOOD)
        {
            throw std::runtime_error{UA_StatusCode_name(status)};
        }

        UA_Variant_delete(variant);
    }
};

} // namespace PMClient
