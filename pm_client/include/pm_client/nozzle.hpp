#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

enum class NozzleId
{
    HeadNozzle,
    GonioNozzle
};

class Nozzle
{
  private:
    Client *m_client;

  public:
    explicit Nozzle(Client *client, NozzleId id) : m_client(client), nozzle_id(id)
    {
    }

    NozzleId nozzle_id;

    UA_NodeId vacuum_node_id = UA_NODEID_NULL;

    UA_NodeId air_node_id = UA_NODEID_NULL;

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] bool get_vacuum() const;

    [[nodiscard]] bool get_air() const;

    void set_vacuum(bool value);

    void set_air(bool value);
};

} // namespace PMClient
