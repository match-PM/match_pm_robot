#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

enum class NozzleId
{
    Head,
    Gonio,
    Nest,
    DoseurGlue,
    DoseurGlue2K,
};

enum class NozzleState
{
    Vacuum = -1,
    Off = 0,
    Air = 1,
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

    UA_NodeId state_node_id = UA_NODEID_NULL;

    [[nodiscard]] bool is_ok() const;

    void set_state(NozzleState state);

    [[nodiscard]] NozzleState get_state() const;
};

} // namespace PMClient
