#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

enum class PneumaticId
{
    UV1,
    UV2,
    Glue,
    Glue2K,
    CameraMire,
    ProtectDoseur,
};

enum class Position
{
    Forward = 1,
    Neutral = 0,
    Backward = -1,
};

class PneumaticCylinder
{
  private:
    Client *m_client;

  public:
    explicit PneumaticCylinder(Client *client, PneumaticId id) : m_client(client), pneumatic_id(id)
    {
    }

    PneumaticId pneumatic_id;

    UA_NodeId position_node_id = UA_NODEID_NULL;

    UA_NodeId move_cmd_node_id = UA_NODEID_NULL;

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] Position get_position() const;

    void move(Position dir);
};

} // namespace PMClient
