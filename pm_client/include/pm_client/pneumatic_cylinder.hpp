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

    UA_NodeId is_forward_node_id = UA_NODEID_NULL;

    UA_NodeId is_backward_node_id = UA_NODEID_NULL;

    UA_NodeId move_forward_cmd_node_id = UA_NODEID_NULL;

    UA_NodeId move_backward_cmd_node_id = UA_NODEID_NULL;

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] bool get_is_forward() const;

    [[nodiscard]] bool get_is_backward() const;

    void move_forward();

    void move_backward();
};

} // namespace PMClient
