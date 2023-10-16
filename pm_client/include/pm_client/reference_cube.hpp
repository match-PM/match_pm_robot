#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

class ReferenceCube
{
  private:
    Client *m_client;

  public:
    UA_NodeId pushed = UA_NODEID_NULL;

    explicit ReferenceCube(Client *client) : m_client{client}
    {
    }

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] bool get_pushed() const;
};

} // namespace PMClient
