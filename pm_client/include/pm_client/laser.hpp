#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

class Laser
{
  private:
    Client *m_client;

  public:
    UA_NodeId measurement = UA_NODEID_NULL;

    explicit Laser(Client *client) : m_client{client}
    {
    }

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] double get_measurement() const;
};

} // namespace PMClient
