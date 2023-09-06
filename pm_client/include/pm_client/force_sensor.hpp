#pragma once

#include <array>

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

class ForceSensor
{
  private:
    Client *m_client;

  public:
    UA_NodeId measurements = UA_NODEID_NULL;

    explicit ForceSensor(Client *client) : m_client{client}
    {
    }

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] std::array<double, 7> get_measurements() const;
};

} // namespace PMClient
