#pragma once

#include <array>

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

class HoenleUV
{
  private:
    Client *m_client;

  public:
    UA_NodeId on_off = UA_NODEID_NULL;
    UA_NodeId time = UA_NODEID_NULL;
    UA_NodeId power = UA_NODEID_NULL;

    explicit HoenleUV(Client *client) : m_client{client}
    {
    }

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] std::array<bool, 4> get_on_off() const;

    void set_on_off(std::array<bool, 4> values);

    void set_power(std::array<int, 4> power);

    void set_time(std::array<double, 4> time);
};

} // namespace PMClient
