#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

class Skills
{
  private:
    Client *m_client;
    UA_NodeId m_folder = UA_NODEID_NULL;

  public:
    UA_NodeId dispense_method = UA_NODEID_NULL;

    explicit Skills(Client *client, UA_NodeId folder) : m_client{client}, m_folder{folder}
    {
    }

    [[nodiscard]] bool is_ok() const;

    void dispense(unsigned int time, int z_height, bool z_move) const;
};

} // namespace PMClient
