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
    UA_NodeId force_sensing_move_method = UA_NODEID_NULL;

    explicit Skills(Client *client, UA_NodeId folder) : m_client{client}, m_folder{folder}
    {
    }

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] bool dispense(unsigned int time, int z_height, bool z_move) const;

    [[nodiscard]] bool force_sensing_move(
        int start_x, int start_y, int start_z, int target_x, int target_y, int target_z,
        float max_fx, float max_fy, float max_fz, unsigned int step_size
    ) const;
};

} // namespace PMClient
