#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

class Camera1
{
  private:
    Client *m_client;

  public:
    UA_NodeId coax_light = UA_NODEID_NULL;
    UA_NodeId ring_light = UA_NODEID_NULL;
    UA_NodeId ring_light_rgb = UA_NODEID_NULL;

    explicit Camera1(Client *client) : m_client{client}
    {
    }

    [[nodiscard]] bool is_ok() const;

    void set_coax_light(bool on);

    [[nodiscard]] bool get_coax_light() const;

    void set_ring_light(bool segment1, bool segment2, bool segment3, bool segment4);

    void get_ring_light(bool &segment1, bool &segment2, bool &segment3, bool &segment4) const;

    void set_ring_light_color(int red, int green, int blue);

    void get_ring_light_color(int &red, int &green, int &blue) const;
};

class Camera2
{
  private:
    Client *m_client;

  public:
    UA_NodeId light = UA_NODEID_NULL;

    explicit Camera2(Client *client) : m_client{client}
    {
    }

    [[nodiscard]] bool is_ok() const;

    void set_light(int intensity);

    [[nodiscard]] int get_light() const;
};

} // namespace PMClient
