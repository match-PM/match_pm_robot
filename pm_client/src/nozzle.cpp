#include "pm_client/nozzle.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool Nozzle::is_ok() const
{
    auto node_ids = {
        &vacuum_node_id,
        &air_node_id,
    };

    for (const auto &node_id : node_ids)
    {
        if (UA_NodeId_isNull(node_id))
        {
            return false;
        }
    }

    return true;
}

[[nodiscard]] bool Nozzle::get_vacuum() const
{
    return m_client->read_node_value<bool>(this->vacuum_node_id);
}

[[nodiscard]] bool Nozzle::get_air() const
{
    return m_client->read_node_value<bool>(this->air_node_id);
}

void Nozzle::set_air(bool value)
{
    m_client->write_node_value<bool>(this->vacuum_node_id, value);
}

void Nozzle::set_vacuum(bool value)
{
    m_client->write_node_value<bool>(this->air_node_id, value);
}

} // namespace PMClient
