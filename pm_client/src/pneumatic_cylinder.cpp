#include "pm_client/pneumatic_cylinder.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool PneumaticCylinder::is_ok() const
{
    auto node_ids = {
        &position_node_id,
        &move_cmd_node_id,
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

[[nodiscard]] Position PneumaticCylinder::get_position() const
{
    return static_cast<Position>(m_client->read_node_value<int>(this->position_node_id));
}

void PneumaticCylinder::move(Position position)
{
    m_client->write_node_value<int>(this->move_cmd_node_id, static_cast<int>(position));
}

} // namespace PMClient
