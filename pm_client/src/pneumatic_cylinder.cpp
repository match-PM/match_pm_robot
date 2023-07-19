#include "pm_client/pneumatic_cylinder.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool PneumaticCylinder::is_ok() const
{
    auto node_ids = {
        &is_forward_node_id,
        &is_backward_node_id,
        &move_forward_cmd_node_id,
        &move_backward_cmd_node_id,
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

[[nodiscard]] bool PneumaticCylinder::get_is_forward() const
{
    return m_client->read_node_value<bool>(this->is_forward_node_id);
}

[[nodiscard]] bool PneumaticCylinder::get_is_backward() const

{
    return m_client->read_node_value<bool>(this->is_backward_node_id);
}

void PneumaticCylinder::move_forward()
{
    m_client->write_node_value<bool>(this->move_forward_cmd_node_id, true);
}

void PneumaticCylinder::move_backward()
{
    m_client->write_node_value<bool>(this->move_backward_cmd_node_id, true);
}

} // namespace PMClient
