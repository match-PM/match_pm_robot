#include <exception>

#include "pm_client/client.hpp"
#include "pm_client/nozzle.hpp"

namespace PMClient
{

[[nodiscard]] bool Nozzle::is_ok() const
{
    return !UA_NodeId_isNull(&state_node_id);
}

void Nozzle::set_state(NozzleState state)
{
    m_client->write_node_value<int>(this->state_node_id, static_cast<int>(state));
}

[[nodiscard]] NozzleState Nozzle::get_state() const
{
    auto state = m_client->read_node_value<int>(this->state_node_id);
    if (state > 1 || state < -1)
    {
        throw std::runtime_error{"invalid nozzle state"};
    }

    return static_cast<NozzleState>(state);
}

} // namespace PMClient
