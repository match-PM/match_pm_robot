#include <initializer_list>

#include "pm_client/camera.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool Laser::is_ok() const
{
    return !UA_NodeId_isNull(&this->measurement);
}

[[nodiscard]] double Laser::get_measurement() const
{
    return m_client->read_node_value<double>(this->measurement);
}

} // namespace PMClient
