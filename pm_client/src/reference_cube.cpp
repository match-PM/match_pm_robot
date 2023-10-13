#include <initializer_list>

#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool ReferenceCube::is_ok() const
{
    return !UA_NodeId_isNull(&this->pushed);
}

[[nodiscard]] bool ReferenceCube::get_pushed() const
{
    return m_client->read_node_value<bool>(this->pushed);
}

} // namespace PMClient
