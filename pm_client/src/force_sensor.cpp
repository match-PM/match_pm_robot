#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool ForceSensor::is_ok() const
{
    return !UA_NodeId_isNull(&this->measurements);
}

[[nodiscard]] std::array<double, 7> ForceSensor::get_measurements() const
{
    return m_client->read_node_values<double, 7>(this->measurements);
}

[[nodiscard]] void ForceSensor::set_bias() const
{
    return m_client->write_node_value<bool>(this->bias, true);
}

} // namespace PMClient
