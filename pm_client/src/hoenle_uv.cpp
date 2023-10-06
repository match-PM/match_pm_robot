#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool HoenleUV::is_ok() const
{
    return !UA_NodeId_isNull(&this->on_off) && !UA_NodeId_isNull(&this->power) &&
           !UA_NodeId_isNull(&this->time);
}

[[nodiscard]] std::array<bool, 4> HoenleUV::get_on_off() const
{
    return m_client->read_node_values<bool, 4>(this->on_off);
}

void HoenleUV::set_on_off(std::array<bool, 4> values)
{
    m_client->write_node_values<bool, 4>(this->on_off, values);
}

void HoenleUV::set_power(std::array<int, 4> power)
{
    m_client->write_node_values<int, 4>(this->power, power);
}

void HoenleUV::set_time(std::array<double, 4> time)
{
    m_client->write_node_values<double, 4>(this->time, time);
}

} // namespace PMClient
