#include <initializer_list>

#include "pm_client/camera.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

[[nodiscard]] bool Camera1::is_ok() const
{
    auto node_ids = {
        &coax_light,
        &ring_light,
        &ring_light_rgb,
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

void Camera1::set_coax_light(bool on)
{
    m_client->write_node_value<bool>(this->coax_light, on);
}

[[nodiscard]] bool Camera1::get_coax_light() const
{
    return m_client->read_node_value<bool>(this->coax_light);
}

void Camera1::set_ring_light(bool segment1, bool segment2, bool segment3, bool segment4)
{
    std::array<bool, 4> data = {segment1, segment2, segment3, segment4};
    m_client->write_node_values<bool, 4>(this->ring_light, data);
}

void Camera1::get_right_light(bool &segment1, bool &segment2, bool &segment3, bool &segment4) const
{
    std::array<bool, 4> data = m_client->read_node_values<bool, 4>(this->ring_light);
    segment1 = data[0];
    segment2 = data[1];
    segment3 = data[2];
    segment4 = data[3];
}

void Camera1::set_ring_light_color(int red, int green, int blue)
{
    std::array<int, 3> data = {red, green, blue};
    m_client->write_node_values<int, 3>(this->ring_light_rgb, data);
}

void Camera1::get_ring_light_color(int &red, int &green, int &blue) const
{
    std::array<int, 3> data = m_client->read_node_values<int, 3>(this->ring_light_rgb);
    red = data[0];
    green = data[1];
    blue = data[2];
}

[[nodiscard]] bool Camera2::is_ok() const
{
    if (UA_NodeId_isNull(&light))
    {
        return false;
    }

    return true;
}

void Camera2::set_light(int intensity)
{
    m_client->write_node_value<int>(this->light, intensity);
}

[[nodiscard]] int Camera2::get_light() const
{
    return m_client->read_node_value<int>(this->light);
}

} // namespace PMClient
