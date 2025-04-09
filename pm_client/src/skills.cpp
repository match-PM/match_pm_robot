
#include "pm_client/skills.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

bool Skills::is_ok() const
{
    return !UA_NodeId_isNull(&m_folder) && !UA_NodeId_isNull(&this->dispense_method) &&
           !UA_NodeId_isNull(&this->force_sensing_move_method);
}

bool Skills::dispense(unsigned int time, int z_height, bool z_move) const
{
    std::array<UA_Variant, 3> inputs =
        {make_variant(time), make_variant(z_height), make_variant(z_move)};

    std::size_t output_size;
    UA_Variant *outputs;

    m_client->call_method(
        m_folder,
        this->dispense_method,
        inputs.size(),
        inputs.data(),
        &output_size,
        &outputs
    );

    const UA_Boolean success = *static_cast<UA_Boolean *>(outputs[0].data);

    UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);

    for (auto &input : inputs)
    {
        UA_Variant_clear(&input);
    }

    return success;
}

bool Skills::force_sensing_move(
    int start_x, int start_y, int start_z, int target_x, int target_y, int target_z, float max_fx,
    float max_fy, float max_fz, unsigned int step_size
) const
{
    std::array<UA_Variant, 10> inputs = {
        make_variant(start_x),
        make_variant(start_y),
        make_variant(start_z),
        make_variant(target_x),
        make_variant(target_y),
        make_variant(target_z),
        make_variant(max_fx),
        make_variant(max_fy),
        make_variant(max_fz),
        make_variant(step_size)
    };

    std::size_t output_size;
    UA_Variant *outputs;

    m_client->call_method(
        m_folder,
        this->force_sensing_move_method,
        inputs.size(),
        inputs.data(),
        &output_size,
        &outputs
    );

    const UA_Boolean success = *static_cast<UA_Boolean *>(outputs[0].data);

    UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);

    for (auto &input : inputs)
    {
        UA_Variant_clear(&input);
    }

    return success;
}

} // namespace PMClient
