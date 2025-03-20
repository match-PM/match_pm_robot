
#include "pm_client/skills.hpp"
#include "pm_client/client.hpp"

namespace PMClient
{

void Skills::dispense(unsigned int time, int z_height, bool z_move) const
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

    UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);

    for (auto &input : inputs)
    {
        UA_Variant_clear(&input);
    }
}

} // namespace PMClient
