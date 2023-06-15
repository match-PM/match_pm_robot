#pragma once

#include "open62541/open62541.h"

namespace PMClient
{

template<typename T>
struct type_to_ua
{
};

template<>
struct type_to_ua<bool>
{
    std::size_t value = UA_TYPES_BOOLEAN;
};

template<>
struct type_to_ua<int>
{
    std::size_t value = UA_TYPES_INT32;
};

template<>
struct type_to_ua<double>
{
    std::size_t value = UA_TYPES_DOUBLE;
};

} // namespace PMClient
