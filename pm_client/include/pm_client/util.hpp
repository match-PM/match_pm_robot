#pragma once

#include <cstdint>

#include "open62541/open62541.h"

namespace PMClient
{

static_assert(sizeof(bool) == sizeof(UA_Boolean));
static_assert(sizeof(int) == sizeof(UA_Int32));
static_assert(sizeof(double) == sizeof(UA_Double));

template<typename T>
struct type_to_ua
{
};

template<>
struct type_to_ua<bool>
{
    static const std::size_t value = UA_TYPES_BOOLEAN;
};

template<>
struct type_to_ua<std::uint8_t>
{
    static const std::size_t value = UA_TYPES_BYTE;
};

template<>
struct type_to_ua<int>
{
    static const std::size_t value = UA_TYPES_INT32;
};

template<>
struct type_to_ua<double>
{
    static const std::size_t value = UA_TYPES_DOUBLE;
};

} // namespace PMClient
