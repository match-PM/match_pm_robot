#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

#include "open62541/open62541.h"

namespace PMClient
{

/**
 * Exception thrown when an OPC UA operation fails.
 * Includes status code and contextual information.
 */
class OpcuaException : public std::runtime_error
{
  private:
    UA_StatusCode m_status_code;
    std::string m_context;

  public:
    OpcuaException(
        UA_StatusCode status_code, const std::string &operation, const std::string &context = ""
    )
        : std::runtime_error(format_message(status_code, operation, context)),
          m_status_code(status_code), m_context(context)
    {
    }

    /**
     * Get the OPC UA status code
     */
    [[nodiscard]] UA_StatusCode status_code() const
    {
        return m_status_code;
    }

    /**
     * Get additional context about the error
     */
    [[nodiscard]] const std::string &context() const
    {
        return m_context;
    }

  private:
    static std::string
    format_message(UA_StatusCode status, const std::string &operation, const std::string &context)
    {
        std::ostringstream oss;
        oss << "OPC UA operation failed: " << operation
            << " | Status: " << UA_StatusCode_name(status);
        if (!context.empty())
        {
            oss << " | Context: " << context;
        }
        return oss.str();
    }
};

/**
 * Exception thrown when a node has unexpected type or structure
 */
class NodeTypeException : public std::runtime_error
{
  public:
    NodeTypeException(const std::string &expected, const std::string &actual)
        : std::runtime_error("Node type mismatch: expected " + expected + ", got " + actual)
    {
    }
};

/**
 * Exception thrown when a node ID is null or invalid
 */
class InvalidNodeException : public std::runtime_error
{
  public:
    InvalidNodeException(const std::string &node_name, const std::string &context = "")
        : std::runtime_error(
              "Invalid node: " + node_name + (context.empty() ? "" : " (" + context + ")")
          )
    {
    }
};

/**
 * Exception thrown during client initialization if required nodes are missing
 */
class InitializationException : public std::runtime_error
{
  public:
    explicit InitializationException(const std::string &message)
        : std::runtime_error("PM Client initialization failed: " + message)
    {
    }
};

/**
 * Helper function to convert OPC UA status code to string with description
 */
inline std::string status_code_to_string(UA_StatusCode status)
{
    std::ostringstream oss;
    oss << UA_StatusCode_name(status) << " (0x" << std::hex << status << ")";
    return oss.str();
}

/**
 * Check if status code indicates success
 */
[[nodiscard]] inline bool is_status_good(UA_StatusCode status)
{
    return (status >> 16) == 0x0000; // Success category in OPC UA
}

/**
 * Check if status code indicates a connection error
 */
[[nodiscard]] inline bool is_connection_error(UA_StatusCode status)
{
    // Connection errors typically fall in security or network categories
    return status == UA_STATUSCODE_BADCONNECTIONCLOSED ||
           status == UA_STATUSCODE_BADCONNECTIONREJECTED ||
           status == UA_STATUSCODE_BADNOTCONNECTED ||
           status == UA_STATUSCODE_BADSECURITYCHECKSFAILED;
}

/**
 * Check if status code indicates a timeout
 */
[[nodiscard]] inline bool is_timeout_error(UA_StatusCode status)
{
    return status == UA_STATUSCODE_BADTIMEOUT || status == UA_STATUSCODE_BADREQUESTTOOLARGE;
}

} // namespace PMClient
