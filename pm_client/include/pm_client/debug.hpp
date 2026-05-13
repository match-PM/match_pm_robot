#pragma once

#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "open62541/open62541.h"

namespace PMClient
{

/**
 * Debug logging levels
 */
enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error,
    Critical
};

/**
 * Simple debug logger for pm_client
 * Can be configured to output to console, ROS2 logging, or custom handlers
 */
class DebugLogger
{
  public:
    /**
     * Set global log level - messages below this level are ignored
     */
    static void set_level(LogLevel level)
    {
        s_min_level = level;
    }

    /**
     * Get current global log level
     */
    static LogLevel get_level()
    {
        return s_min_level;
    }

    /**
     * Enable/disable console output
     */
    static void set_console_output(bool enabled)
    {
        s_console_enabled = enabled;
    }

    /**
     * Log a message at specified level
     */
    static void log(LogLevel level, const std::string &category, const std::string &message)
    {
        if (level < s_min_level)
            return;

        if (s_console_enabled)
        {
            print_to_console(level, category, message);
        }
    }

    /**
     * Log with formatted string (printf-style)
     */
    template<typename... Args>
    static void logf(LogLevel level, const std::string &category, const char *format, Args... args)
    {
        if (level < s_min_level)
            return;

        // Simple formatting using ostringstream
        std::ostringstream oss;
        format_message(oss, format, args...);
        log(level, category, oss.str());
    }

    // Convenience methods
    static void debug(const std::string &category, const std::string &message)
    {
        log(LogLevel::Debug, category, message);
    }

    static void info(const std::string &category, const std::string &message)
    {
        log(LogLevel::Info, category, message);
    }

    static void warning(const std::string &category, const std::string &message)
    {
        log(LogLevel::Warning, category, message);
    }

    static void error(const std::string &category, const std::string &message)
    {
        log(LogLevel::Error, category, message);
    }

    static void critical(const std::string &category, const std::string &message)
    {
        log(LogLevel::Critical, category, message);
    }

    /**
     * Log an OPC UA status code with context
     */
    static void log_status(
        const std::string &category, UA_StatusCode status, const std::string &operation,
        const std::string &context = ""
    )
    {
        std::ostringstream oss;
        oss << "OPC UA Status [" << operation << "]: " << UA_StatusCode_name(status);
        if (!context.empty())
        {
            oss << " | " << context;
        }

        LogLevel level = (status == UA_STATUSCODE_GOOD) ? LogLevel::Debug : LogLevel::Error;
        log(level, category, oss.str());
    }

  private:
    static LogLevel s_min_level;
    static bool s_console_enabled;

    static void
    print_to_console(LogLevel level, const std::string &category, const std::string &message)
    {
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);

        std::string level_str = level_to_string(level);
        std::cerr << "[" << std::put_time(&tm, "%H:%M:%S") << "] [" << level_str << "] ["
                  << category << "] " << message << std::endl;
    }

    static std::string level_to_string(LogLevel level)
    {
        switch (level)
        {
            case LogLevel::Debug:
                return "DEBUG";
            case LogLevel::Info:
                return "INFO";
            case LogLevel::Warning:
                return "WARN";
            case LogLevel::Error:
                return "ERROR";
            case LogLevel::Critical:
                return "CRIT";
            default:
                return "UNKNOWN";
        }
    }

    // Simple printf-style formatting helper
    template<typename T>
    static void format_message(std::ostringstream &oss, const char *fmt, T arg)
    {
        while (*fmt != '\0')
        {
            if (*fmt == '%' && *(fmt + 1) != '%')
            {
                oss << arg;
                fmt += 2;
                break;
            }
            if (*fmt == '%' && *(fmt + 1) == '%')
            {
                oss << '%';
                fmt += 2;
                continue;
            }
            oss << *fmt;
            ++fmt;
        }
        while (*fmt != '\0')
        {
            oss << *fmt;
            ++fmt;
        }
    }

    template<typename T, typename... Args>
    static void format_message(std::ostringstream &oss, const char *fmt, T arg, Args... args)
    {
        while (*fmt != '\0')
        {
            if (*fmt == '%' && *(fmt + 1) != '%')
            {
                oss << arg;
                format_message(oss, fmt + 2, args...);
                return;
            }
            if (*fmt == '%' && *(fmt + 1) == '%')
            {
                oss << '%';
                fmt += 2;
                continue;
            }
            oss << *fmt;
            ++fmt;
        }
    }
};

// Initialize static members
inline LogLevel DebugLogger::s_min_level = LogLevel::Warning;
inline bool DebugLogger::s_console_enabled = true;

} // namespace PMClient
