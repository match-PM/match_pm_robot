#pragma once

#include <string>
#include <vector>
#include <limits>

#include "open62541/open62541.h"

namespace PMClient
{

class Client;

struct ForceSensingMoveResult
{
    bool success;
    std::string error;
    bool threshold_exceeded;
};

struct LaserMeasurementState
{
    double measurement = std::numeric_limits<double>::quiet_NaN();
    UA_StatusCode status = UA_STATUSCODE_BADWAITINGFORINITIALDATA;
    UA_DateTime source_timestamp = 0;
    UA_UInt32 sequence = 0;
    UA_UInt32 age_ms = 0xFFFFFFFFu;
    bool scanning = false;
    std::string message;
    bool valid() const { return status == UA_STATUSCODE_GOOD && !scanning && age_ms <= 500; }
};

struct LaserMeasurementRequest
{
    UA_UInt32 id = 0;
    UA_DateTime server_epoch = 0;
};

struct LaserLineScanResult
{
    bool success = false;
    std::vector<double> distances;
    std::string message;
    UA_UInt32 actual_length_um = 0;
    UA_UInt32 actual_interval_um = 0;
    bool restoration_verified = false;
};

class Skills
{
  private:
    Client *m_client;
    UA_NodeId m_folder = UA_NODEID_NULL;

  public:
    UA_NodeId dispense_method = UA_NODEID_NULL;
    UA_NodeId force_sensing_move_method = UA_NODEID_NULL;
    UA_NodeId laser_line_scan_method = UA_NODEID_NULL;
    UA_NodeId start_laser_line_scan_method = UA_NODEID_NULL;
    UA_NodeId get_laser_line_scan_result_method = UA_NODEID_NULL;
    UA_NodeId laser_measurement_state_method = UA_NODEID_NULL;
    UA_NodeId request_laser_measurement_method = UA_NODEID_NULL;
    UA_NodeId laser_measurement_result_method = UA_NODEID_NULL;

    explicit Skills(Client *client, UA_NodeId folder) : m_client{client}, m_folder{folder}
    {
    }

    [[nodiscard]] bool is_ok() const;

    [[nodiscard]] bool
    dispense(unsigned int time, unsigned int z_height, bool z_move, std::string endpoint) const;

    [[nodiscard]] LaserLineScanResult laser_line_scan(
        unsigned int interval, unsigned int length_um, std::string endpoint
    ) const;

    [[nodiscard]] LaserLineScanResult laser_line_scan(
        unsigned int interval, unsigned int length_um, std::string endpoint,
        unsigned int timeout_ms
    ) const;

    [[nodiscard]] LaserMeasurementState laser_measurement_state(unsigned int call_timeout_ms = 1000) const;
    [[nodiscard]] LaserMeasurementRequest request_laser_measurement(
        unsigned int timeout_ms, unsigned int call_timeout_ms = 1000) const;
    [[nodiscard]] LaserMeasurementState laser_measurement_result(
        const LaserMeasurementRequest &request, bool &done, unsigned int call_timeout_ms = 1000) const;

    [[nodiscard]] ForceSensingMoveResult force_sensing_move(
        int start_x, int start_y, int start_z, int start_t, int target_x, int target_y,
        int target_z, float max_fx, float max_fy, float max_fz, unsigned int step_size,
        std::string endpoint
    ) const;
};

} // namespace PMClient
