
#include "pm_client/skills.hpp"
#include "pm_client/client.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace PMClient
{

bool Skills::is_ok() const
{
    return !UA_NodeId_isNull(&m_folder) && !UA_NodeId_isNull(&this->dispense_method) &&
           !UA_NodeId_isNull(&this->force_sensing_move_method);
}

// bool Skills::dispense(unsigned int time, unsigned int z_height, bool z_move) const
// {
//     std::array<UA_Variant, 3> inputs =
//         {make_variant(time), make_variant(z_height), make_variant(z_move)};

//     std::size_t output_size;
//     UA_Variant *outputs;

//     m_client->call_method(
//         m_folder,
//         this->dispense_method,
//         inputs.size(),
//         inputs.data(),
//         &output_size,
//         &outputs
//     );

//     const UA_Boolean success = *static_cast<UA_Boolean *>(outputs[0].data);

//     UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);

//     for (auto &input : inputs)
//     {
//         UA_Variant_clear(&input);
//     }

//     return success;
// }

// LUIS version
// bool Skills::dispense(unsigned int time, unsigned int z_height, bool z_move) const
// {
//     std::array<UA_Variant, 3> inputs =
//         {make_variant(time), make_variant(z_height), make_variant(z_move)};

//     std::size_t output_size = 0;
//     UA_Variant *outputs = nullptr;

//     m_client->call_method(
//         m_folder,
//         this->dispense_method,
//         inputs.size(),
//         inputs.data(),
//         &output_size,
//         &outputs
//     );

//     bool success = false;

//     if (output_size > 0 && outputs != nullptr)
//     {
//         if (outputs[0].type == &UA_TYPES[UA_TYPES_BOOLEAN] && outputs[0].data != nullptr)
//         {
//             success = *static_cast<UA_Boolean *>(outputs[0].data) != 0;
//         }
//     }

//     UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);

//     for (auto &input : inputs)
//     {
//         UA_Variant_clear(&input);
//     }

//     return success;
// }

// chat saver method
bool Skills::dispense(
    unsigned int time, unsigned int z_height, bool z_move, std::string endpoint
) const
{
    // Defensive default
    bool success = false;

    // Prepare input variants
    std::array<UA_Variant, 3> inputs;
    for (auto &v : inputs)
        UA_Variant_init(&v);

    inputs[0] = make_variant(time);
    inputs[1] = make_variant(z_height);
    inputs[2] = make_variant(z_move);

    std::size_t output_size = 0;
    UA_Variant *outputs = nullptr;

    try
    {
        // Optional but highly recommended:
        // check client state before calling
        // if (!m_client || !m_client->is_connected())
        // {
        //     throw std::runtime_error("OPC UA client not connected");
        // }

        // UA_StatusCode status = m_client->call_method(
        //     m_folder,
        //     this->dispense_method,
        //     inputs.size(),
        //     inputs.data(),
        //     &output_size,
        //     &outputs
        // );

        m_client->call_method(
            m_folder,
            this->dispense_method,
            inputs.size(),
            inputs.data(),
            &output_size,
            &outputs,
            endpoint
        );

        // if (status != UA_STATUSCODE_GOOD)
        // {
        //     throw std::runtime_error(
        //         std::string("call_method failed: ") + UA_StatusCode_name(status)
        //     );
        // }

        if (output_size > 0 && outputs != nullptr &&
            outputs[0].type == &UA_TYPES[UA_TYPES_BOOLEAN] && outputs[0].data != nullptr)
        {
            success = (*static_cast<UA_Boolean *>(outputs[0].data)) != UA_FALSE;
        }
    }
    catch (const std::exception &e)
    {
        // Log but do not crash the process
        // std::cerr << "[dispense] OPC UA error: " << e.what() << std::endl;
        success = false;
    }

    // Cleanup outputs safely
    if (outputs != nullptr)
    {
        UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);
    }

    // Cleanup inputs
    for (auto &input : inputs)
    {
        UA_Variant_clear(&input);
    }

    return success;
}

LaserLineScanResult Skills::laser_line_scan(
    unsigned int interval, unsigned int length_um, std::string endpoint
) const
{
    return laser_line_scan(interval, length_um, endpoint, 120000);
}

LaserLineScanResult Skills::laser_line_scan(
    unsigned int interval, unsigned int length_um, std::string endpoint, unsigned int timeout_ms
) const
{
    // Keep the existing API, but never reconnect and replay a side-effecting scan.
    (void)endpoint;
    LaserLineScanResult result;
    if (length_um == 0 || length_um > 1100 || interval == 0)
    {
        result.message = "Length must be 1..1100 um and interval must be greater than zero";
        return result;
    }

    if (timeout_ms == 0)
    {
        result.message = "Laser scan timeout must be greater than zero";
        return result;
    }

    if (m_client == nullptr || UA_NodeId_isNull(&m_folder) ||
        UA_NodeId_isNull(&start_laser_line_scan_method) ||
        UA_NodeId_isNull(&get_laser_line_scan_result_method))
    {
        result.message = "OPC UA server requires StartLaserLineScan and GetLaserLineScanResult; "
                         "update the server before using this client";
        return result;
    }

    // The server expects interval first, then length, both as UInt32 scalars.
    std::array<UA_Variant, 2> inputs = {make_variant(interval), make_variant(length_um)};
    std::size_t output_size = 0;
    UA_Variant *outputs = nullptr;

    const auto read_string = [](const UA_Variant &value) {
        const auto &text = *static_cast<const UA_String *>(value.data);
        if (text.length == 0)
        {
            return std::string{};
        }
        if (text.data == nullptr)
        {
            throw std::runtime_error("Invalid LaserLineScan OPC UA string");
        }
        return std::string(reinterpret_cast<const char *>(text.data), text.length);
    };

    try
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        const auto remaining_ms = [&]() -> UA_UInt32 {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) throw std::runtime_error("Timed out waiting for laser scan; server may still be acquiring/restoring");
            return static_cast<UA_UInt32>(std::max<std::int64_t>(1,
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()));
        };
        m_client->call_method_once(
            m_folder,
            start_laser_line_scan_method,
            inputs.size(),
            inputs.data(),
            &output_size,
            &outputs,
            remaining_ms()
        );
        (void)remaining_ms();

        if (output_size != 3 || outputs == nullptr ||
            !UA_Variant_hasScalarType(&outputs[0], &UA_TYPES[UA_TYPES_UINT32]) ||
            !UA_Variant_hasScalarType(&outputs[1], &UA_TYPES[UA_TYPES_BOOLEAN]) ||
            !UA_Variant_hasScalarType(&outputs[2], &UA_TYPES[UA_TYPES_STRING]))
        {
            throw std::runtime_error("Unexpected StartLaserLineScan output types or count");
        }
        if (*static_cast<const UA_Boolean *>(outputs[1].data) == UA_FALSE)
        {
            throw std::runtime_error(read_string(outputs[2]));
        }
        auto job_id = *static_cast<const UA_UInt32 *>(outputs[0].data);
        if (job_id == 0)
        {
            throw std::runtime_error("Server returned an invalid laser scan ID");
        }
        UA_Variant job_input;
        UA_Variant_init(&job_input);
        UA_Variant_setScalar(&job_input, &job_id, &UA_TYPES[UA_TYPES_UINT32]);
        job_input.storageType = UA_VARIANT_DATA_NODELETE;

        while (true)
        {
            if (std::chrono::steady_clock::now() >= deadline)
            {
                throw std::runtime_error("Timed out waiting for laser scan " + std::to_string(job_id) +
                                         "; the server may still be acquiring/restoring");
            }
            UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);
            outputs = nullptr;
            output_size = 0;
            m_client->call_method_once(
                m_folder, get_laser_line_scan_result_method, 1, &job_input, &output_size, &outputs,
                remaining_ms()
            );
            (void)remaining_ms();
            if (output_size != 7 || outputs == nullptr ||
                !UA_Variant_hasScalarType(&outputs[0], &UA_TYPES[UA_TYPES_STRING]) ||
                !UA_Variant_hasScalarType(&outputs[1], &UA_TYPES[UA_TYPES_STRING]) ||
                !UA_Variant_hasScalarType(&outputs[2], &UA_TYPES[UA_TYPES_BOOLEAN]) ||
                !UA_Variant_hasScalarType(&outputs[3], &UA_TYPES[UA_TYPES_BOOLEAN]) ||
                !UA_Variant_hasScalarType(&outputs[4], &UA_TYPES[UA_TYPES_UINT32]) ||
                !UA_Variant_hasScalarType(&outputs[5], &UA_TYPES[UA_TYPES_UINT32]) ||
                !UA_Variant_hasScalarType(&outputs[6], &UA_TYPES[UA_TYPES_BOOLEAN]))
            {
                throw std::runtime_error("Unexpected GetLaserLineScanResult output types or count");
            }
            if (*static_cast<const UA_Boolean *>(outputs[3].data) != UA_FALSE)
            {
                break;
            }
            const auto remaining = deadline - std::chrono::steady_clock::now();
            const auto poll_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::milliseconds(100)
            );
            std::this_thread::sleep_for(std::min(remaining, poll_period));
        }

        result.actual_length_um = *static_cast<const UA_UInt32 *>(outputs[4].data);
        result.actual_interval_um = *static_cast<const UA_UInt32 *>(outputs[5].data);
        result.restoration_verified = *static_cast<const UA_Boolean *>(outputs[6].data) != UA_FALSE;
        result.message = read_string(outputs[1]);
        const bool success = *static_cast<const UA_Boolean *>(outputs[2].data) != UA_FALSE;
        const auto profile = read_string(outputs[0]);
        if (!success)
        {
            // The server returns its raw profile even on failure. Expose enough
            // detail to diagnose empty data, invalid readings and count mismatches.
            const auto record_count = std::count(profile.begin(), profile.end(), ';');
            auto preview = profile.substr(0, 160);
            for (auto &character : preview)
            {
                if (static_cast<unsigned char>(character) < 0x20 || character == 0x7f)
                {
                    character = ' ';
                }
            }
            if (profile.size() > preview.size())
            {
                preview += "...";
            }

            result.message += " | requested_length_um=" + std::to_string(length_um) +
                              ", interval_um=" + std::to_string(interval) +
                              ", profile_bytes=" + std::to_string(profile.size()) +
                              ", records=" + std::to_string(record_count) +
                              ", invalid_marker=" +
                              (profile.find("--------") != std::string::npos ? "yes" : "no") +
                              ", raw_preview=[" + preview + "]";
        }
        else
        {
            if (profile.empty() || profile.back() != ';')
            {
                throw std::runtime_error("Laser returned an empty or incomplete profile");
            }

            std::istringstream records(profile);
            std::string record;
            while (std::getline(records, record, ';'))
            {
                // The full height must occupy [3,11), followed by two numeric fields.
                const auto first = record.find_first_not_of(" \t\r\n");
                const auto last_nonspace = record.find_last_not_of(" \t\r\n");
                if (first != std::string::npos) record = record.substr(first, last_nonspace - first + 1);
                const auto last_comma = record.find(',', 12);
                if (record.size() < 15 || record.compare(0, 3, "OP,") != 0 || record[11] != ',' ||
                    last_comma == std::string::npos || last_comma == 12 || last_comma + 1 == record.size() ||
                    record.find(',', last_comma + 1) != std::string::npos || record.find("--------") != std::string::npos)
                {
                    throw std::runtime_error("Laser returned an invalid profile record");
                }

                for (std::size_t i = 12; i < record.size(); ++i)
                {
                    if (i == last_comma) continue;
                    if ((i == 12 || i == last_comma + 1) && (record[i] == '+' || record[i] == '-'))
                    {
                        if (i + 1 == last_comma || i + 1 == record.size())
                            throw std::runtime_error("Empty numeric profile field");
                        continue;
                    }
                    if (record[i] < '0' || record[i] > '9')
                        throw std::runtime_error("Malformed profile field");
                }
                // Match Robot1.cpp: each Keyence record stores the height at [3, 11).
                // Keep the sensor's units; no metre conversion is applied here.
                std::istringstream height(record.substr(3, 8));
                height.imbue(std::locale::classic());
                double distance;
                if (!(height >> distance) || !std::isfinite(distance) ||
                    !(height >> std::ws).eof())
                {
                    throw std::runtime_error("Laser returned a non-numeric profile height");
                }
                result.distances.push_back(distance);
            }

            if (!result.restoration_verified || result.actual_length_um == 0 ||
                result.actual_length_um > 1100 || result.actual_interval_um == 0 ||
                result.distances.size() != result.actual_length_um / result.actual_interval_um + 1)
                throw std::runtime_error("Profile geometry/count or restoration metadata is inconsistent");
            (void)remaining_ms();
            result.success = true;
        }
    }
    catch (const std::exception &e)
    {
        result.success = false;
        result.distances.clear();
        result.message = std::string("Laser line scan failed: ") + e.what();
    }

    if (outputs != nullptr)
    {
        UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);
    }
    for (auto &input : inputs)
    {
        UA_Variant_clear(&input);
    }

    return result;
}

namespace
{
struct MethodOutput
{
    std::size_t size = 0;
    UA_Variant *values = nullptr;
    ~MethodOutput() { if (values) UA_Array_delete(values, size, &UA_TYPES[UA_TYPES_VARIANT]); }
    void require(std::size_t index, std::size_t type) const
    {
        if (!values || index >= size || !UA_Variant_hasScalarType(&values[index], &UA_TYPES[type]))
            throw std::runtime_error("Unexpected laser measurement output schema; update server and ROS together");
    }
    std::string text(std::size_t index) const
    {
        require(index, UA_TYPES_STRING);
        const auto &s = *static_cast<const UA_String *>(values[index].data);
        if (s.length != 0 && !s.data) throw std::runtime_error("Invalid laser measurement string");
        return s.length == 0 ? std::string{} : std::string(reinterpret_cast<const char *>(s.data), s.length);
    }
};
LaserMeasurementState parse_measurement(const MethodOutput &output,
    std::chrono::steady_clock::time_point started)
{
    const std::size_t types[] = {UA_TYPES_DOUBLE, UA_TYPES_UINT32, UA_TYPES_DATETIME,
        UA_TYPES_UINT32, UA_TYPES_UINT32, UA_TYPES_BOOLEAN, UA_TYPES_STRING};
    for (std::size_t i = 0; i < 7; ++i) output.require(i, types[i]);
    LaserMeasurementState sample;
    sample.measurement = *static_cast<const UA_Double *>(output.values[0].data);
    sample.status = *static_cast<const UA_UInt32 *>(output.values[1].data);
    sample.source_timestamp = *static_cast<const UA_DateTime *>(output.values[2].data);
    sample.sequence = *static_cast<const UA_UInt32 *>(output.values[3].data);
    sample.age_ms = *static_cast<const UA_UInt32 *>(output.values[4].data);
    // Include the full round trip conservatively; Windows and ROS clocks need
    // not be synchronised for an age bound. Local delivery age is added in ROS.
    const auto elapsed = std::max<std::int64_t>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started).count());
    sample.age_ms = static_cast<UA_UInt32>(std::min<std::uint64_t>(0xFFFFFFFFu,
        static_cast<std::uint64_t>(sample.age_ms) + elapsed));
    sample.scanning = *static_cast<const UA_Boolean *>(output.values[5].data) != UA_FALSE;
    sample.message = output.text(6);
    if (sample.status == UA_STATUSCODE_GOOD &&
        (!std::isfinite(sample.measurement) || sample.sequence == 0 || sample.source_timestamp == 0 ||
         sample.measurement == -9998.99 || sample.measurement == -9999.99 || !sample.valid()))
    {
        sample.status = UA_STATUSCODE_BADNODATA;
        sample.message = "Inconsistent or stale measurement metadata";
    }
    if (!sample.valid()) sample.measurement = std::numeric_limits<double>::quiet_NaN();
    return sample;
}
}

LaserMeasurementState Skills::laser_measurement_state(unsigned int timeout_ms) const
{
    if (UA_NodeId_isNull(&laser_measurement_state_method))
        throw std::runtime_error("Server lacks GetLaserMeasurementState; update the Windows server");
    MethodOutput output;
    const auto started = std::chrono::steady_clock::now();
    m_client->call_method_once(m_folder, laser_measurement_state_method, 0, nullptr,
        &output.size, &output.values, timeout_ms);
    if (output.size != 7) throw std::runtime_error("Unexpected measurement state output count");
    return parse_measurement(output, started);
}

LaserMeasurementRequest Skills::request_laser_measurement(unsigned int timeout_ms, unsigned int call_timeout_ms) const
{
    if (timeout_ms == 0 || timeout_ms > 600000 || UA_NodeId_isNull(&request_laser_measurement_method))
        throw std::runtime_error("Fresh measurement requires updated server and timeout 1..600000 ms");
    UA_Variant input;
    UA_Variant_init(&input);
    MethodOutput output;
    // Borrow stack data, so this input needs no allocation cleanup.
    UA_Variant_setScalar(&input, &timeout_ms, &UA_TYPES[UA_TYPES_UINT32]);
    m_client->call_method_once(m_folder, request_laser_measurement_method, 1, &input,
        &output.size, &output.values, call_timeout_ms);
    if (output.size != 4) throw std::runtime_error("Unexpected measurement request output count");
    output.require(0, UA_TYPES_UINT32); output.require(1, UA_TYPES_DATETIME);
    output.require(2, UA_TYPES_BOOLEAN); output.require(3, UA_TYPES_STRING);
    if (*static_cast<const UA_Boolean *>(output.values[2].data) == UA_FALSE)
        throw std::runtime_error(output.text(3));
    LaserMeasurementRequest request;
    request.id = *static_cast<const UA_UInt32 *>(output.values[0].data);
    request.server_epoch = *static_cast<const UA_DateTime *>(output.values[1].data);
    if (request.id == 0 || request.server_epoch == 0) throw std::runtime_error("Invalid measurement request identity");
    return request;
}

LaserMeasurementState Skills::laser_measurement_result(
    const LaserMeasurementRequest &request, bool &done, unsigned int timeout_ms) const
{
    if (UA_NodeId_isNull(&laser_measurement_result_method))
        throw std::runtime_error("Server lacks GetLaserMeasurementResult; update the Windows server");
    UA_Variant inputs[2];
    UA_Variant_init(&inputs[0]); UA_Variant_init(&inputs[1]);
    UA_Variant_setScalar(&inputs[0], const_cast<UA_UInt32 *>(&request.id), &UA_TYPES[UA_TYPES_UINT32]);
    UA_Variant_setScalar(&inputs[1], const_cast<UA_DateTime *>(&request.server_epoch), &UA_TYPES[UA_TYPES_DATETIME]);
    MethodOutput output;
    const auto started = std::chrono::steady_clock::now();
    m_client->call_method_once(m_folder, laser_measurement_result_method, 2, inputs,
        &output.size, &output.values, timeout_ms);
    if (output.size != 8) throw std::runtime_error("Unexpected measurement result output count");
    output.require(7, UA_TYPES_BOOLEAN);
    done = *static_cast<const UA_Boolean *>(output.values[7].data) != UA_FALSE;
    return parse_measurement(output, started);
}

ForceSensingMoveResult Skills::force_sensing_move(
    int start_x, int start_y, int start_z, int start_t, int target_x, int target_y, int target_z,
    float max_fx, float max_fy, float max_fz, unsigned int step_size, std::string endpoint
) const
{
    std::array<UA_Variant, 11> inputs = {
        make_variant(start_x),
        make_variant(start_y),
        make_variant(start_z),
        make_variant(start_t),
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
        &outputs,
        endpoint
    );

    const UA_String *error = static_cast<UA_String *>(outputs[0].data);
    const std::string error_str(reinterpret_cast<char *>(error->data), error->length);

    const bool threshold_exceeded = *static_cast<UA_Boolean *>(outputs[1].data);

    UA_Array_delete(outputs, output_size, &UA_TYPES[UA_TYPES_VARIANT]);

    for (auto &input : inputs)
    {
        UA_Variant_clear(&input);
    }

    return ForceSensingMoveResult{error_str.empty(), error_str, threshold_exceeded};
}

} // namespace PMClient
