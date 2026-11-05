#include "StdAfx.h"
#include "Robot1.h"
#include "LaserScan.h"
#include "pm_client/client.hpp"
#include <iostream>
#include <cmath>
#include <limits>
#include <map>

std::atomic<DWORD> fake_clock{10000};
FakeRobot Robot1;
UA_Server *server = nullptr;
UA_NodeId folder;
UA_NodeId start_method;
UA_NodeId result_method;
UA_NodeId measurement_state_method, request_measurement_method, measurement_result_method;
std::string tampered_profile;
int poll_delay_ms = 0;
int measurement_reply_delay_ms = 0;
int checks = 0;
int starts = 0;
int polls = 0;
bool auto_work = true;
bool never_complete = false;
bool lose_start_reply = false;

void Check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
    ++checks;
}

struct Reply
{
    UA_CallMethodResult value;
    Reply(UA_NodeId method, size_t count, UA_Variant *inputs)
    {
        UA_CallMethodRequest request;
        UA_CallMethodRequest_init(&request);
        request.objectId = folder;
        request.methodId = method;
        request.inputArgumentsSize = count;
        request.inputArguments = inputs;
        value = UA_Server_call(server, &request);
        Check(value.statusCode == UA_STATUSCODE_GOOD, "Method invocation failed");
    }
    ~Reply() { UA_CallMethodResult_clear(&value); }
};

UA_NodeId FindMethod(const char *name)
{
    UA_BrowseDescription description;
    UA_BrowseDescription_init(&description);
    description.nodeId = folder;
    description.browseDirection = UA_BROWSEDIRECTION_FORWARD;
    description.resultMask = UA_BROWSERESULTMASK_ALL;
    UA_BrowseResult result = UA_Server_browse(server, 0, &description);
    UA_NodeId found = UA_NODEID_NULL;
    for (size_t i = 0; i < result.referencesSize; ++i)
    {
        const auto &reference = result.references[i];
        const std::string browse_name(reinterpret_cast<char *>(reference.browseName.name.data),
                                      reference.browseName.name.length);
        if (browse_name == name) UA_NodeId_copy(&reference.nodeId.nodeId, &found);
    }
    UA_BrowseResult_clear(&result);
    Check(!UA_NodeId_isNull(&found), "Method was not registered");
    return found;
}

UA_UInt32 Start(unsigned interval = 10, unsigned length = 100)
{
    UA_Variant input[2];
    UA_Variant_init(&input[0]); UA_Variant_init(&input[1]);
    UA_Variant_setScalar(&input[0], &interval, &UA_TYPES[UA_TYPES_UINT32]);
    UA_Variant_setScalar(&input[1], &length, &UA_TYPES[UA_TYPES_UINT32]);
    Reply reply(start_method, 2, input);
    Check(*static_cast<UA_Boolean *>(reply.value.outputArguments[1].data), "Start rejected");
    return *static_cast<UA_UInt32 *>(reply.value.outputArguments[0].data);
}

bool Result(UA_UInt32 id, bool expected_success)
{
    UA_Variant input;
    UA_Variant_init(&input);
    UA_Variant_setScalar(&input, &id, &UA_TYPES[UA_TYPES_UINT32]);
    Reply reply(result_method, 1, &input);
    Check(*static_cast<UA_Boolean *>(reply.value.outputArguments[2].data) == expected_success,
          "Unexpected result success flag");
    return *static_cast<UA_Boolean *>(reply.value.outputArguments[3].data);
}

namespace PMClient
{
Client::Client() : m_client(nullptr) {}
Client::~Client() = default;
void Client::call_method(UA_NodeId, UA_NodeId, size_t, UA_Variant *, size_t *, UA_Variant **, std::string)
{ throw std::runtime_error("Automatic retries are forbidden in scan path"); }
void Client::call_method_once(UA_NodeId object, UA_NodeId method, size_t count,
    UA_Variant *inputs, size_t *size, UA_Variant **outputs, UA_UInt32 timeout_ms)
{
    (void)timeout_ms;
    Check(UA_NodeId_equal(&object, &folder), "Wrong object node");
    if (UA_NodeId_equal(&method, &start_method))
    {
        ++starts;
        Check(count == 2 && UA_Variant_hasScalarType(&inputs[0], &UA_TYPES[UA_TYPES_UINT32]) &&
              UA_Variant_hasScalarType(&inputs[1], &UA_TYPES[UA_TYPES_UINT32]), "Incorrect input types");
    }
    else if (UA_NodeId_equal(&method, &result_method))
    {
        ++polls;
        Check(count == 1, "Wrong polling request");
        if (poll_delay_ms != 0) std::this_thread::sleep_for(std::chrono::milliseconds(poll_delay_ms));
        if (auto_work && !never_complete) OPCUAServer::LaserScan::ProcessWorker();
    }
    else if (UA_NodeId_equal(&method, &measurement_result_method))
    {
        Check(count == 2, "Measurement polling must carry request ID and server epoch");
        if (auto_work && !never_complete) OPCUAServer::LaserScan::ProcessWorker();
    }
    else Check(UA_NodeId_equal(&method, &measurement_state_method) ||
        UA_NodeId_equal(&method, &request_measurement_method), "Unknown measurement method");
    Reply reply(method, count, inputs);
    if (measurement_reply_delay_ms != 0 && UA_NodeId_equal(&method, &measurement_state_method))
        std::this_thread::sleep_for(std::chrono::milliseconds(measurement_reply_delay_ms));
    if (!tampered_profile.empty() && UA_NodeId_equal(&method, &result_method))
    {
        UA_Variant_clear(&reply.value.outputArguments[0]);
        UA_String text{tampered_profile.size(), reinterpret_cast<UA_Byte *>(&tampered_profile[0])};
        UA_Variant_setScalarCopy(&reply.value.outputArguments[0], &text, &UA_TYPES[UA_TYPES_STRING]);
    }
    if (lose_start_reply && UA_NodeId_equal(&method, &start_method))
        throw std::runtime_error("BadConnectionClosed");
    *size = reply.value.outputArgumentsSize;
    Check(UA_Array_copy(reply.value.outputArguments, *size, reinterpret_cast<void **>(outputs),
                       &UA_TYPES[UA_TYPES_VARIANT]) == UA_STATUSCODE_GOOD, "Output copy failed");
}
}

std::string Profile(unsigned count, const char *height = "+0001.25")
{
    std::string data;
    for (unsigned i = 0; i < count; ++i) data += std::string("OP,") + height + ",-0001,010;";
    return data;
}

int main()
{
    server = UA_Server_new();
    unsigned ns = UA_Server_addNamespace(server, "TestLaser");
    UA_ObjectAttributes attributes = UA_ObjectAttributes_default;
    Check(UA_Server_addObjectNode(server, UA_NODEID_NUMERIC(ns, 9000),
        UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER), UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(ns, const_cast<char *>("Skills")), UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
        attributes, nullptr, &folder) == UA_STATUSCODE_GOOD, "Cannot create folder");
    OPCUAServer::LaserScan::Register(server, folder);
    start_method = FindMethod("StartLaserLineScan");
    result_method = FindMethod("GetLaserLineScanResult");
    measurement_state_method = FindMethod("GetLaserMeasurementState");
    request_measurement_method = FindMethod("RequestLaserMeasurement");
    measurement_result_method = FindMethod("GetLaserMeasurementResult");
    OPCUAServer::LaserScan::SetWorkerReady();
    PMClient::Client client;
    PMClient::Skills skills(&client, folder);
    skills.start_laser_line_scan_method = start_method;
    skills.get_laser_line_scan_result_method = result_method;
    skills.laser_measurement_state_method = measurement_state_method;
    skills.request_laser_measurement_method = request_measurement_method;
    skills.laser_measurement_result_method = measurement_result_method;
    const auto initial = skills.laser_measurement_state();
    Check(!initial.valid() && std::isnan(initial.measurement) && initial.source_timestamp == 0 &&
        initial.sequence == 0, "Initial cache must not advertise a valid measurement");

    Robot1.Keyence.profiles = {Profile(13)};
    auto response = skills.laser_line_scan(10, 100, "unused");
    Check(response.success && response.distances.size() == 13 && response.distances[0] == 1.25,
          "End-to-end valid scan failed");
    Check(Robot1.Keyence.program == "4", "Program was not restored");
    Check(response.actual_length_um == 120 && response.actual_interval_um == 10 &&
        response.restoration_verified, "Structured actual geometry/restoration missing");
    Check(Robot1.Keyence.actual_length == 120 && Robot1.Keyence.actual_interval == 10, "Incorrect settings");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(111, "--------")};
    response = skills.laser_line_scan(30, 1000, "unused");
    Check(!response.success && response.distances.empty(), "Invalid profile was accepted");
    Check(Robot1.Keyence.reads == 3 && Robot1.Keyence.program == "4", "Retry/restore failed");
    Check(response.message.find("records=111, invalid_marker=yes") != std::string::npos,
          "Missing raw diagnostics");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13, "--------"), Profile(13)};
    response = skills.laser_line_scan(10, 100, "unused");
    Check(response.success && Robot1.Keyence.reads == 2, "Retry did not recover transient invalid reading");

    Robot1.Keyence.Reset();
    Robot1.Keyence.throw_read = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.program == "4", "Exception restoration failed");

    Robot1.Keyence.Reset();
    Robot1.Keyence.wrong_width = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.reads == 0 && Robot1.Keyence.program == "4", "Readback not checked");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    Robot1.Keyence.reply_suffix = "\r\n";
    response = skills.laser_line_scan(10, 100, "unused");
    Check(response.success && Robot1.Keyence.program == "4", "CR/LF-terminated readback was rejected");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    Robot1.Keyence.padded_numbers = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(response.success, "Zero-padded program number was rejected");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    Robot1.Keyence.wrong_selection = true;
    Robot1.Keyence.wrong_restoration = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.reads == 0, "Program 15 was mistaken for program 5");
    Check(response.message.find("select laser program 5") != std::string::npos &&
          response.message.find("restoration of laser program 4") != std::string::npos,
          "Restoration failure overwrote the primary failure");
    Check(response.message.find("PR,15") != std::string::npos, "Actual program reply was omitted");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    Robot1.Keyence.empty_program_reply = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.reads == 0, "Empty program readback was accepted");
    Check(response.message.find("PR reply=[]") != std::string::npos &&
          response.message.find("PW,5 reply=[PW]") != std::string::npos,
          "Empty readback/write acknowledgement was not diagnosed");

    Robot1.Keyence.Reset();
    Robot1.Keyence.empty_write_reply = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.reads == 0 && Robot1.Keyence.program == "4",
          "Missing write acknowledgement did not fail/restore");
    Check(response.message.find("PW,5 reply=[]") != std::string::npos,
          "Missing write acknowledgement was not diagnosed");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    Robot1.Keyence.wrong_restoration = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && response.distances.empty() && Robot1.Keyence.reads == 1,
          "Unverified restoration was reported as scan success");
    Check(response.message.find("PW,4 reply=[PW]") != std::string::npos &&
          response.message.find("PR reply=[PR,5]") != std::string::npos,
          "Restoration readback diagnostics missing");

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    Robot1.Keyence.delayed_restoration = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(response.success && Robot1.Keyence.restoration_reads == 3,
          "Bounded readback polling did not allow delayed program change");
    Check(std::count(Robot1.Keyence.commands.begin(), Robot1.Keyence.commands.end(), "PW,4") == 1,
          "Readback polling replayed the program change");

    Robot1.Keyence.Reset();
    Robot1.Keyence.multiple_frames = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.reads == 0, "Multiple response frames were accepted");
    Check(response.message.find("\\r\\n") != std::string::npos,
          "Diagnostic control characters were not escaped");

    Robot1.Keyence.Reset();
    Robot1.Keyence.machine_error = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.commands.empty() && Robot1.Keyence.program_reads == 0,
          "Scan accessed the sensor while a machine error was active");
    Check(response.message.find("Active machine error") != std::string::npos,
          "Active machine error was not diagnosed");

    Robot1.Keyence.Reset();
    Robot1.Keyence.disabled = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.commands.empty(), "Scan used disabled serial communication");
    Check(response.message.find("disabled") != std::string::npos, "Disabled serial communication was not diagnosed");

    Robot1.Keyence.Reset();
    Robot1.Keyence.error_after_selection = true;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && Robot1.Keyence.reads == 0 && Robot1.Keyence.program == "5",
          "Active error was bypassed or program restoration was fabricated");
    Check(Robot1.Keyence.machine_error && Robot1.Keyence.commands.size() == 1,
          "Acquisition cleared a machine error or bypassed the command block");
    Check(response.message.find("PW,4 not sent: Active machine error") != std::string::npos,
          "Blocked restoration was not explicitly reported");

    for (const auto &malformed : {Profile(13, "+0001.250"),
        std::string("XX,+0001.25,-0001,010;"), std::string("OP,+0001.25,-0001,01x;")})
    {
        Robot1.Keyence.Reset();
        Robot1.Keyence.profiles = {malformed};
        response = skills.laser_line_scan(10, 100, "unused");
        Check(!response.success && response.distances.empty(), "Malformed profile accepted by server");
    }
    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    tampered_profile = Profile(13, "+0001.250");
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && response.distances.empty(), "Client accepted an overlong height from the server");
    tampered_profile.clear();
    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    poll_delay_ms = 30;
    response = skills.laser_line_scan(10, 100, "unused", 5);
    Check(!response.success && response.distances.empty() && response.message.find("Timed out") != std::string::npos,
        "Terminal success received after deadline was accepted");
    poll_delay_ms = 0;

    Robot1.Keyence.Reset();
    Robot1.Keyence.profiles = {Profile(13)};
    auto id = Start();
    Check(!Result(id, false), "Queued scan should be pending");
    Check(Result(id + 100, false), "Unknown scan ID should be a terminal failure");
    std::thread worker([] { OPCUAServer::LaserScan::ProcessWorker(); });
    worker.join();
    {
        unsigned interval = 10, length = 100;
        UA_Variant input[2];
        UA_Variant_init(&input[0]); UA_Variant_init(&input[1]);
        UA_Variant_setScalar(&input[0], &interval, &UA_TYPES[UA_TYPES_UINT32]);
        UA_Variant_setScalar(&input[1], &length, &UA_TYPES[UA_TYPES_UINT32]);
        Reply rejected(start_method, 2, input);
        Check(!*static_cast<UA_Boolean *>(rejected.value.outputArguments[1].data),
              "Uncollected result was overwritten by another job");
    }
    Check(Result(id, true), "Completed scan missing");

    Robot1.Keyence.Reset();
    Robot1.Keyence.block = true;
    Robot1.Keyence.profiles = {Profile(13)};
    id = Start();
    std::thread blocked_worker([] { OPCUAServer::LaserScan::ProcessWorker(); });
    {
        std::unique_lock<std::mutex> lock(Robot1.Keyence.mutex);
        Robot1.Keyence.condition.wait(lock, [] { return Robot1.Keyence.entered; });
    }
    const auto started = std::chrono::steady_clock::now();
    auto_work = false;
    const auto fresh_during_scan = skills.request_laser_measurement(120000);
    bool fresh_done = false;
    const auto waiting = skills.laser_measurement_result(fresh_during_scan, fresh_done);
    Check(!fresh_done && !waiting.valid() && waiting.scanning,
        "Measurement during profile must remain pending and unavailable");
    Check(!Result(id, false), "Blocking sensor should leave result pending");
    UA_DateTime timestamp;
    OPCUAServer::LaserScan::ReadCachedMeasurement(timestamp);
    Check(std::chrono::steady_clock::now() - started < std::chrono::milliseconds(100),
          "Network/cache callback waited on the sensor");
    Check(Robot1.Keyence.scalar_reads == 0, "Cached read accessed serial port during scan");
    {
        unsigned interval = 10, length = 100;
        UA_Variant input[2];
        UA_Variant_init(&input[0]); UA_Variant_init(&input[1]);
        UA_Variant_setScalar(&input[0], &interval, &UA_TYPES[UA_TYPES_UINT32]);
        UA_Variant_setScalar(&input[1], &length, &UA_TYPES[UA_TYPES_UINT32]);
        Reply rejected(start_method, 2, input);
        Check(!*static_cast<UA_Boolean *>(rejected.value.outputArguments[1].data),
              "Simultaneous acquisition was accepted");
    }
    {
        std::lock_guard<std::mutex> lock(Robot1.Keyence.mutex);
        Robot1.Keyence.block = false;
    }
    Robot1.Keyence.condition.notify_all();
    blocked_worker.join();
    Check(Result(id, true), "Blocked scan did not complete after sensor was released");

    // More than one second has elapsed in the scan; do not renew read demand.
    OPCUAServer::LaserScan::ProcessWorker();
    const auto post_scan = skills.laser_measurement_result(fresh_during_scan, fresh_done);
    Check(fresh_done && post_scan.valid() && post_scan.measurement == 42.25 && post_scan.sequence != 0,
        "Fresh request expired with the old one-second cache demand");
    OPCUAServer::LaserScan::ReadCachedMeasurement(timestamp);
    OPCUAServer::LaserScan::ProcessWorker();
    const double cached = OPCUAServer::LaserScan::ReadCachedMeasurement(timestamp);
    Check(cached == 42.25 && timestamp != 0 && Robot1.Keyence.scalar_reads == 1,
          "Normal scalar measurement was not refreshed by the worker");

    const auto good = skills.laser_measurement_state();
    fake_clock.fetch_add(501);
    const auto stale = skills.laser_measurement_state();
    Check(!stale.valid() && stale.status == UA_STATUSCODE_BADNODATA, "Stale cache advertised Good");
    Robot1.Keyence.throw_scalar = true;
    OPCUAServer::LaserScan::ProcessWorker();
    const auto failed = skills.laser_measurement_state();
    Check(!failed.valid() && std::isnan(failed.measurement) && failed.sequence == good.sequence &&
        failed.source_timestamp == good.source_timestamp, "Failed refresh fabricated a valid sample or source time");
    Robot1.Keyence.throw_scalar = false;
    Robot1.Keyence.scalar_value = -9998.99;
    const auto invalid_request = skills.request_laser_measurement(10000);
    OPCUAServer::LaserScan::ProcessWorker();
    const auto invalid = skills.laser_measurement_result(invalid_request, fresh_done);
    Check(fresh_done && !invalid.valid(), "Invalid driver sentinel satisfied a fresh request");
    Robot1.Keyence.scalar_value = 42.25;

    const auto expires = skills.request_laser_measurement(20);
    fake_clock.fetch_add(21);
    const auto expired = skills.laser_measurement_result(expires, fresh_done);
    Check(fresh_done && expired.status == UA_STATUSCODE_BADTIMEOUT, "Server refresh demand did not expire");

    const auto abandoned = skills.request_laser_measurement(20);
    fake_clock.fetch_add(21);
    const auto replacement = skills.request_laser_measurement(10000);
    Check(replacement.id != abandoned.id, "Expired uncollected request prevented a new measurement");
    OPCUAServer::LaserScan::ProcessWorker();
    Check(skills.laser_measurement_result(replacement, fresh_done).valid() && fresh_done,
        "Replacement for abandoned request did not complete");

    measurement_reply_delay_ms = 550;
    const auto delayed_sample = skills.laser_measurement_state();
    Check(!delayed_sample.valid() && delayed_sample.age_ms >= 500,
        "Network delivery delay was omitted from measurement age");
    measurement_reply_delay_ms = 0;

    // An acquisition already in progress before admission cannot satisfy a new request.
    fake_clock.fetch_add(100);
    (void)skills.laser_measurement_state();
    Robot1.Keyence.block_scalar = true;
    Robot1.Keyence.scalar_entered = false;
    std::thread old_acquisition([] { OPCUAServer::LaserScan::ProcessWorker(); });
    {
        std::unique_lock<std::mutex> lock(Robot1.Keyence.mutex);
        Robot1.Keyence.condition.wait(lock, [] { return Robot1.Keyence.scalar_entered; });
    }
    const auto after_start = skills.request_laser_measurement(10000);
    {
        std::lock_guard<std::mutex> lock(Robot1.Keyence.mutex);
        Robot1.Keyence.block_scalar = false;
    }
    Robot1.Keyence.condition.notify_all();
    old_acquisition.join();
    const auto still_waiting = skills.laser_measurement_result(after_start, fresh_done);
    Check(!fresh_done && !still_waiting.valid(), "An acquisition started before the request was returned as fresh");
    OPCUAServer::LaserScan::ProcessWorker();
    const auto newest = skills.laser_measurement_result(after_start, fresh_done);
    Check(fresh_done && newest.valid(), "New acquisition did not satisfy pending request");
    auto wrong_epoch = after_start;
    --wrong_epoch.server_epoch;
    const auto unknown = skills.laser_measurement_result(wrong_epoch, fresh_done);
    Check(fresh_done && unknown.status == UA_STATUSCODE_BADNOTFOUND, "Server epoch was not checked");
    Robot1.Keyence.wrong_restoration = true;
    const auto unverified = skills.request_laser_measurement(10000);
    const auto before_scalar = Robot1.Keyence.scalar_reads;
    OPCUAServer::LaserScan::ProcessWorker();
    const auto unsafe = skills.laser_measurement_result(unverified, fresh_done);
    Check(fresh_done && !unsafe.valid() && Robot1.Keyence.scalar_reads == before_scalar,
        "Scalar acquisition used an unverified ordinary program");
    Robot1.Keyence.wrong_restoration = false;
    auto_work = true;

    const auto old_starts = starts;
    Check(!skills.laser_line_scan(0, 100, "unused").success, "Zero interval accepted");
    Check(!skills.laser_line_scan(10, 1101, "unused").success, "Oversize length accepted");
    Check(starts == old_starts, "Invalid inputs were sent to server");
    skills.start_laser_line_scan_method = UA_NODEID_NULL;
    Check(!skills.laser_line_scan(10, 100, "unused").success, "Old server was accepted");
    Check(starts == old_starts, "Blocking fallback was used");
    skills.start_laser_line_scan_method = start_method;
    never_complete = true;
    response = skills.laser_line_scan(10, 100, "unused", 2);
    Check(!response.success && response.message.find("Timed out") != std::string::npos, "Client timeout failed");
    never_complete = false;
    OPCUAServer::LaserScan::ProcessWorker();
    Check(Result(id + 1, true), "Timed-out client job was not retained");

    lose_start_reply = true;
    const int before = starts;
    response = skills.laser_line_scan(10, 100, "unused");
    Check(!response.success && starts == before + 1, "Lost start reply caused automatic replay");
    OPCUAServer::LaserScan::ProcessWorker();

    UA_NodeId_clear(&measurement_state_method); UA_NodeId_clear(&request_measurement_method);
    UA_NodeId_clear(&measurement_result_method);
    UA_NodeId_clear(&start_method); UA_NodeId_clear(&result_method); UA_NodeId_clear(&folder);
    UA_Server_delete(server);
    std::cout << checks << " protocol, parsing, retry and responsiveness checks passed\n";
}
