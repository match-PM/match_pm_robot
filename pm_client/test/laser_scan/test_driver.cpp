#include "StdAfx.h"
#include <deque>
#include <iostream>
std::atomic<DWORD> fake_clock{10000};
enum { ackNone = 0, ackOk = 1, ackRetry = 2, AnswersAbort = 3,
       AnswersRetryAbort = 4, AutoAcknowledgeNone = 5 };
constexpr char CHAR_CR = '\r';
struct EAbort {};
bool machine_error = false;
bool is_task = true;
inline bool GetErrorListing() { return machine_error; }
inline bool IsCTask() { return is_task; }
struct FakeTask { bool Task_GetInterruptRequest() { return false; } };
inline FakeTask &GetCurrentTask() { static FakeTask task; return task; }
struct CTextLines { template<class T> void Add(const T &) {} };
inline CString Intl(int, const char *value) { return value; }
inline CString Str(int value) { return std::to_string(value); }
inline int CalculateApplicationErrorCode(int, int) { return 0; }
template<class... T> int CmpSetError(const T &...) { return ackOk; }
struct FakeMaster { bool GetInAuto() { return false; } } Master;
struct OPERATION_NAME { explicit OPERATION_NAME(const char *) {} };
int waits = 0;
inline bool WaitCondition(bool done)
{
    if (!done && ++waits > 1000) throw std::runtime_error("Serial lock was not released");
    return done;
}
#define waiting(condition) while (!WaitCondition(condition))

struct FakeSerial
{
    struct { bool Enable = true; } Parameters;
    struct Port
    {
        int GetComPortNumber() { return 1; }
        CString GetErrorMessage() { return "fake"; }
        void Reset() {}
    } SerialPort;
    std::deque<std::string> responses;
    int reads = 0;
    int throw_at = 0;
    int elapsed_per_read = 0;
    int sends = 0;
    void SendOrder(char *) { ++sends; }
    CString WaitAnswerReceived(int, int &error)
    {
        ++reads;
        if (reads == throw_at) throw std::runtime_error("Receive failed");
        fake_clock.fetch_add(elapsed_per_read);
        if (responses.empty()) { fake_clock.fetch_add(300); error = -1; return ""; }
        std::string response = responses.front(); responses.pop_front();
        error = response.empty() ? -1 : 0;
        if (response.empty()) fake_clock.fetch_add(300);
        return response;
    }
};

namespace KeyenceSensorHead_
{
volatile LONG SemaphoreSendReceive = TRUE;
BOOL IsError = FALSE;
CDownTimer ProfileMeasureTimer;
CDownTimer DTIBetweenTwoOrder;
class CKeyenceDetector
{
public:
    FakeSerial *RS232Com;
    mutable bool IsComError = false;
    int FTaskErrorCode = 0;
    explicit CKeyenceDetector(FakeSerial &serial) : RS232Com(&serial) {}
    std::string GetMeasureProfile(CLong, CLong);
    CString SendCommand(CLong, CString, CString) const;
    CString GetCommunicationStatus() const;
    void WaitReady() const {}
    CString GetName() const { return "FakeKeyence"; }
    int CheckAnswerReceived(CString, int, int &) const { return ackOk; }
};
}

// These are extracted unchanged from the staged production source at test time.
#include "driver_under_test.inc"

int checks = 0;
void Check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
    ++checks;
}

int main()
{
    using namespace KeyenceSensorHead_;
    FakeSerial serial;
    CKeyenceDetector detector(serial);
    serial.responses = {"OP,+0001.25,00000,010", "OP,+0002.50,00001,010", "OP,+0003.75,00002,010", "UNRELATED"};
    auto profile = detector.GetMeasureProfile(3, 1);
    Check(serial.reads == 3 && serial.responses.front() == "UNRELATED", "Reader consumed one record too many");
    Check(std::count(profile.begin(), profile.end(), ';') == 3, "Incorrect profile record count");
    Check(SemaphoreSendReceive == TRUE, "Success did not release serial ownership");

    serial.responses.clear(); serial.reads = 0;
    Check(detector.GetMeasureProfile(3, 1).empty(), "First read timeout was ignored");
    Check(SemaphoreSendReceive == TRUE, "First read timeout leaked ownership");

    serial.responses = {"OP,+0001.25,00000,010"}; serial.reads = 0;
    Check(detector.GetMeasureProfile(3, 1).empty(), "Mid-profile timeout was ignored");
    Check(SemaphoreSendReceive == TRUE, "Mid-profile timeout leaked ownership");

    serial.responses = {"OP,+0001.25,00000,010", "OP,+0002.50,00001,010"};
    serial.reads = 0; serial.throw_at = 2;
    bool threw = false;
    try { detector.GetMeasureProfile(3, 1); } catch (const std::runtime_error &) { threw = true; }
    Check(threw && SemaphoreSendReceive == TRUE, "Exception leaked ownership");

    serial.throw_at = 0; serial.reads = 0;
    serial.responses = {"OP,+0001.25,00000,010"};
    Check(!detector.GetMeasureProfile(1, 1).empty() && serial.reads == 1, "Single-point read/recovery failed");
    int previous = serial.reads;
    Check(detector.GetMeasureProfile(0, 1).empty() && detector.GetMeasureProfile(1, 0).empty(), "Invalid parameters accepted");
    Check(serial.reads == previous, "Invalid parameters accessed the serial port");

    serial.responses.assign(550, "OP,+0001.25,00000,010");
    serial.reads = 0; serial.elapsed_per_read = 100;
    Check(detector.GetMeasureProfile(1100, 2).empty(), "Total receive deadline was not enforced");
    Check(serial.reads <= 300 && SemaphoreSendReceive == TRUE, "Receive deadline leaked ownership");

    serial.responses = {"PR,4"}; serial.reads = 0; serial.elapsed_per_read = 0;
    Check(detector.SendCommand(17, "", "PR") == "PR,4", "Command reply was lost");
    Check(SemaphoreSendReceive == TRUE, "Command success leaked serial ownership");

    serial.throw_at = 1; serial.reads = 0; threw = false;
    try { detector.SendCommand(17, "", "PR"); } catch (const std::runtime_error &) { threw = true; }
    Check(threw && SemaphoreSendReceive == TRUE, "Command exception leaked ownership");
    serial.throw_at = 0;

    machine_error = true;
    Check(detector.GetCommunicationStatus() == "Active machine error blocks laser communication",
          "Machine error status was not exposed");
    const int sends = serial.sends;
    Check(detector.SendCommand(17, "", "PW,5") == "" && serial.sends == sends,
          "Command bypassed an active machine error");
    Check(detector.GetMeasureProfile(3, 1).empty() && serial.sends == sends,
          "Profile acquisition bypassed an active machine error");
    Check(SemaphoreSendReceive == TRUE && machine_error, "Machine error bypass/abort leaked ownership");
    machine_error = false;

    serial.Parameters.Enable = false;
    Check(detector.GetCommunicationStatus() == "Laser RS232 communication is disabled",
          "Disabled serial port was not exposed");
    detector.SendCommand(17, "", "PW,5");
    Check(serial.sends == sends, "Command used a disabled serial port");
    serial.Parameters.Enable = true;

    detector.IsComError = true; is_task = false;
    Check(detector.GetCommunicationStatus() == "Laser communication is in an error state",
          "Non-task communication error was not exposed");
    Check(detector.SendCommand(17, "", "PR") == "" && serial.sends == sends,
          "Command bypassed a non-task communication error");
    detector.IsComError = false; is_task = true;
    Check(detector.GetCommunicationStatus() == "", "Healthy communication was reported blocked");
    std::cout << checks << " production serial-command/profile-reader checks passed\n";
}
