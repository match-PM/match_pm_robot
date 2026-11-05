#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using BOOL = int;
using LONG = std::int32_t;
using DWORD = std::uint32_t;
using ULONGLONG = std::uint64_t;
using CLong = int;
using CBool = bool;
using VOID = void;
constexpr BOOL TRUE = 1;
constexpr BOOL FALSE = 0;
extern std::atomic<DWORD> fake_clock;
inline DWORD GetTickCount() { return fake_clock.load(); }
inline ULONGLONG GetTickCount64() { return fake_clock.load(); }
inline void Wait(int ms) { fake_clock.fetch_add(ms); std::this_thread::yield(); }
using CRITICAL_SECTION = std::recursive_mutex;
inline void InitializeCriticalSection(CRITICAL_SECTION *) {}
inline void DeleteCriticalSection(CRITICAL_SECTION *) {}
inline void EnterCriticalSection(CRITICAL_SECTION *mutex) { mutex->lock(); }
inline void LeaveCriticalSection(CRITICAL_SECTION *mutex) { mutex->unlock(); }
inline LONG InterlockedExchange(volatile LONG *value, LONG desired)
{ return __atomic_exchange_n(value, desired, __ATOMIC_SEQ_CST); }
inline LONG InterlockedCompareExchange(volatile LONG *value, LONG desired, LONG expected)
{ __atomic_compare_exchange_n(value, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST); return expected; }
class CString
{
    std::string text;
public:
    CString() = default;
    CString(const char *value) : text(value) {}
    CString(const std::string &value) : text(value) {}
    CString(int value) : text(std::to_string(value)) {}
    int GetLength() const { return text.size(); }
    char operator[](int index) const { return text.at(index); }
    CString Right(int count) const { return text.substr(text.size() > size_t(count) ? text.size() - count : 0); }
    operator std::string() const { return text; }
    operator const char *() const { return text.c_str(); }
    friend std::string operator+(const CString &left, const char *right) { return left.text + right; }
    friend std::string operator+(const CString &left, const CString &right) { return left.text + right.text; }
    friend std::string operator+(const std::string &left, const CString &right) { return left + right.text; }
    bool operator==(const char *right) const { return text == right; }
    bool operator!=(const char *right) const { return text != right; }
};
class CDownTimer
{
    DWORD start;
    DWORD duration;
public:
    explicit CDownTimer(int ms = 0) : start(GetTickCount()), duration(ms) {}
    void SetTime(int ms) { start = GetTickCount(); duration = ms; }
    bool GetElapsed() const { return GetTickCount() - start >= duration; }
};
