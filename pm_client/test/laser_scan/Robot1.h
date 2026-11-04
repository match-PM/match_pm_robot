#pragma once
#include "StdAfx.h"
struct FakeKeyence
{
    std::mutex mutex;
    std::condition_variable condition;
    bool block = false;
    bool entered = false;
    bool throw_read = false;
    bool wrong_width = false;
    bool machine_error = false;
    bool disabled = false;
    bool empty_write_reply = false;
    bool empty_program_reply = false;
    bool wrong_selection = false;
    bool wrong_restoration = false;
    bool delayed_restoration = false;
    bool multiple_frames = false;
    bool padded_numbers = false;
    bool error_after_selection = false;
    std::string reply_suffix;
    int program_reads = 0;
    int restoration_reads = 0;
    int reads = 0;
    int scalar_reads = 0;
    int actual_length = 0;
    int actual_interval = 0;
    std::string program = "4";
    std::string setting;
    std::vector<std::string> profiles;
    std::vector<std::string> commands;
    void Reset()
    {
        block = entered = throw_read = wrong_width = false;
        machine_error = disabled = empty_write_reply = empty_program_reply = false;
        wrong_selection = wrong_restoration = delayed_restoration = multiple_frames = false;
        padded_numbers = error_after_selection = false;
        reply_suffix.clear();
        program_reads = restoration_reads = 0;
        reads = scalar_reads = 0;
        profiles.clear(); commands.clear(); program = "4";
    }
    CString GetCommunicationStatus() const
    {
        if (disabled) return "Laser RS232 communication is disabled";
        if (machine_error) return "Active machine error blocks laser communication";
        return "";
    }
    void SetLaser(CString value)
    {
        std::string command = value;
        commands.push_back(command);
        if (command.rfind("PW,", 0) == 0) program = command.substr(3);
        if (command == "PW,5" && error_after_selection) machine_error = true;
        if (command.rfind("SD,SC,", 0) == 0) setting = command.substr(6);
    }
    CString CheckLaser(CString value)
    {
        std::string command = value;
        if (command.rfind("PW,", 0) == 0 || command.rfind("SD,", 0) == 0)
        {
            SetLaser(value);
            return empty_write_reply ? "" : command.substr(0, 2) + reply_suffix;
        }
        if (command == "PR")
        {
            ++program_reads;
            if (empty_program_reply) return "";
            std::string answer = program;
            if (program == "5" && wrong_selection) answer = "15";
            if (program == "4")
            {
                ++restoration_reads;
                if (wrong_restoration || (delayed_restoration && restoration_reads < 3)) answer = "5";
            }
            if (multiple_frames) return "PR,15\r\nPR," + answer;
            if (padded_numbers) answer = "0" + answer;
            return "PR," + answer + reply_suffix;
        }
        if (command == "SR,SC") return "SR,SC," + (wrong_width ? "X" : setting) + reply_suffix;
        throw std::runtime_error("Unexpected query");
    }
    std::string GetMeasureProfile(int length, int interval)
    {
        std::unique_lock<std::mutex> lock(mutex);
        entered = true;
        condition.notify_all();
        condition.wait(lock, [&] { return !block; });
        actual_length = length; actual_interval = interval;
        ++reads;
        if (throw_read) throw std::runtime_error("Fake sensor exception");
        return profiles.empty() ? "" : profiles[std::min(size_t(reads - 1), profiles.size() - 1)];
    }
    double GetMesureOut1(BOOL, CLong &) { ++scalar_reads; return 42.25; }
};
struct FakeRobot { FakeKeyence Keyence; };
extern FakeRobot Robot1;
