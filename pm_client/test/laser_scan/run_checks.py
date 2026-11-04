#!/usr/bin/env python3
"""Exercise the production laser server/client against an offline sensor double.

The Windows machine framework is mocked; no network listener or hardware commands
are used. This does not replace building App1M0484 with its Windows SDKs.
"""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-root", required=True, type=Path,
                        help="Directory containing the server's Soft folder")
    parser.add_argument("--open62541-prefix", required=True, type=Path,
                        help="Installed open62541 prefix, containing include and lib")
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    tests = Path(__file__).resolve().parent
    repo = tests.parents[2]
    soft = args.server_root.resolve() / "Soft"
    includes = args.open62541_prefix.resolve() / "include"
    library = args.open62541_prefix.resolve() / "lib"
    module = soft / "OpcuaServer"

    # The existing server files use legacy single-byte encodings. Latin-1
    # preserves every byte; the extracted logic and all new identifiers are ASCII.
    driver = (soft / "AppComponents/AppKeyenceSensorHead.cpp").read_bytes().decode("latin1")
    guard_start = driver.index("\tclass CSerialTransaction")
    guard_end = driver.index("\tBOOL IsError;", guard_start)
    profile_start = driver.index("\tstd::string CKeyenceDetector::GetMeasureProfile")
    profile_end = driver.index("\tCBool CKeyenceDetector::SetAutoZeroOn", profile_start)
    status_start = driver.index("\tCString CKeyenceDetector::GetCommunicationStatus()")
    status_end = driver.index("\tCString CKeyenceDetector::GetTextLineDisplayMeasure", status_start)
    command_start = driver.index("\tCString CKeyenceDetector::SendCommand")
    command_end = driver.index("\tCLong CKeyenceDetector::CheckAnswerReceived", command_start)
    production_driver = (
        "namespace KeyenceSensorHead_ {\n" + driver[guard_start:guard_end]
        + driver[status_start:status_end] + driver[command_start:command_end]
        + driver[profile_start:profile_end] + "\n}\n"
    )

    ns = {"ms": "http://schemas.microsoft.com/developer/msbuild/2003"}
    project = ET.parse(soft / "App1M0484.vcxproj")
    entries = {entry.attrib.get("Include") for entry in project.findall(".//ms:ClCompile", ns)}
    if "OpcuaServer\\LaserScan.cpp" not in entries:
        raise RuntimeError("The Windows project does not compile LaserScan.cpp")
    robot = (soft / "Robot1.cpp").read_bytes()
    task = (module / "COPCUAServerTask.cpp").read_bytes()
    if b"OPCUALaserAcquisitionTask.Define(this)" not in robot:
        raise RuntimeError("Laser acquisition task is not defined on the robot")
    if b"OPCUALaserAcquisitionTask.Start()" not in task:
        raise RuntimeError("OPC UA startup does not start the laser worker")

    flags = [os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Wpedantic", "-pthread"]
    if args.sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]

    with tempfile.TemporaryDirectory(prefix="pm-laser-check-") as build:
        build = Path(build)
        (build / "driver_under_test.inc").write_text(production_driver)
        task_source = task.decode("latin1")
        task_start = task_source.index("VOID CLaserAcquisitionTask::ExecuteTask()")
        task_end = task_source.index("// Eingangsfunktion", task_start)
        (build / "worker_under_test.cpp").write_text(
            '#include "COPCUAServerTask.h"\n#include "LaserScan.h"\nnamespace OPCUAServer {\n'
            + task_source[task_start:task_end] + "\n}\n"
        )
        common = flags + ["-I", str(tests), "-I", str(build)]
        subprocess.run(common + [str(tests / "test_driver.cpp"), "-o", str(build / "driver")], check=True)
        subprocess.run([str(build / "driver")], check=True)
        subprocess.run(common + [
            "-I", str(module), "-I", str(repo / "pm_client/include"),
            "-I", str(includes), "-I", str(includes / "open62541"),
            str(tests / "test_protocol.cpp"), str(module / "LaserScan.cpp"),
            str(build / "worker_under_test.cpp"),
            str(repo / "pm_client/src/skills.cpp"), "-L", str(library),
            "-Wl,-rpath," + str(library), "-lopen62541", "-o", str(build / "protocol")
        ], check=True)
        subprocess.run([str(build / "protocol")], check=True)


if __name__ == "__main__":
    main()
