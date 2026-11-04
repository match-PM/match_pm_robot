# Laser line scan integration

The ROS service remains `/pm_opcua_skills_controller/LaserLineScan`, using
`pm_msgs/srv/LaserLineScan`. Length and interval are in micrometres. Returned
heights retain the Keyence sensor's units. The scan does not move any robot axis.

## Server execution

`CLaserAcquisitionTask` runs device IO outside the OPC UA request loop. It is
defined with the robot and started after the server starts successfully.
`OpcuaServer/LaserScan.cpp` implements these methods under `Objects/Skills`:

| Method | Inputs, in order | Outputs, in order |
| --- | --- | --- |
| `StartLaserLineScan` | UInt32 interval, UInt32 length | UInt32 scanId, Boolean accepted, String message |
| `GetLaserLineScanResult` | UInt32 scanId | String stringResult, String messageState, Boolean boolSuccess, Boolean done |

Start returns immediately. The worker selects program 5, applies profile mode,
verifies width/interval, allows settling, and tries at most three profiles.
Program 4 is restored and verified before the result is marked complete.
Invalid `--------` readings remain failures; no fabricated heights are returned.
The original `LaserLineScan` OPC UA method returns an update instruction without
starting a blocking acquisition, so older clients cannot block motion IO.

The server holds one job at a time and preserves an uncollected completed result
for at least 60 seconds. IDs prevent a poll from reading a different job's result.
Ordinary `Laser/Measurement` reads return a cached value and its source timestamp;
the worker refreshes it when requested and not scanning. Before the first refresh,
the value is the existing driver's invalid sentinel, `-9998.99`.

The ROS client requires both new methods, starts once, then polls every 100 ms.
The default polling deadline is 120 seconds. A lost connection is reported without
replaying the start request. After a client timeout, acquisition/restoration may
still be running; a subsequent start is rejected while the server is busy.

## Build and deploy

1. Build the Windows `Soft/App1M0484.sln` application, using its existing SDK/toolchain.
   `LaserScan.cpp` and `LaserScan.h` are included in the project.
2. Restart that OPC UA server application so the worker and new methods are active.
3. From the ROS workspace root, build `pm_client` and `pm_opcua_skills_controller`,
   source `install/setup.bash`, and restart the robot launch that owns the controller.
   `pm_msgs` does not need regeneration if `LaserLineScan.srv` is already built.

A new ROS client refuses the old server with a clear update message. Full Windows
compilation and a live scan are required to validate the machine framework and
sensor configuration; the offline checks below cannot perform those steps.

If heights remain `--------` after settling/retries, check a manual profile with
program 5 at the same calibrated measurement position. The receive fixes cannot
turn an invalid sensor reading into a valid one.

### Program selection/restoration errors

Program readback validates the complete `PR,<program>` response rather than its
last character. Leading/trailing whitespace and CR/LF terminators are removed;
zero-padded program numbers are supported. Readback is polled at most three times
with 100 ms between attempts after the existing one-second settling delay. Only
queries are repeated, not the program-change command.

A failed restoration remains a failure even when profile data was acquired.
It is appended to any earlier acquisition error instead of replacing it. The
message includes the write acknowledgement and actual readback, for example
`PW,4 reply=[PW]; PR reply=[PR,5]; expected [PR,4]`. An empty reply is shown as `[]`;
control characters are escaped so the ROS log stays on one line.

The worker checks the driver's communication status before starting and before
configuration commands. Disabled RS232 communication or an active machine error
is reported explicitly; the scan never clears errors or bypasses these blocks.
If restoration cannot be confirmed, check the machine application's reported
error and verify program 4 before further laser use. Rebuild/restart the Windows
application after changing these sources; no ROS interface change is needed.

## Offline regression checks

These compile the actual client and server acquisition module against an in-memory
open62541 server and simulated sensor. They also compile the production receive
loop, command handling and serial-ownership guard with simulated serial input.
They cover complete profiles, extra reads, timeouts, exception cleanup,
invalid-data retries, restoration,
program reply framing, blocked/disabled communication, primary-error preservation,
job IDs, concurrent requests, cached-read responsiveness, and lost start replies.

```bash
python3 pm_client/test/laser_scan/run_checks.py \
  --server-root /home/pmlab/pm_Server/01_PM_Zelle/04_PM_Zelle_Programm/laser \
  --open62541-prefix /home/pmlab/pm_ros2_ws/install/open62541
```

Add `--sanitize` for address/undefined-behavior checks. Leak detection may need to be
disabled with `ASAN_OPTIONS=detect_leaks=0` in restricted or ptraced environments.
