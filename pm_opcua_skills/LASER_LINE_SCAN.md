# Laser line scans and fresh measurements

The system has one physical Keyence sensor. Only the Windows server opens its
serial connection. ROS communicates with that server through OPC UA. Neither
ROS service adds robot motion.

## ROS interfaces and migration

Rebuild `pm_msgs` and every affected ROS caller together with the changed client,
hardware interface and controllers. Existing measurement clients and Float64
subscribers must migrate; retaining a service or topic name does not preserve
wire compatibility. Update the Windows server at the same time.

| Interface | Contract |
| --- | --- |
| `/pm_opcua_skills_controller/LaserLineScan` | `pm_msgs/srv/LaserLineScan`: UInt32 `length_um` and `interval`; response includes `success`, `distances`, `message`, `actual_length_um`, `actual_interval_um`, `sample_count`, and `restoration_verified`. |
| `/pm_sensor_controller/Laser/GetMeasurement` | `pm_msgs/srv/LaserGetMeasurement`: `timeout_ms` (zero uses the configured default); response includes `success`, a `sample` of type `pm_msgs/msg/LaserMeasurement`, and `message`. It waits for a new validated driver read rather than returning a pre-request cached number. |
| `/pm_sensor_controller/Laser/Stream` | `pm_msgs/msg/LaserMeasurement`, replacing `std_msgs/msg/Float64`: measurement, validity, valid-read sequence, last successful read-completion timestamp, age, OPC UA status, scanning state, and diagnostic message. |

Heights retain the sensor's native units. No metre conversion or calibration is
invented. Confirm the deployed sensor's units and calibration before converting
these values into physical distances. A successful driver read is not proof of
a newly triggered hardware exposure; timestamps describe successful read
completion, not an independently verified device exposure time.

Profile settings are discrete. Requested lengths 1..1100 micrometres round up to
120, 250, 500, or 1100. Positive intervals 1..4 map to 2, 5..9 map to 5, and values
10 or greater map to 10 micrometres. The profile reader expects both endpoints,
so the expected count is `actual_length_um / actual_interval_um + 1`. For example,
length 100 and interval 30 produce length 120, interval 10, and 13 records. The
inclusive endpoint count is based on the observed machine response and still needs
verification for all sensor settings. The configured window length is
not a claim about the first sample's origin or the distance between the first
and last sample; those require the machine's coordinate/calibration reference.
Use the actual numeric response fields when interpreting the profile.

The server and client reject incomplete profiles, wrong record counts,
`--------` markers, non-finite heights, overlong height fields, and malformed
record boundaries. Each accepted record has the `OP,` prefix, an eight-character
height field, and two nonempty numeric fields. The latter fields are validated
for framing, without inventing their physical meaning. Failed scans return no
successful partial height vector.

## Windows worker and OPC UA protocol

`CLaserAcquisitionTask` runs sensor IO outside the OPC UA request loop.
`OpcuaServer/LaserScan.cpp` registers the following methods under `Objects/Skills`.
Every callback only accesses shared state; it never performs serial IO or waits
for the device.

| Method | Inputs in order | Outputs in order |
| --- | --- | --- |
| `StartLaserLineScan` | UInt32 interval, UInt32 length | UInt32 scanId, Boolean accepted, String message |
| `GetLaserLineScanResult` | UInt32 scanId | String rawProfile, String message, Boolean success, Boolean done, UInt32 actualLengthUm, UInt32 actualIntervalUm, Boolean restorationVerified |
| `GetLaserMeasurementState` | None | Double measurement, UInt32 status, DateTime sourceTimestamp, UInt32 sequence, UInt32 ageMs, Boolean scanning, String message |
| `RequestLaserMeasurement` | UInt32 timeoutMs (1..600000) | UInt32 requestId, DateTime serverEpoch, Boolean accepted, String message |
| `GetLaserMeasurementResult` | UInt32 requestId, DateTime serverEpoch | The seven measurement-state outputs, followed by Boolean done |

The original `LaserLineScan` OPC UA method returns a migration error rather
than performing a blocking acquisition. Older scan clients expecting four
outputs from `GetLaserLineScanResult` also require rebuilding.

The profile worker selects program 5, configures profile mode and sampling,
verifies readback, settles, and attempts at most three profiles. It then
attempts to restore and verify program 4. Terminal `done=true` includes failures:
it does not prove restoration. `success=true` requires validated data and
verified restoration. Restoration errors append to acquisition errors.

One profile is admitted at a time. A completed, uncollected profile prevents
replacement for 60 seconds; successful terminal retrieval marks it collected.
Collection does not delete it, but allows a subsequent start to replace it.
An unknown/replaced ID is a terminal failure. These slots are process-wide and
not session-owned. Trusted callers and diagnostic tools must coordinate result
collection; IDs provide correlation, not access control.

## Fresh ordinary readings

An ordinary read never runs concurrently with a profile in the Windows worker.
A fresh-measurement request accepted during a profile survives the old
one-second cache-demand window. After the profile, the worker verifies program
4 and performs the new driver read. An acquisition already in progress when a
request arrives cannot satisfy that request. A retained request takes priority
over admitting another profile, so repeated scans cannot starve it.

There is one outstanding fresh-measurement slot. Another outstanding request is
rejected. Completion retains a valid result until collected or older than the
500 ms freshness limit; unsuccessful/expired results may be replaced. Each poll
must carry both request ID and server epoch, so an old request cannot silently
match a new server instance. Requests expire at their server-side deadline.
A timeout does not cancel an active profile or bypass its restoration.

Each successful scalar read advances the sequence and sets a source timestamp.
Initial, failed, sentinel (`-9998.99` or `-9999.99`), stale, and scan-time samples
are unavailable. Failed attempts do not advance the successful-read sequence or
source time. Numeric values are NaN when unavailable; callers must check
`valid` and `status_code`, not only whether a response arrived. During a scan,
the topic reports unavailable ordinary data.

The server computes age with its monotonic clock. The client conservatively
adds the RPC round-trip duration; ROS adds time since receipt. Samples older
than 500 ms are unusable. This avoids comparing unsynchronised Windows and ROS
wall clocks to establish freshness. The source timestamp is still useful for
logging. Consumers can impose a stricter age requirement.

The sensor controller uses a dedicated OPC UA client and worker for this
service/topic, separate from PMSystem's auxiliary and motion clients. Deferred
service responses and a 10 ms timer keep the executor available while the worker
waits. Only one ROS fresh-read request is retained; extra requests fail promptly.
Deadlines start when the ROS callback accepts the request. This is bounded
application waiting, not a hard real-time delivery guarantee. Deactivation
rejects outstanding work and joins the worker after its current network work.

Set these parameters on `pm_sensor_controller`:

| Parameter | Default and meaning |
| --- | --- |
| `laser_opcua_endpoint` | Empty, disabling connections until explicitly configured. The real-hardware YAML supplies `opc.tcp://PC1M0484-1:4840`; override it to match PMSystem's endpoint, including Unity/simulation deployments. |
| `laser_measurement_timeout_ms` | 120000; valid range 1..600000. A nonzero service `timeout_ms` overrides this default. |
| `laser_rpc_timeout_ms` | 500; valid range 1..5000. Each service RPC also respects the remaining caller budget. |

A missing/older server yields an explicit error; there is no direct ROS serial
fallback. Valid force data continues updating when laser quality is bad, and
invalid force data does not replace the last force snapshot. PMSystem's legacy
numeric laser state becomes NaN on invalid readings or failed sensor transport.

The skills controller uses separate clients/callback groups for scans and
Dispense, with a two-thread executor. Its `laser_scan_timeout_ms` defaults to
120000 and accepts 1..600000. The scan client budgets blocking calls against the
remaining deadline and rejects late terminal success. It does not replay a lost
start reply at the application layer; open62541 may reconnect before a call.
The server may still be acquiring/restoring after the caller has timed out.

## Machine prerequisites and recovery

Before live use, verify that program 5 may be reconfigured and that program 4 is
the required ordinary-measurement baseline. This implementation preserves those
existing program numbers. It does not restore an arbitrary previous program.
If restoration is unverified, ordinary acquisition verifies program 4 before
reading; an active communication/machine error remains a failure. No code clears
machine errors or bypasses the driver's communication blocks.

**Native Windows routines must be excluded while a service acquisition is
active.** The existing driver guard protects individual serial transactions,
not an entire configure/read/restore operation. This change does not establish
that the machine's native task scheduling provides that exclusion. Confirm the
supported operating mode or add framework-level coordination before enabling
overlapping native workflows. Do not infer an additional sensor from these
internal callers: they use the same physical device through Windows.

Program readback accepts complete, zero-padded program numbers and trims
whitespace/CR/LF after the driver returns a reply. Queries are polled up to three
times, with 100 ms between attempts. The worker invokes each configuration write
once; the underlying driver may retransmit inside its own retry/error handling.
Driver framing checks happen before the worker's normalisation.

## Build, verification and deployment

1. Build `Soft/App1M0484.sln` with the machine's established Windows SDK/toolchain.
   `LaserScan.cpp` and `LaserScan.h` are already in its project. This Linux
   workspace cannot validate the Windows machine framework build.
2. From the ROS workspace, rebuild the changed interfaces and dependants together:

   ```bash
   colcon build --packages-select pm_msgs pm_client pm_hardware_interface \
     pm_sensor_controller pm_opcua_skills_controller
   source install/setup.bash
   ```

3. Update all external scalar subscribers/service callers for the changed ROS
   types. The current repository has no additional non-test callers to migrate.
4. Restart the Windows application and the ROS launch that owns these controllers
   together. General launch instructions are in [the repository README](../README.md).
   Validate calibration, program recovery, native-operation exclusion, and one
   stationary scan on the real machine before relying on the measurements.

Offline server/client/driver checks use a simulated sensor and in-process UA
method calls. The runner compiles production scan/measurement parsing and server
worker logic and extracted driver routines. It substitutes transport, sensor, and machine framework;
it does not prove Windows scheduling, serial timing, or calibration.

```bash
python3 pm_client/test/laser_scan/run_checks.py \
  --server-root /path/containing/Soft \
  --open62541-prefix /path/to/installed/open62541
```

The server root contains `Soft`; the prefix contains `include` and `lib`. Add
`--sanitize` for address/undefined-behavior checks. The source checkout did not
include the referenced sensor-controller service and auxiliary-sensor tests, so
those paths have not been verified by this runner.
