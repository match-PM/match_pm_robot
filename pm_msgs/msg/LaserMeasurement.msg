# Sensor-native height units; no conversion to metres is implied.
float64 measurement
bool valid
uint32 sequence
builtin_interfaces/Time source_timestamp
# Age evaluated by the server's monotonic clock, plus local delivery age.
uint32 age_ms
# OPC UA StatusCode. A timestamp alone does not establish validity.
uint32 status_code
bool scanning
string message
