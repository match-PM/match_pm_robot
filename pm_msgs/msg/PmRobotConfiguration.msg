# Absolute path of the bringup configuration represented by this message.
string config_file_path

# True when the real-hardware configuration is active, false for simulation/Unity.
bool use_real_config

# Complete launch-time bringup snapshot serialized as YAML.
# Editing the file does not change this snapshot; restart bringup to reload it.
string config_yaml
