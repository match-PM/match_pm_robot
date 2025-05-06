import re

# Mapping from dataType node IDs to C types and UA type enums
type_map = {
    '1LU': ('UA_Boolean', 'UA_TYPES_BOOLEAN', 'false'),
    '11LU': ('UA_Double', 'UA_TYPES_DOUBLE', '0.0'),
    '6LU': ('UA_Int32', 'UA_TYPES_INT32', '0'),
    '12LU': ('UA_Float', 'UA_TYPES_FLOAT', '0.0f'),
    # Extend as needed
}

def add_variant_initialization_and_fix_access(code):
    # Force all access levels to 3
    code = re.sub(r'attr\.userAccessLevel\s*=\s*\d+;', 'attr.userAccessLevel = 3;', code)
    code = re.sub(r'attr\.accessLevel\s*=\s*\d+;', 'attr.accessLevel = 3;', code)

    # Add array initialization and UA_Variant_setArray
    pattern = re.compile(
        r'(UA_VariableAttributes\s+attr\s*=\s*UA_VariableAttributes_default;\s*.*?)'
        r'(attr\.arrayDimensionsSize\s*=\s*1;\s*)'
        r'(UA_UInt32\s+arrayDimensions\[1\];\s*arrayDimensions\[0\]\s*=\s*(\d+);.*?)'
        r'(attr\.arrayDimensions\s*=\s*&arrayDimensions\[0\];\s*)'
        r'(attr\.dataType\s*=\s*UA_NODEID_NUMERIC\(ns\[0\],\s*(\d+LU)\);)',
        re.DOTALL
    )

    def replacer(match):
        pre = match.group(1)
        size = int(match.group(4))
        data_type_id = match.group(7)
        post = match.group(5) + match.group(6)

        if data_type_id not in type_map:
            print(f"Warning: Data type {data_type_id} not in type map. Skipping.")
            return match.group(0)

        c_type, ua_type, default_val = type_map[data_type_id]
        init_array = f"{c_type} init_val[{size}] = {{ {', '.join([default_val] * size)} }};\n"
        variant_set = f"UA_Variant_setArray(&attr.value, init_val, {size}, &UA_TYPES[{ua_type}]);\n"

        return pre + match.group(2) + match.group(3) + post + init_array + variant_set

    return re.sub(pattern, replacer, code)

# Load file
with open("pm_opcua_server.c", "r") as f:
    original_code = f.read()

# Modify code
modified_code = add_variant_initialization_and_fix_access(original_code)

# Save result
with open("opcua_server_modified.c", "w") as f:
    f.write(modified_code)

print("Modification complete. Output written to opcua_server_modified.c.")
