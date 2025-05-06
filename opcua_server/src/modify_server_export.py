import re

# Mapping from dataType node IDs to C types and UA type enums
type_map = {
    '1LU': ('UA_Boolean', 'UA_TYPES_BOOLEAN', 'false'),
    '11LU': ('UA_Double', 'UA_TYPES_DOUBLE', '0.0'),
    '6LU': ('UA_Int32', 'UA_TYPES_INT32', '0'),
    '12LU': ('UA_Float', 'UA_TYPES_FLOAT', '0.0f'),
    # Extend as needed
}

# Define your known node IDs and their init values
unitsperincrement_values = {
    "ns=1;i=50358": 1.66666666667e-05,
    "ns=1;i=50342": 1.66666666667e-05,
    "ns=1;i=50326": 0.000563936275201,
    "ns=1;i=50310": 0.000804634695848,
    "ns=1;i=50294": 7.60006755616e-05,
    "ns=1;i=50278": 0.05,
    "ns=1;i=50262": 0.05,
    "ns=1;i=50246": 0.05,
}

def enforce_access_levels(code):
    code = re.sub(r'attr\.userAccessLevel\s*=\s*\d+;', 'attr.userAccessLevel = 3;', code)
    code = re.sub(r'attr\.accessLevel\s*=\s*\d+;', 'attr.accessLevel = 3;', code)
    return code

def inject_array_initialization(code):
    # Match a whole block from attr init until displayName (which is the next safe anchor)
    pattern = re.compile(
        r'(UA_VariableAttributes\s+attr\s*=\s*UA_VariableAttributes_default;\s*)'
        r'(.*?)'
        r'(attr\.arrayDimensionsSize\s*=\s*1;\s*)'
        r'(UA_UInt32\s+arrayDimensions\[1\];\s*arrayDimensions\[0\]\s*=\s*(\d+);.*?)'
        r'(attr\.arrayDimensions\s*=\s*&arrayDimensions\[0\];\s*)'
        r'(attr\.dataType\s*=\s*UA_NODEID_NUMERIC\(ns\[0\],\s*(\d+LU)\);.*?)'
        r'(attr\.displayName\s*=.*?)',  # we now end right before displayName
        re.DOTALL
    )

    def replacer(match):
        full_block = match.group(0)

        # Stronger: match only if init for this attr is missing
        if re.search(r'UA_Variant_setArray\s*\(\s*&attr\.value', full_block):
            return full_block  # Already has array init

        pre_init = match.group(1) + match.group(2)
        size = int(match.group(5))
        data_type_id = match.group(8)
        post_array = match.group(3) + match.group(4) + match.group(6)
        post_dtype = match.group(7)
        post_display = match.group(9)

        if data_type_id not in type_map:
            return full_block

        c_type, ua_type, default_val = type_map[data_type_id]

        init_array = f"{c_type} init_val[{size}] = {{ {', '.join([default_val] * size)} }};\n"
        variant_set = f"UA_Variant_setArray(&attr.value, init_val, {size}, &UA_TYPES[{ua_type}]);\n"

        injected = pre_init + post_array + post_dtype + init_array + variant_set + post_display
        return injected

    return re.sub(pattern, replacer, code)


def inject_unitsperincrement_scalars(code, value_map):
    for node_id, value in value_map.items():
        ns, i = node_id.replace("ns=", "").replace("i=", "").split(";")

        # Find comment with the correct node ID
        pattern = re.compile(
            rf'(/\*\s*UnitsPerIncrement\s*-\s*ns={ns};i={i}\s*\*/)',
            re.MULTILINE
        )

        for match in pattern.finditer(code):
            start_pos = match.end()
            # Look ahead for the attr block starting after this match
            attr_match = re.search(
                r'(UA_VariableAttributes\s+attr\s*=\s*UA_VariableAttributes_default;\s*\n)',
                code[start_pos:]
            )
            if not attr_match:
                continue  # couldn't find an attr block after comment

            insert_pos = start_pos + attr_match.end()

            # Check if the scalar is already set in the next ~20 lines
            lookahead = code[start_pos:start_pos + 800]  # look at some reasonable range
            if 'UA_Variant_setScalar(&attr.value' in lookahead:
                continue  # already initialized

            # Create the insertion
            init_code = (
                f"    UA_Double init_val = {value};\n"
                f"    UA_Variant_setScalar(&attr.value, &init_val, &UA_TYPES[UA_TYPES_DOUBLE]);\n"
            )

            # Insert the code
            code = code[:insert_pos] + init_code + code[insert_pos:]

    return code


# ---------- MAIN FLOW ----------

with open("pm_opcua_server.c", "r") as f:
    code = f.read()

code = enforce_access_levels(code)
code = inject_array_initialization(code)
code = inject_unitsperincrement_scalars(code, unitsperincrement_values)

with open("opcua_server_modified.c", "w") as f:
    f.write(code)

print("✅ All modifications applied and saved to 'opcua_server_modified.c'.")
