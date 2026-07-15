import argparse
import json
import ntpath
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
import yaml
from datetime import datetime, timezone

############################################
# This script generates a combined URDF file for Unity simulation
# by combining multiple URDF files generated from different scenarios.
# The script reads the pm_robot_bringup_config.yaml configuration file, modifies it for each scenario,
# generates a URDF file for each scenario, and then combines them into a single URDF file.
# It also fixes mesh paths, renames links based on the STL file names, and emits a JSON manifest
# (pm_robot_unity_manifest.json) that maps every YAML selection to the URDF links it owns.
# The Unity side (configureRobot.cs and the delta-import editor tool) consumes that manifest,
# so adding a new tool/chuck/tip in ROS requires no Unity code changes.
############################################


def generate_combined_urdf(yaml_path, xacro_path, output_dir,
                           output_name='pm_robot_unity.urdf', unity_project=None):
    # Load original YAML
    with open(yaml_path, 'r') as file:
        original_config = yaml.safe_load(file)

    # Ensure output directory
    os.makedirs(output_dir, exist_ok=True)

    # Backup YAML
    yaml_backup_path = yaml_path + '.backup'
    shutil.copyfile(yaml_path, yaml_backup_path)

    # Combined URDF <robot> element
    combined_robot_el = ET.Element('robot', attrib={'name': 'pm_robot'})

    # Keep track of which elements we've seen
    seen_elements = set()

    # Track generated intermediate URDF files
    generated_urdfs = []

    # Manifest sections collected while iterating the scenario loops
    manifest_sections = []

    # Set use_keyence_bottom in yaml file to True
    original_config['measuring_systems']['use_keyence_bottom'] = True

    tools_cfg = original_config['pm_robot_tools']
    gripper_1_config = tools_cfg['pm_robot_tool_parallel_gripper_1_jaw']
    gripper_2_config = tools_cfg['pm_robot_tool_parallel_gripper_2_jaws']
    vacuum_config = tools_cfg['pm_robot_vacuum_tools']
    dispenser_config = original_config['pm_robot_1K_dispenser_tip']
    gonio_left_config = original_config['pm_robot_gonio_left']
    gonio_right_config = original_config['pm_robot_gonio_right']

    # Disable all tools by default so no scenario leaks whichever tool happens
    # to be selected in the config. Each tool type is enabled only inside its
    # own dedicated loop below. This keeps the per-scenario link sets clean,
    # which the manifest diffing below relies on.
    gripper_1_config['use_paralell_gripper'] = False
    gripper_2_config['use_paralell_gripper'] = False
    vacuum_config['use_vacuum_tool'] = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def is_gripper_owned(name):
        """Links the parallel-gripper xacro emits for the tool itself. They are
        deliberately kept at their generic xacro names (never STL-renamed) and
        made unique via the tool/jaw suffix instead, because that is how the
        Unity scene objects are named."""
        return name.startswith('Tool_Parallel_Gripper') or name == 'PM_Robot_Tool_TCP'

    def fix_mesh_paths_and_link_names(xml_element, link_name_map, keep=None):
        if xml_element.tag == 'link' and not (keep and keep(xml_element.get('name'))):
            old_name = xml_element.get('name')
            # Attempt to find STL for renaming
            for geom_el in xml_element.findall('.//geometry/mesh'):
                filename = geom_el.get('filename', '')
                if filename:
                    # Convert to relative path if needed
                    if filename.startswith('file://'):
                        if 'meshes/' in filename:
                            rel_path = 'meshes/' + filename.split('meshes/')[-1]
                        else:
                            rel_path = filename.replace('file://', '')
                        geom_el.set('filename', rel_path)
                        filename = rel_path

                    # If it's an STL, use it for renaming
                    if filename.lower().endswith('.stl'):
                        base = ntpath.basename(filename)  # e.g. "Tool_Schunk_SPT_Holder.STL"
                        stl_basename = os.path.splitext(base)[0]  # "Tool_Schunk_SPT_Holder"
                        # Rename the link
                        xml_element.set('name', stl_basename)
                        link_name_map[old_name] = stl_basename
                        break
        else:
            # If not a link, just fix any meshes found
            for mesh_el in xml_element.findall('.//mesh'):
                filename = mesh_el.get('filename', '')
                if filename.startswith('file://'):
                    if 'meshes/' in filename:
                        mesh_el.set('filename', 'meshes/' + filename.split('meshes/')[-1])
                    else:
                        mesh_el.set('filename', filename.replace('file://', ''))

        # Recurse into children
        for child in xml_element:
            fix_mesh_paths_and_link_names(child, link_name_map, keep)

    def rename_links(root, mapping, joint_suffix=None):
        """Rename links per ``mapping`` and rewrite joint parent/child and
        gazebo references. If ``joint_suffix`` is given, joints whose child was
        renamed also get their name suffixed (keeps joint names unique when the
        same generic joint appears in several scenarios)."""
        for link_el in root.findall('link'):
            nm = link_el.get('name')
            if nm in mapping:
                link_el.set('name', mapping[nm])

        for joint_el in root.findall('joint'):
            parent_el = joint_el.find('parent')
            if parent_el is not None and parent_el.get('link') in mapping:
                parent_el.set('link', mapping[parent_el.get('link')])

            child_el = joint_el.find('child')
            if child_el is not None and child_el.get('link') in mapping:
                child_el.set('link', mapping[child_el.get('link')])
                if joint_suffix:
                    joint_el.set('name', f"{joint_el.get('name')}_{joint_suffix}")

        for gz_el in root.findall('gazebo'):
            ref = gz_el.get('reference')
            if ref in mapping:
                gz_el.set('reference', mapping[ref])

    def process_scenario(urdf_path, keep=None):
        """Parse a scenario URDF, fix mesh paths and rename links to their STL
        basenames. ``keep(name)`` marks links that must not be renamed (the
        gripper-owned ones). Links shared with other scenarios - notably the
        tool changer 't_axis_toolchanger' -> 't_axis_tool' - MUST be renamed in
        every scenario, otherwise the same physical link ends up in the
        combined URDF twice under two names.
        Returns (root, stl_renamed_names)."""
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        link_name_map = {}
        fix_mesh_paths_and_link_names(root, link_name_map, keep)
        # Joint names are intentionally NOT changed for STL renames (parity
        # with the long-standing behavior the Unity scene was built against).
        rename_links(root, link_name_map)
        return root, set(link_name_map.values())

    def link_names(root):
        return {l.get('name') for l in root.findall('link')}

    def descendants_of(root, base_link):
        """All links strictly below ``base_link`` in the joint tree."""
        children = {}
        for joint_el in root.findall('joint'):
            parent_el = joint_el.find('parent')
            child_el = joint_el.find('child')
            if parent_el is None or child_el is None:
                continue
            children.setdefault(parent_el.get('link'), []).append(child_el.get('link'))

        result = set()
        stack = [base_link]
        while stack:
            for ch in children.get(stack.pop(), []):
                if ch not in result:
                    result.add(ch)
                    stack.append(ch)
        return result

    def item_meta(root, owned):
        """Root links (owned links whose parent is outside the item) and mesh
        paths referenced by the owned links."""
        parent_of = {}
        for joint_el in root.findall('joint'):
            parent_el = joint_el.find('parent')
            child_el = joint_el.find('child')
            if parent_el is not None and child_el is not None:
                parent_of[child_el.get('link')] = parent_el.get('link')

        root_links = sorted(l for l in owned if parent_of.get(l) not in owned)

        meshes = []
        for link_el in root.findall('link'):
            if link_el.get('name') in owned:
                for mesh_el in link_el.findall('.//mesh'):
                    fn = mesh_el.get('filename', '')
                    if fn and fn not in meshes:
                        meshes.append(fn)
        return root_links, meshes

    def add_to_combined(root):
        for child in list(root):
            child_str = ET.tostring(child, encoding='unicode')
            if child_str not in seen_elements:
                seen_elements.add(child_str)
                combined_robot_el.append(child)

    def write_config_and_run_xacro(tag):
        with open(yaml_path, 'w') as temp_yaml:
            yaml.dump(original_config, temp_yaml)
        urdf_file = os.path.join(output_dir, f'temp_{tag}.urdf')
        subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)
        generated_urdfs.append(urdf_file)
        return urdf_file

    def finalize_single_level_section(scenarios, yaml_path_key, enable_key,
                                      select_key, unity_parent):
        """Diff the scenarios of a single-selection section (dispenser tips,
        chucks) against each other: links present in every scenario are base
        robot, the rest belong to the selected item. Owned links that were not
        STL-renamed (e.g. the STL-less '1K_Dispenser_Tip') are renamed to the
        YAML selection name so they are unique and follow the convention."""
        all_sets = [link_names(root) for _, root, _ in scenarios]
        common = set.intersection(*all_sets) if len(all_sets) > 1 else set()

        items = []
        for (yaml_name, root, stl_renamed), links in zip(scenarios, all_sets):
            owned = links - common

            generic = sorted(l for l in owned if l not in stl_renamed)
            if generic:
                if len(generic) == 1:
                    mapping = {generic[0]: yaml_name}
                else:
                    mapping = {g: f'{g}_{yaml_name}' for g in generic}
                rename_links(root, mapping, joint_suffix=yaml_name)
                owned = (owned - set(mapping)) | set(mapping.values())

            root_links, meshes = item_meta(root, owned)
            items.append({
                'yamlNames': [yaml_name],
                'links': sorted(owned),
                'rootLinks': root_links,
                'meshes': meshes,
            })
            add_to_combined(root)

        manifest_sections.append({
            'yamlPath': yaml_path_key,
            'enableKey': enable_key,
            'selectKeys': [select_key],
            'unityParent': unity_parent,
            'items': items,
        })

    def finalize_tool_sections(tool_scenarios, tool_sections):
        """Two-phase processing of all tool scenarios (vacuum + grippers).

        Tool-owned links are the structural descendants of the T-axis rotation
        plate ('Gripper_Rot_Plate'), EXCEPT links that appear in more than one
        scenario with byte-identical <link> and <joint> elements — those are
        shared tool-changer infrastructure (t_axis_toolchanger, t_axis_tool)
        and stay unsuffixed so every scenario deduplicates onto one copy.
        'PM_Robot_Tool_TCP' also appears in every scenario but with a
        different joint origin per tool, so it fails the identity test and is
        correctly suffixed per tool. Caveat: a part shared identically by two
        tools would be classified as infrastructure and never toggled off in
        Unity (same as the pre-manifest behavior for t_axis_tool).
        """
        occurrences = {}  # link name -> list of (link_str, joint_str)
        parent_name = {}  # link name -> parent link name (identical for shared candidates)
        per_scenario = []
        for sc in tool_scenarios:
            root = sc['root']
            structural = descendants_of(root, 'Gripper_Rot_Plate')
            link_els = {l.get('name'): l for l in root.findall('link')}
            joint_els = {}
            for j in root.findall('joint'):
                child_el = j.find('child')
                parent_el = j.find('parent')
                if child_el is not None:
                    joint_els[child_el.get('link')] = j
                    if parent_el is not None:
                        parent_name[child_el.get('link')] = parent_el.get('link')
            for nm in structural:
                link_str = ET.tostring(link_els[nm], encoding='unicode') if nm in link_els else ''
                joint_str = ET.tostring(joint_els[nm], encoding='unicode') if nm in joint_els else ''
                occurrences.setdefault(nm, []).append((link_str, joint_str))
            per_scenario.append(structural)

        shared = {nm for nm, occ in occurrences.items()
                  if len(occ) > 1 and len(set(occ)) == 1}

        # A link can only be shared if its whole ancestry is shared (or the
        # base robot): a tip that is identical relative to its parent still
        # belongs to the tool when that parent is suffixed per scenario.
        changed = True
        while changed:
            changed = False
            for nm in list(shared):
                parent = parent_name.get(nm)
                if parent in occurrences and parent not in shared:
                    shared.discard(nm)
                    changed = True

        for sc, structural in zip(tool_scenarios, per_scenario):
            root = sc['root']
            owned = structural - shared
            mapping = {nm: f"{nm}_{sc['suffix']}" for nm in owned}
            rename_links(root, mapping, joint_suffix=sc['suffix'])
            owned_suffixed = set(mapping.values())

            root_links, meshes = item_meta(root, owned_suffixed)
            add_to_combined(root)
            tool_sections[sc['section']]['items'].append({
                'yamlNames': list(sc['yaml_names']),
                'links': sorted(owned_suffixed),
                'rootLinks': root_links,
                'meshes': meshes,
            })

        manifest_sections.extend(tool_sections.values())

    # ------------------------------------------------------------------
    # Scenario loops
    # ------------------------------------------------------------------

    try:
        # --- 1K dispenser tips -------------------------------------------------
        scenarios = []
        for tip in dispenser_config['availabe_dispenser_tips']:
            dispenser_config['use_dispenser_tip'] = tip
            urdf_file = write_config_and_run_xacro(tip)
            root, stl_renamed = process_scenario(urdf_file)
            scenarios.append((tip, root, stl_renamed))
        finalize_single_level_section(
            scenarios, 'pm_robot_1K_dispenser_tip', '', 'use_dispenser_tip',
            dispenser_config.get('unity_parent', ''))

        # --- Gonio left chucks -------------------------------------------------
        scenarios = []
        for chuck in gonio_left_config['availabe_chucks']:
            gonio_left_config['use_chuck'] = chuck
            urdf_file = write_config_and_run_xacro(f'left_{chuck}')
            root, stl_renamed = process_scenario(urdf_file)
            scenarios.append((chuck, root, stl_renamed))
        finalize_single_level_section(
            scenarios, 'pm_robot_gonio_left', 'with_Gonio_Left', 'use_chuck',
            gonio_left_config.get('unity_parent', ''))

        # --- Gonio right chucks ------------------------------------------------
        scenarios = []
        for chuck in gonio_right_config['availabe_chucks']:
            gonio_right_config['use_chuck'] = chuck
            urdf_file = write_config_and_run_xacro(f'right_{chuck}')
            root, stl_renamed = process_scenario(urdf_file)
            scenarios.append((chuck, root, stl_renamed))
        finalize_single_level_section(
            scenarios, 'pm_robot_gonio_right', 'with_Gonio_Right', 'use_chuck',
            gonio_right_config.get('unity_parent', ''))

        # --- Tools (vacuum: tool x tip, grippers: tool x jaw) -------------------
        # All tool scenarios are generated first and finalized together in
        # finalize_tool_sections: each tool-owned link gets a
        # '_{tool}_{tip/jaw}' suffix so tools that share STL basenames or the
        # generic gripper link names no longer collapse into one link with
        # conflicting parent joints in the combined URDF. Vacuum links are
        # STL-renamed first (their scene names derive from the STLs); gripper
        # links keep their generic xacro names ('Tool_Parallel_Gripper_*'),
        # matching how the existing scene objects were built.
        tool_scenarios = []
        tool_sections = {
            'vacuum': {
                'yamlPath': 'pm_robot_tools/pm_robot_vacuum_tools',
                'enableKey': 'use_vacuum_tool',
                'selectKeys': ['use_tool', 'use_tip'],
                'unityParent': vacuum_config.get('unity_parent', ''),
                'items': [],
            },
            'gripper1': {
                'yamlPath': 'pm_robot_tools/pm_robot_tool_parallel_gripper_1_jaw',
                'enableKey': 'use_paralell_gripper',
                'selectKeys': ['use_tool', 'use_jaw_type'],
                'unityParent': gripper_1_config.get('unity_parent', ''),
                'items': [],
            },
            'gripper2': {
                'yamlPath': 'pm_robot_tools/pm_robot_tool_parallel_gripper_2_jaws',
                'enableKey': 'use_paralell_gripper',
                'selectKeys': ['use_tool', 'use_jaw_type'],
                'unityParent': gripper_2_config.get('unity_parent', ''),
                'items': [],
            },
        }

        vacuum_config['use_vacuum_tool'] = True
        for tool in vacuum_config['availabe_tools']:
            vacuum_config['use_tool'] = tool['tool_name']
            for tip in tool['availabe_tips']:
                vacuum_config['use_tip'] = tip
                urdf_file = write_config_and_run_xacro(f'{tool["tool_name"]}_{tip}')
                root, _ = process_scenario(urdf_file)
                tool_scenarios.append({
                    'section': 'vacuum',
                    'yaml_names': [tool['tool_name'], tip],
                    'suffix': f'{tool["tool_name"]}_{tip}',
                    'root': root,
                })
        vacuum_config['use_vacuum_tool'] = False

        for gripper_config, section, tag in (
                (gripper_1_config, 'gripper1', 'gripper1'),
                (gripper_2_config, 'gripper2', 'gripper2')):
            gripper_1_config['use_paralell_gripper'] = gripper_config is gripper_1_config
            gripper_2_config['use_paralell_gripper'] = gripper_config is gripper_2_config

            for tool in gripper_config['available_tools']:
                gripper_config['use_tool'] = tool['tool_name']
                for jaw in tool['available_jaws']:
                    gripper_config['use_jaw_type'] = jaw
                    urdf_file = write_config_and_run_xacro(
                        f'{tag}_{tool["tool_name"]}_{jaw}')
                    root, _ = process_scenario(urdf_file, keep=is_gripper_owned)
                    tool_scenarios.append({
                        'section': section,
                        'yaml_names': [tool['tool_name'], jaw],
                        'suffix': f'{tool["tool_name"]}_{jaw}',
                        'root': root,
                    })
            gripper_config['use_paralell_gripper'] = False

        finalize_tool_sections(tool_scenarios, tool_sections)

        # ------------------------------------------------------------------
        # Write outputs
        # ------------------------------------------------------------------

        combined_urdf_path = os.path.join(output_dir, output_name)
        ET.ElementTree(combined_robot_el).write(
            combined_urdf_path, encoding='utf-8', xml_declaration=True)
        print(f'Combined URDF created at: {combined_urdf_path}')

        manifest = {
            'generatedAt': datetime.now(timezone.utc).isoformat(),
            'sourceConfig': os.path.basename(yaml_path),
            'urdf': output_name,
            'sections': manifest_sections,
        }
        manifest_path = os.path.join(output_dir, 'pm_robot_unity_manifest.json')
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        print(f'Unity manifest created at: {manifest_path}')

        if unity_project:
            copy_to_unity_project(unity_project, combined_urdf_path,
                                  manifest_path, combined_robot_el, output_dir)

    finally:
        # Restore the original YAML configuration
        shutil.copyfile(yaml_backup_path, yaml_path)
        os.remove(yaml_backup_path)

        # Clean up all generated temp URDF files
        for temp_urdf_path in generated_urdfs:
            if os.path.exists(temp_urdf_path):
                os.remove(temp_urdf_path)


def copy_to_unity_project(unity_project, urdf_path, manifest_path,
                          combined_robot_el, output_dir):
    """Copy the combined URDF, the manifest, and any missing STL meshes into
    <unity_project>/Assets/PM_Robot/. Existing meshes are left untouched (their
    Unity .meta files must not be regenerated)."""
    pm_robot_dir = os.path.join(unity_project, 'Assets', 'PM_Robot')
    if not os.path.isdir(pm_robot_dir):
        raise FileNotFoundError(f'Not a Unity pm_robot project (missing {pm_robot_dir})')

    shutil.copyfile(urdf_path, os.path.join(pm_robot_dir, os.path.basename(urdf_path)))
    shutil.copyfile(manifest_path, os.path.join(pm_robot_dir, os.path.basename(manifest_path)))

    # Mesh paths in the combined URDF are relative to the package share dir
    # (the parent of the urdf output dir).
    meshes_root = os.path.dirname(output_dir)
    copied = 0
    for mesh_el in combined_robot_el.iter('mesh'):
        rel_path = mesh_el.get('filename', '')
        if not rel_path.startswith('meshes/'):
            continue
        dst = os.path.join(pm_robot_dir, rel_path)
        if os.path.exists(dst):
            continue
        src = os.path.join(meshes_root, rel_path)
        if not os.path.exists(src):
            print(f'WARNING: mesh not found in ROS package: {src}')
            continue
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(src, dst)
        copied += 1
    print(f'Copied URDF + manifest to {pm_robot_dir} ({copied} new mesh(es))')


def main():
    parser = argparse.ArgumentParser(
        description='Generate the combined Unity URDF and manifest for the pm_robot.')
    parser.add_argument(
        '--yaml',
        default='/home/match-pm/ros2_ws/install/pm_robot_bringup/share/pm_robot_bringup/config/pm_robot_bringup_config.yaml',
        help='Path to the installed pm_robot_bringup_config.yaml')
    parser.add_argument(
        '--xacro',
        default='/home/match-pm/ros2_ws/install/pm_robot_description/share/pm_robot_description/urdf/pm_robot_main.xacro',
        help='Path to the installed pm_robot_main.xacro')
    parser.add_argument(
        '--output-dir',
        default='/home/match-pm/ros2_ws/install/pm_robot_description/share/pm_robot_description/urdf',
        help='Directory for the combined URDF and manifest')
    parser.add_argument(
        '--output-name', default='pm_robot_unity.urdf',
        help='Filename of the combined URDF')
    parser.add_argument(
        '--unity-project', default=None,
        help='Optional path to the Unity project; copies URDF, manifest and '
             'missing meshes into <project>/Assets/PM_Robot/')
    args = parser.parse_args()

    generate_combined_urdf(
        yaml_path=args.yaml,
        xacro_path=args.xacro,
        output_dir=args.output_dir,
        output_name=args.output_name,
        unity_project=args.unity_project,
    )


if __name__ == '__main__':
    main()
