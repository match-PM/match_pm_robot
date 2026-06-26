import yaml
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
import ntpath

############################################
# This script generates a combined URDF file for Unity simulation
# by combining multiple URDF files generated from different scenarios.
# The script reads the pm_robot_bringup_config.yaml configuration file, modifies it for each scenario,
# generates a URDF file for each scenario, and then combines them into a single URDF file.
# The script also fixes mesh paths and renames links based on the STL file names.
############################################

def generate_combined_urdf(yaml_path, xacro_path, output_dir):
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

    # Set use_keyence_bottom in yaml file to True
    original_config['measuring_systems']['use_keyence_bottom'] = True

    # original_config['pm_smparpod_station']['with_smarpod_station'] = True

    # Disable all tools by default so that the dispenser/chuck/gonio scenarios
    # don't leak whichever tool happens to be selected in the config. Each tool
    # type is enabled only inside its own dedicated loop below.
    original_config['pm_robot_tools']['pm_robot_tool_parallel_gripper_1_jaw']['use_paralell_gripper'] = False
    original_config['pm_robot_tools']['pm_robot_tool_parallel_gripper_2_jaws']['use_paralell_gripper'] = False

    # Extract the sets of known tips for easy checking
    dispenser_tips = set(original_config['pm_robot_1K_dispenser_tip']['availabe_dispenser_tips'])
    vacuum_tips = set()
    for tool in original_config['pm_robot_tools']['pm_robot_vacuum_tools']['availabe_tools']:
        for tip in tool['availabe_tips']:
            vacuum_tips.add(tip)
    # Now we have sets of known "tip" names: dispenser_tips and vacuum_tips

    def fix_mesh_paths_and_link_names(xml_element, link_name_map):
        if xml_element.tag == 'link':
            old_name = xml_element.get('name')
            # Attempt to find STL for renaming
            stl_basename = None
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
                        rel_path = 'meshes/' + filename.split('meshes/')[-1]
                        mesh_el.set('filename', rel_path)
                    else:
                        mesh_el.set('filename', filename.replace('file://', ''))

        # Recurse into children
        for child in xml_element:
            fix_mesh_paths_and_link_names(child, link_name_map)

    def update_joint_references_in_tree(root, link_name_map):
        for joint_el in root.findall('joint'):
            parent_el = joint_el.find('parent')
            if parent_el is not None:
                p_link = parent_el.get('link', '')
                if p_link in link_name_map:
                    parent_el.set('link', link_name_map[p_link])

            child_el = joint_el.find('child')
            if child_el is not None:
                c_link = child_el.get('link', '')
                if c_link in link_name_map:
                    child_el.set('link', link_name_map[c_link])

    def rename_tool_tcp_link(root, scenario_name, link_name_map):
        """
        Rename <link name="PM_Robot_Tool_TCP"> only if scenario_name is a known tip.
        Otherwise, do not rename PM_Robot_Tool_TCP or rename it with a generic suffix.
        """
        is_tip_scenario = (scenario_name in vacuum_tips)

        for link_el in root.findall('link'):
            if link_el.get('name') == 'PM_Robot_Tool_TCP':
                old_name = 'PM_Robot_Tool_TCP'
                if is_tip_scenario:
                    # Rename PM_Robot_Tool_TCP with the tip name
                    new_name = f"PM_Robot_Tool_TCP_{scenario_name}"
                else:
                    # Not a tip scenario. Just keep PM_Robot_Tool_TCP as is or rename it with a generic suffix.
                    # Let's keep it unchanged to avoid confusion.
                    new_name = old_name

                link_el.set('name', new_name)
                link_name_map[old_name] = new_name
                return old_name, new_name

        return None, None

    def process_urdf_file(urdf_path, scenario_name):
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        link_name_map = {}

        # Attempt to rename PM_Robot_Tool_TCP if this is a tip scenario
        rename_tool_tcp_link(root, scenario_name, link_name_map)

        # Fix mesh paths and rename links based on STL
        fix_mesh_paths_and_link_names(root, link_name_map)

        # Update joint references for any STL-based renames
        update_joint_references_in_tree(root, link_name_map)

        # Deduplicate and add elements to combined robot
        for child in list(root):
            child_str = ET.tostring(child, encoding='unicode')
            if child_str not in seen_elements:
                seen_elements.add(child_str)
                combined_robot_el.append(child)

    def fix_mesh_paths_only(root):
        """Convert absolute file:// mesh paths to repo-relative meshes/ paths,
        without renaming any links."""
        for mesh_el in root.iter('mesh'):
            filename = mesh_el.get('filename', '')
            if filename.startswith('file://'):
                if 'meshes/' in filename:
                    mesh_el.set('filename', 'meshes/' + filename.split('meshes/')[-1])
                else:
                    mesh_el.set('filename', filename.replace('file://', ''))

    def process_parallel_gripper_urdf(urdf_path, suffix):
        """Process a parallel-gripper scenario URDF.

        Every gripper config emits the same generic link/joint names
        (``Tool_Parallel_Gripper_*`` and ``PM_Robot_Tool_TCP``) and different
        jaw configs can reuse the same jaw STL basenames. To keep the combined
        URDF free of duplicate link/joint names, every gripper-owned link and
        joint is given a unique ``suffix`` (tool name + jaw type). Links that
        belong to the base robot (e.g. ``t_axis_toolchanger``,
        ``Gripper_Rot_Plate``) are left untouched so they still deduplicate
        against the other scenarios.
        """
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        fix_mesh_paths_only(root)

        # Rename gripper-owned links.
        link_name_map = {}
        for link_el in root.findall('link'):
            nm = link_el.get('name', '')
            if nm.startswith('Tool_Parallel_Gripper') or nm == 'PM_Robot_Tool_TCP':
                new_nm = f"{nm}_{suffix}"
                link_name_map[nm] = new_nm
                link_el.set('name', new_nm)

        # Update joints: rewrite parent/child references to the renamed links,
        # and uniquify the name of any joint owned by the gripper (i.e. whose
        # child is a gripper link).
        renamed_links = set(link_name_map.values())
        for joint_el in root.findall('joint'):
            parent_el = joint_el.find('parent')
            if parent_el is not None and parent_el.get('link') in link_name_map:
                parent_el.set('link', link_name_map[parent_el.get('link')])

            child_el = joint_el.find('child')
            if child_el is not None and child_el.get('link') in link_name_map:
                child_el.set('link', link_name_map[child_el.get('link')])

            if child_el is not None and child_el.get('link') in renamed_links:
                joint_el.set('name', f"{joint_el.get('name')}_{suffix}")

        # Update gazebo references to renamed links.
        for gz_el in root.findall('gazebo'):
            ref = gz_el.get('reference')
            if ref in link_name_map:
                gz_el.set('reference', link_name_map[ref])

        # Deduplicate and add elements to combined robot.
        for child in list(root):
            child_str = ET.tostring(child, encoding='unicode')
            if child_str not in seen_elements:
                seen_elements.add(child_str)
                combined_robot_el.append(child)

    try:
        for tip in original_config['pm_robot_1K_dispenser_tip']['availabe_dispenser_tips']:
            original_config['pm_robot_1K_dispenser_tip']['use_dispenser_tip'] = tip
            with open(yaml_path, 'w') as temp_yaml:
                yaml.dump(original_config, temp_yaml)

            urdf_file = os.path.join(output_dir, f'temp_{tip}.urdf')
            subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

            generated_urdfs.append(urdf_file)
            process_urdf_file(urdf_file, tip)

        for chuck in original_config['pm_robot_gonio_left']['availabe_chucks']:
            original_config['pm_robot_gonio_left']['use_chuck'] = chuck
            with open(yaml_path, 'w') as temp_yaml:
                yaml.dump(original_config, temp_yaml)

            urdf_file = os.path.join(output_dir, f'temp_left_{chuck}.urdf')
            subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

            generated_urdfs.append(urdf_file)
            # Here 'chuck' is not a tip, so PM_Robot_Tool_TCP won't get a tip-based rename
            process_urdf_file(urdf_file, chuck)

        for chuck in original_config['pm_robot_gonio_right']['availabe_chucks']:
            original_config['pm_robot_gonio_right']['use_chuck'] = chuck
            with open(yaml_path, 'w') as temp_yaml:
                yaml.dump(original_config, temp_yaml)

            urdf_file = os.path.join(output_dir, f'temp_right_{chuck}.urdf')
            subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

            generated_urdfs.append(urdf_file)
            # Again, 'chuck' is not a tip scenario
            process_urdf_file(urdf_file, chuck)

        for tool in original_config['pm_robot_tools']['pm_robot_vacuum_tools']['availabe_tools']:
            original_config['pm_robot_tools']['pm_robot_vacuum_tools']['use_tool'] = tool['tool_name']

            for tip in tool['availabe_tips']:
                original_config['pm_robot_tools']['pm_robot_vacuum_tools']['use_tip'] = tip
                with open(yaml_path, 'w') as temp_yaml:
                    yaml.dump(original_config, temp_yaml)

                urdf_file = os.path.join(output_dir, f'temp_{tool["tool_name"]}_{tip}.urdf')
                subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

                generated_urdfs.append(urdf_file)
                # This is a vacuum tool tip scenario, so PM_Robot_Tool_TCP may be renamed with the tip name
                process_urdf_file(urdf_file, tip)

        # The vacuum tool and the two parallel-gripper groups are selected by
        # independent flags in the config, so isolate each group by disabling
        # the others. Iterate the 1-jaw grippers (one moving jaw each).
        gripper_1_config = original_config['pm_robot_tools']['pm_robot_tool_parallel_gripper_1_jaw']
        gripper_2_config = original_config['pm_robot_tools']['pm_robot_tool_parallel_gripper_2_jaws']
        original_config['pm_robot_tools']['pm_robot_vacuum_tools']['use_vacuum_tool'] = False

        gripper_1_config['use_paralell_gripper'] = True
        gripper_2_config['use_paralell_gripper'] = False
        for tool in gripper_1_config['available_tools']:
            gripper_1_config['use_tool'] = tool['tool_name']
            for jaw in tool['available_jaws']:
                gripper_1_config['use_jaw_type'] = jaw
                with open(yaml_path, 'w') as temp_yaml:
                    yaml.dump(original_config, temp_yaml)

                urdf_file = os.path.join(output_dir, f'temp_gripper1_{tool["tool_name"]}_{jaw}.urdf')
                subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

                generated_urdfs.append(urdf_file)
                process_parallel_gripper_urdf(urdf_file, f'{tool["tool_name"]}_{jaw}')

        # Iterate the 2-jaw grippers (two moving jaws each).
        gripper_1_config['use_paralell_gripper'] = False
        gripper_2_config['use_paralell_gripper'] = True
        for tool in gripper_2_config['available_tools']:
            gripper_2_config['use_tool'] = tool['tool_name']
            for jaw in tool['available_jaws']:
                gripper_2_config['use_jaw_type'] = jaw
                with open(yaml_path, 'w') as temp_yaml:
                    yaml.dump(original_config, temp_yaml)

                urdf_file = os.path.join(output_dir, f'temp_gripper2_{tool["tool_name"]}_{jaw}.urdf')
                subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

                generated_urdfs.append(urdf_file)
                process_parallel_gripper_urdf(urdf_file, f'{tool["tool_name"]}_{jaw}')

        # for chuck in original_config['pm_smarpod_station']['availabe_chucks']:
        #     original_config['pm_smarpod_station']['use_chuck'] = chuck
        #     with open(yaml_path, 'w') as temp_yaml:
        #         yaml.dump(original_config, temp_yaml)

        #     urdf_file = os.path.join(output_dir, f'temp_smarpod_{chuck}.urdf')
        #     subprocess.run(['ros2', 'run', 'xacro', 'xacro', xacro_path, '-o', urdf_file], check=True)

        #     generated_urdfs.append(urdf_file)
        #     # Here 'chuck' is not a tip, so PM_Robot_Tool_TCP won't get a tip-based rename
        #     process_urdf_file(urdf_file, chuck)

        # Write out the combined URDF (with a single <robot>), now with relative mesh paths
        combined_urdf_path = os.path.join(output_dir, 'pm_robot_unity_asg_gripper.urdf')
        ET.ElementTree(combined_robot_el).write(combined_urdf_path, encoding='utf-8', xml_declaration=True)
        print(f'pm_robot_unity.urdf file created at: {combined_urdf_path}')

    finally:
        # Restore the original YAML configuration
        shutil.copyfile(yaml_backup_path, yaml_path)
        os.remove(yaml_backup_path)

        # Clean up all generated temp URDF files
        for temp_urdf_path in generated_urdfs:
            if os.path.exists(temp_urdf_path):
                os.remove(temp_urdf_path)


# Generate the combined URDF
# modify the paths as needed
generate_combined_urdf(
    yaml_path='/home/match-pm/ros2_ws/install/pm_robot_bringup/share/pm_robot_bringup/config/pm_robot_bringup_config.yaml',
    xacro_path='/home/match-pm/ros2_ws/install/pm_robot_description/share/pm_robot_description/urdf/pm_robot_main.xacro',
    output_dir='/home/match-pm/ros2_ws/install/pm_robot_description/share/pm_robot_description/urdf'
)
