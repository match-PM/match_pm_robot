import xml.etree.ElementTree as ET
from collections import Counter

def find_duplicate_links(urdf_content):
    """
    Parses a URDF file content to find duplicate link names.

    Args:
        urdf_content (str): The content of the URDF file as a string.

    Returns:
        list: A list of link names that appear multiple times.
    """

    
    try:
        # Parse the URDF content
        root = ET.fromstring(urdf_content)

        # Extract all link names
        link_names = [link.attrib['name'] for link in root.findall('.//link')]

        # Count occurrences of each link name
        link_counts = Counter(link_names)

        # Filter links that appear more than once
        duplicates = [name for name, count in link_counts.items() if count > 1]

        return duplicates

    except ET.ParseError as e:
        print(f"Error parsing URDF: {e}")
        return []
    

path='/home/match-mover/Documents/ros2_ws/install/pm_robot_description/share/pm_robot_description/urdf/pm_robot_unity_smarpod.urdf'

# Load the URDF content from the file
with open(path, 'r') as file:
    urdf_content = file.read()

duplicates = find_duplicate_links(urdf_content)
print("Duplicate links:", duplicates)