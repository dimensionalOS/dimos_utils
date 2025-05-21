#!/usr/bin/env python3
"""
Script to update existing code to use the lcm_msgs import pattern.

This script:
1. Searches for python files using the LCM message types
2. Updates the import statements to use the lcm_msgs prefix
"""

import os
import re
import sys
from pathlib import Path

# Message packages to update imports for
MSG_PACKAGES = [
    "sensor_msgs",
    "geometry_msgs",
    "std_msgs",
    "actionlib_msgs",
    "builtin_interfaces",
    "diagnostic_msgs",
    "foxglove_msgs",
    "nav_msgs",
    "shape_msgs",
    "stereo_msgs",
    "tf2_msgs",
    "trajectory_msgs",
    "visualization_msgs",
]

def update_file(file_path):
    """Update import statements in a file."""
    with open(file_path, 'r') as f:
        content = f.read()

    # Look for import statements using our message packages
    updated = False
    for pkg in MSG_PACKAGES:
        # Match import statements like "from sensor_msgs import JointState"
        pattern = fr'from\s+{pkg}\s+import\s+'
        if re.search(pattern, content):
            # Replace with "from lcm_msgs.sensor_msgs import JointState"
            content = re.sub(pattern, f'from lcm_msgs.{pkg} import ', content)
            updated = True
            
        # Match import statements like "import sensor_msgs"
        pattern = fr'import\s+{pkg}\b'
        if re.search(pattern, content):
            # Replace with "import lcm_msgs.sensor_msgs as sensor_msgs"
            content = re.sub(pattern, f'import lcm_msgs.{pkg} as {pkg}', content)
            updated = True

    if updated:
        print(f"Updating {file_path}")
        with open(file_path, 'w') as f:
            f.write(content)
        return True
    return False

def main():
    """Main function."""
    # Get the directory to search in
    if len(sys.argv) > 1:
        search_dir = sys.argv[1]
    else:
        search_dir = "."
    
    # Find Python files to update
    search_path = Path(search_dir)
    python_files = list(search_path.glob('**/*.py'))
    
    # Skip the message package files themselves and the lcm_msgs directory
    python_files = [
        f for f in python_files 
        if not any(pkg in str(f) for pkg in MSG_PACKAGES + ['lcm_msgs']) 
        or not f.is_relative_to(search_path / 'lcm_msgs')
    ]
    
    # Update the import statements in each file
    updated_count = 0
    for file_path in python_files:
        if update_file(file_path):
            updated_count += 1
    
    print(f"Updated {updated_count} files.")

if __name__ == '__main__':
    main()