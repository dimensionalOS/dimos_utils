"""
Helper module for finding and copying the extension module during installation
"""
import os
import sys
import shutil
import glob
from pathlib import Path

def find_tf_lcm_extension():
    """Find the tf_lcm extension module from various locations"""
    # Potential locations to search
    search_dirs = [
        # Current directory
        os.path.abspath('.'),
        # Build directory
        os.path.abspath('./build'),
        # CMake build directories
        os.path.abspath('../build'),
        # Any subdirectories
        *glob.glob(os.path.abspath('./*/'))
    ]
    
    # Potential extension file patterns (platform-specific)
    patterns = [
        "*_tf_lcm_py.so",            # Unix/Linux
        "*_tf_lcm_py.*.so",          # Unix with Python version
        "*_tf_lcm_py.pyd",           # Windows
        "*_tf_lcm_py.*.pyd",         # Windows with Python version
        "tf_lcm_py.*.so",            # Alternative naming
        "tf_lcm_py.so",              # Alternative naming
    ]
    
    # Search for the extension in all directories
    found_extensions = []
    for directory in search_dirs:
        for pattern in patterns:
            # Try recursive glob
            for path in glob.glob(os.path.join(directory, "**", pattern), recursive=True):
                found_extensions.append(path)
    
    if found_extensions:
        # Sort by modification time (newest first)
        found_extensions.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return found_extensions[0]
    
    return None

def copy_extension_to_package():
    """Find and copy the extension to the package directory"""
    # Find the extension
    extension_path = find_tf_lcm_extension()
    
    if extension_path:
        # Make sure the target directory exists
        target_dir = os.path.join(os.path.dirname(__file__), 'tf_lcm_py')
        os.makedirs(target_dir, exist_ok=True)
        
        # Copy the extension
        target_path = os.path.join(target_dir, os.path.basename(extension_path))
        print(f"Copying extension: {extension_path} -> {target_path}")
        shutil.copy(extension_path, target_path)
        return True
    
    print("WARNING: Could not find tf_lcm_py extension module")
    return False

if __name__ == "__main__":
    # If run directly, try to copy the extension
    success = copy_extension_to_package()
    sys.exit(0 if success else 1)
