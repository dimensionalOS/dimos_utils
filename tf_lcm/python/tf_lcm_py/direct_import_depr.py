"""
Direct import module for tf_lcm_py

This script provides low-level loading of the tf_lcm_py module
without relying on pythonic import mechanisms that might not work.
"""

import os
import sys
import glob
import importlib
from ctypes import CDLL, RTLD_GLOBAL
from pathlib import Path

def locate_module():
    """Find the module file using a variety of search strategies"""
    # Get the directory containing this file
    this_dir = Path(__file__).parent
    
    # Look for any shared library in this directory
    for ext in ['.so', '.pyd', '.cpython-*-darwin.so', '*.dll']:
        files = list(this_dir.glob(f"*{ext}"))
        for file in files:
            if '_tf_lcm_py' in file.name or 'tf_lcm_py' in file.name:
                return str(file)
    
    # Nothing found
    return None

def load_module():
    """Load the module directly using ctypes to avoid import issues"""
    module_path = locate_module()
    if not module_path:
        raise ImportError(f"Could not find tf_lcm_py module in {Path(__file__).parent}")
    
    # Load the library directly using ctypes
    try:
        # Force loading with RTLD_GLOBAL to ensure symbols are available
        lib = CDLL(module_path, RTLD_GLOBAL)
        return True
    except Exception as e:
        print(f"Error loading library via ctypes: {e}")
        return False

# Try loading the module
success = load_module()
if success:
    print(f"Successfully loaded tf_lcm_py module via direct loading")
else:
    print(f"Failed to load tf_lcm_py module, import may still fail")
