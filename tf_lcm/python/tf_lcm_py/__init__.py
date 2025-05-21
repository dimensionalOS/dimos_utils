"""
tf_lcm_py - Python bindings for the tf_lcm transform library

This package provides Python access to the tf_lcm library,
which is a LCM-based alternative to ROS TF2 for managing
coordinate frame transforms.

Usage:
    import tf_lcm_py
    
    # Create a transform buffer
    buffer = tf_lcm_py.Buffer(cache_time=10.0)
    
    # Use with native Python LCM for best performance
    import lcm
    lc = lcm.LCM()
    
    # Setup transform callbacks
    def tf_callback(channel, data):
        msg = lcm_msgs.tf2_msgs.TFMessage.decode(data)
        for i in range(msg.transforms_length):
            buffer.set_transform(msg.transforms[i], "lcm_publisher", False)
    
    # Subscribe to the correct channel names
    lc.subscribe("tf#tf2_msgs.TFMessage", tf_callback)
"""

"""
tf_lcm_py - Python bindings for the tf_lcm transform library

This package provides Python access to the tf_lcm library,
which is a LCM-based alternative to ROS TF2 for managing
coordinate frame transforms.
"""

import os
import sys
import importlib.util
import sysconfig
from pathlib import Path

# Version information
__version__ = "0.1.0"
__import_method__ = None

# Key module names to try - in order of preference
MODULE_NAMES = ['_tf_lcm_py', 'tf_lcm_py']

def import_module():
    """Import the appropriate extension module for the current Python version"""
    # Get the expected extension suffix for the current Python version
    # This is crucial - a module compiled for one Python version won't work with another
    ext_suffix = sysconfig.get_config_var('EXT_SUFFIX')
    if not ext_suffix:
        # Fallback in case sysconfig doesn't provide the suffix
        if sys.platform == 'win32':
            ext_suffix = '.pyd'
        else:
            ext_suffix = '.so'
            
    # The package directory where extension modules should be located
    package_dir = Path(__file__).parent
    
    # Look for modules with the exact extension suffix first - these are most likely to work
    for module_name in MODULE_NAMES:
        # Try version-specific module first (most reliable)
        module_path = package_dir / f"{module_name}{ext_suffix}"
        if module_path.exists():
            try:
                spec = importlib.util.spec_from_file_location(module_name, str(module_path))
                if spec:
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    return module, None, f"Version-specific module: {module_path}"
            except Exception as e:
                return None, e, f"Failed to import version-specific module: {module_path}"
    
    # If we couldn't find or load a version-specific module, collect information for error message
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    all_modules = []
    
    # List all extension modules in the package directory
    for ext in ['.so', '.pyd', '*.cpython-*.so', '*-darwin.so']:
        all_modules.extend(list(package_dir.glob(f"*{ext}")))
    
    # Create detailed error message
    available_modules = "\n  ".join([str(m) for m in all_modules])
    error_msg = f"\nCould not find a compatible extension module for Python {python_version}.\n"
    error_msg += f"Extension modules should match the pattern: *{ext_suffix}\n"
    
    if available_modules:
        error_msg += f"\nAvailable modules that are NOT compatible with current Python version:\n  {available_modules}\n"
    else:
        error_msg += f"\nNo extension modules found in {package_dir}\n"
    
    error_msg += f"\nPlease run 'python build.py' with Python {python_version} to build a compatible extension module.\n"
    error_msg += "Each Python version (3.10, 3.12, etc.) requires its own compiled extension module."
    
    return None, ImportError(error_msg), None

# Attempt to import the appropriate module
module, error, import_info = import_module()

if module:
    __import_method__ = import_info
    # Import all public symbols from the module into the current namespace
    for name in dir(module):
        if not name.startswith('_'):
            globals()[name] = getattr(module, name)
else:
    # No compatible module found
    raise error


