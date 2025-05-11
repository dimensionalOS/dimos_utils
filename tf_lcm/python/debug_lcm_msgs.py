#!/usr/bin/env python3
"""
Debug script to inspect lcm_msgs module structure
"""

import sys
import os
import inspect

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Try to import lcm_msgs
print("Trying to import lcm_msgs...")
try:
    import lcm_msgs
    print(f"Successfully imported lcm_msgs from: {lcm_msgs.__file__}")
    print(f"Module attributes: {dir(lcm_msgs)}")
    
    # Check if geometry_msgs is directly accessible
    if hasattr(lcm_msgs, 'geometry_msgs'):
        print("lcm_msgs has geometry_msgs attribute")
        print(f"geometry_msgs attributes: {dir(lcm_msgs.geometry_msgs)}")
    else:
        print("lcm_msgs does NOT have geometry_msgs attribute")
        
        # Try to find any modules that might contain what we need
        for name in dir(lcm_msgs):
            if not name.startswith('__'):
                attr = getattr(lcm_msgs, name)
                if inspect.ismodule(attr):
                    print(f"Found submodule: {name}")
                    print(f"Submodule attributes: {dir(attr)}")
                elif inspect.isclass(attr):
                    print(f"Found class: {name}")
    
except ImportError as e:
    print(f"Failed to import lcm_msgs: {e}")
    
# Try direct imports of potential message types
print("\nTrying direct imports of message types...")

try:
    from lcm_msgs import TransformStamped, Vector3, Quaternion
    print("Direct import of message types successful")
except ImportError as e:
    print(f"Direct import failed: {e}")

try:
    # Check Python path
    print("\nPython sys.path:")
    for path in sys.path:
        print(f"  {path}")
except Exception as e:
    print(f"Error showing sys.path: {e}")
