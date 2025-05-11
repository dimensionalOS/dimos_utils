#!/usr/bin/env python3
"""
Build script for tf_lcm_py that handles CMake, extension module location, and proper installation.
Run this script instead of using pip install directly.

This script will:
1. Build the C++ extension module using CMake
2. Find and copy the module to the correct location
3. Install the package with pip

Usage:
    python build.py [install|develop|wheel]

Options:
    install  - Install the package (default)
    develop  - Install in development mode
    wheel    - Build a wheel without installing
"""
import os
import sys
import glob
import shutil
import platform
import subprocess
from pathlib import Path

def find_cmake():
    """Find the cmake executable"""
    try:
        subprocess.check_output(["cmake", "--version"])
        return "cmake"
    except:
        # Try common cmake locations
        candidates = [
            "/usr/local/bin/cmake",
            "/usr/bin/cmake",
            "C:\\Program Files\\CMake\\bin\\cmake.exe",
        ]
        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        
        print("ERROR: CMake not found. Please install CMake and add it to your PATH.")
        sys.exit(1)

def find_extension(build_dir, package_dir, module_name="tf_lcm_py"):
    """Find the built extension module
    
    Args:
        build_dir: Directory where build files are located
        package_dir: Directory where the package is located
        module_name: Name of the module to search for (default: tf_lcm_py)
        
    Returns:
        Path to the found module or None if not found
    """
    # Determine the extension suffix based on platform
    if platform.system() == "Windows":
        extensions = [".pyd", "*.pyd"]
    else:
        extensions = [".so", "*.so"]
    
    # Check if module_name already has an underscore prefix
    if not module_name.startswith("_"):
        # Try both with and without underscore
        module_patterns = [f"*{module_name}*", f"*_{module_name}*"]
    else:
        # Already has underscore, just search for it
        module_patterns = [f"*{module_name}*"]
    
    # Search patterns
    patterns = []
    for mod_pattern in module_patterns:
        # Various locations in the build directory
        patterns.append(os.path.join(build_dir, "**", mod_pattern))
        # The package directory
        patterns.append(os.path.join(package_dir, "**", mod_pattern))
    
    # Find all matching files
    found_modules = []
    for pattern in patterns:
        for ext in extensions:
            matches = glob.glob(pattern + ext, recursive=True)
            found_modules.extend(matches)
    
    if found_modules:
        # Find the newest module (most recently modified)
        found_modules.sort(key=lambda f: os.path.getmtime(f), reverse=True)
        return found_modules[0]
    
    return None

def main():
    # Parse command-line arguments
    import argparse
    parser = argparse.ArgumentParser(description="Build and install tf_lcm_py")
    parser.add_argument('action', nargs='?', default='install', 
                        choices=['install', 'develop', 'wheel'],
                        help='Action to perform (default: install)')
    parser.add_argument('--keep-build', action='store_true',
                        help='Keep build directory instead of cleaning it')
    parser.add_argument('--force-abi-tag', action='store_true',
                        help='Force the extension to use Python version-specific ABI tags')
    args = parser.parse_args()
    
    # Get Python version info
    import platform
    py_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    py_version_compact = f"{sys.version_info.major}{sys.version_info.minor}"
    platform_name = platform.system().lower()
    
    # Get the current directory (python/ directory)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Define paths
    source_dir = os.path.abspath(os.path.join(current_dir, "..")) # tf_lcm directory
    build_dir = os.path.join(current_dir, "build")
    package_dir = os.path.join(current_dir, "tf_lcm_py")
    
    # Clean the build directory before rebuilding
    if not args.keep_build and os.path.exists(build_dir):
        print("Cleaning build directory...")
        shutil.rmtree(build_dir)
    os.makedirs(build_dir, exist_ok=True)
    
    # Make sure package directory exists
    os.makedirs(package_dir, exist_ok=True)
    
    print(f"Building for Python {sys.version}")
    print(f"Python {py_version} on {platform_name}")
    print(f"Source directory: {source_dir}")
    print(f"Package directory: {package_dir}")
    print(f"Building version-specific extension module for Python {py_version}")
    
    # Step 1: Build the extension module using CMake
    print("\nStep 1: Building extension module with CMake...")
    cmake_exe = find_cmake()
    
    # Configure CMake
    print("Configuring CMake...")
    cmake_args = [
        cmake_exe,
        source_dir,
        f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={package_dir}",
        f"-DPYTHON_EXECUTABLE={sys.executable}",
        "-DBUILD_PYTHON_BINDINGS=ON",
    ]
    
    # Add any platform-specific flags
    if sys.platform == 'darwin':  # macOS
        # Set deployment target to ensure compatibility
        cmake_args.append("-DCMAKE_OSX_DEPLOYMENT_TARGET=10.14")
    
    subprocess.check_call(cmake_args, cwd=build_dir)
    
    # Build the Python module
    print("Building extension module...")
    build_args = [cmake_exe, "--build", ".", "--config", "Release"]
    subprocess.check_call(build_args, cwd=build_dir)
    
    # Print clear message about Python version compatibility
    print(f"\nExtension module built specifically for Python {py_version}")
    print(f"If you need to use this package with other Python versions, run this script once for each version.")
    
    # Step 2: Find the built modules
    print("\nStep 2: Locating built extension modules...")
    
    # First try to find _tf_lcm_py target
    module_path = find_extension(build_dir, package_dir, "_tf_lcm_py")
    if not module_path:
        # If not found, try tf_lcm_py target (without underscore)
        module_path = find_extension(build_dir, package_dir, "tf_lcm_py")
    
    if not module_path:
        print("\nERROR: Could not find any extension modules!")
        print("Listing all shared libraries in build directory:")
        for ext in [".so", ".pyd", ".dll"]:
            for file in glob.glob(os.path.join(build_dir, "**", f"*{ext}"), recursive=True):
                print(f"  {file}")
        sys.exit(1)
    
    print(f"Found extension module: {module_path}")
    
    # Step 3: Copy all built modules to the package directory with correct names
    print("\nStep 3: Copying modules to the package directory...")
    
    # Copy all related modules we can find
    module_files = []
    for name in ["_tf_lcm_py", "tf_lcm_py"]:
        for ext in [".so", ".pyd", ".dll", ".cpython-*-darwin.so"]:
            for module in glob.glob(os.path.join(build_dir, f"**/*{name}*{ext}"), recursive=True):
                target_name = os.path.basename(module)
                target_path = os.path.join(package_dir, target_name)
                
                # Check if source and destination are different files
                if os.path.abspath(module) != os.path.abspath(target_path):
                    print(f"Copying {module} -> {target_path}")
                    shutil.copy2(module, target_path)
                    module_files.append(target_path)
                else:
                    print(f"Skipping copy of {module} (same as destination)")
                    module_files.append(module)  # Still add to module_files
    
    if not module_files:
        # If we found a module earlier but failed to copy it, try direct copy
        target_path = os.path.join(package_dir, os.path.basename(module_path))
        
        # Check if source and destination are different files
        if os.path.abspath(module_path) != os.path.abspath(target_path):
            print(f"Directly copying {module_path} -> {target_path}")
            shutil.copy2(module_path, target_path)
            module_files.append(target_path)
        else:
            print(f"Skipping copy of {module_path} (same as destination)")
            module_files.append(module_path)  # Still add to module_files
    
    # Step 4: Install the package
    print(f"\nStep 4: {args.action.capitalize()}ing the package...")
    
    if args.action == 'wheel':
        # Build a wheel
        pip_args = [sys.executable, "-m", "pip", "wheel", ".", "-w", "dist"]
        subprocess.check_call(pip_args, cwd=current_dir)
        print(f"\nWheel file created in {os.path.join(current_dir, 'dist')}")
    elif args.action == 'develop':
        # Install in development mode
        pip_args = [sys.executable, "-m", "pip", "install", "-e", "."]
        subprocess.check_call(pip_args, cwd=current_dir)
        print("\nSuccessfully installed tf_lcm_py in development mode!")
    else:  # 'install'
        # Regular installation
        pip_args = [sys.executable, "-m", "pip", "install", "."]
        subprocess.check_call(pip_args, cwd=current_dir)
        print("\nSuccessfully installed tf_lcm_py!")
    
    # Print import example
    print("\nTo use the package, simply import it in your Python code:")
    print("import tf_lcm_py")
    print("\nMake sure to subscribe to the correct channels:")
    print('lc.subscribe("tf#tf2_msgs.TFMessage", tf_callback)')
    
    # Print version compatibility information
    print("\nPython Version Compatibility:")
    print(f"✓ This package is now built for Python {py_version}")
    print("Note: If you need to use this package with multiple Python versions:")
    print("1. Switch to each Python version (e.g., 'pyenv global 3.12.x')")
    print("2. Run this build script again with that Python version")
    print("The module loading system will automatically select the right version at runtime.")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
