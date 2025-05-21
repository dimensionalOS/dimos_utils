#!/usr/bin/env python3
"""
Setup script for tf_lcm_py package
"""

import os
import sys
import subprocess
import shutil
import glob
from pathlib import Path
from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """
    Custom extension class for CMake-based build
    """
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """
    Custom build_ext command to build the C++ extension using CMake
    """
    def run(self):
        # Check if CMake is available
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError("CMake must be installed to build the extension")

        # Build all extensions
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        # Define build directory
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        
        # Create build directory if it doesn't exist
        build_temp = os.path.join(self.build_temp, "build")
        os.makedirs(build_temp, exist_ok=True)
        
        # Ensure the package directory exists
        pkg_dir = os.path.join(self.build_lib, 'tf_lcm_py')
        os.makedirs(pkg_dir, exist_ok=True)
        
        # Set CMake args
        # We want to build just the Python bindings, not the full project
        source_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        
        # Configure CMake with specific output directory
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={pkg_dir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            "-DBUILD_PYTHON_BINDINGS=ON",
        ]
        
        # Add platform-specific build settings
        build_args = ["--", "-j4"]  # Parallel build
        
        # Print useful information for debugging
        print(f"Building extension in {build_temp}")
        print(f"Output directory: {pkg_dir}")
        print(f"Source directory: {source_dir}")
        print(f"CMake arguments: {cmake_args}")
        
        # Call CMake to configure and build
        subprocess.check_call(
            ["cmake", source_dir] + cmake_args, 
            cwd=build_temp
        )
        
        # Build the project
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", "_tf_lcm_py"] + build_args, 
            cwd=build_temp
        )
        
        # Find the built module with flexible pattern matching
        patterns = [
            # Search in library output directory
            os.path.join(pkg_dir, "**", "*tf_lcm_py*.so"), 
            # Search in temporary build directory
            os.path.join(build_temp, "**", "*tf_lcm_py*.so"),
            # Search in the site-packages directory
            os.path.join(self.build_lib, "**", "*tf_lcm_py*.so"),
            # Search at various relative paths
            os.path.join(build_temp, "*tf_lcm_py*.so"),
            os.path.join(self.build_lib, "*tf_lcm_py*.so"),
        ]
        
        # Look for the extension using all patterns
        for pattern in patterns:
            print(f"Searching with pattern: {pattern}")
            ext_files = glob.glob(pattern, recursive=True)
            if ext_files:
                break
        
        # If still not found, try listing all .so files to help diagnose
        if not ext_files:
            print("Module not found with specific patterns, listing all .so files:")
            all_so_files = glob.glob(os.path.join(build_temp, "**", "*.so"), recursive=True)
            for so_file in all_so_files:
                print(f"Found .so file: {so_file}")
            
            all_so_files = glob.glob(os.path.join(self.build_lib, "**", "*.so"), recursive=True)
            for so_file in all_so_files:
                print(f"Found .so file in build_lib: {so_file}")
            
            raise RuntimeError("Could not find built extension file")


# Create a simplified setup() call that just focuses on getting the extension built
# We'll let pyproject.toml handle the rest of the metadata

# Copy all .so files as package data
# This ensures all shared libraries are included in the distribution
package_data = {'tf_lcm_py': ['*.so', '*.pyd']}

# Manually ensuring the __init__.py is included
if os.path.exists('tf_lcm_py/__init__.py'):
    print("Found __init__.py, will be included in package")
else:
    print("WARNING: __init__.py not found in tf_lcm_py directory")

# Simplified setup call
setup(
    name="tf_lcm_py",
    packages=["tf_lcm_py"],
    package_data=package_data,
    ext_modules=[CMakeExtension("tf_lcm_py._tf_lcm_py")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)
