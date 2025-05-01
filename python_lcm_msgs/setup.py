from setuptools import setup, find_packages
import os

setup(
    name="lcm_ros_msgs",
    version="0.1.1",
    description="LCM generated Python bindings for ROS based types",
    author="Dimensional",
    packages=find_packages(),  # This will find lcm_msgs and all subpackages
    install_requires=[
        "lcm",
    ],
    python_requires=">=3.6",
)