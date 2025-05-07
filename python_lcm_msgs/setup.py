# This file is kept for backward compatibility with older pip versions
# For modern pip (25+), pyproject.toml is used instead

from setuptools import setup

# This minimal setup.py allows installation with:
# - pip install -e .
# - python setup.py develop
# - python setup.py install

if __name__ == '__main__':
    try:
        # Use setuptools.setup which is defined by the pyproject.toml configuration
        setup()
    except Exception as e:
        # Fallback setup for older versions that might not support pyproject.toml
        print("Warning: Using fallback setup. Consider upgrading pip and setuptools.")
        from setuptools import find_packages
        setup(
            name="lcm_msgs",
            version="0.1.1",
            description="LCM generated Python bindings for ROS based types",
            author="Dimensional",
            packages=find_packages(),  # This will find lcm_msgs and all subpackages
            install_requires=[
                "lcm",
                "foxglove-websocket",
            ],
            python_requires=">=3.8",
        )