# LCM Message Definitions and ROS msg to LCM Converter

This repository contains LCM message definitions and a tool to convert ROS messages to LCM messages.

- To note, Time.lcm and Duration.lcm are either from builtin_interfaces or from std_msgs. We need to make sure to rerun both the Python msg to lcm converter as well as the lcm-gen command to generate the proper bindings for both the std_msgs and builtin_interfaces namespaces.

- [ ] TODO: Check other LCM types like PointCloud to make sure that they are being converted over to the LCM files like time and duration since there are ROS message types that have the same name and they need Python bindings too.

- The lcm-gen command is used to generate the language bindings for the LCM message definitions.

- The Python msg to lcm converter is used to convert the ROS messages to LCM messages.

