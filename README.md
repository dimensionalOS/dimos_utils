# LCM Message Definitions and ROS msg to LCM Converter

This repository contains LCM message definitions and a tool to convert ROS messages to LCM messages.


- The Python msg to lcm converter is used to convert the ROS messages to LCM messages.

```py
python3 ros_to_lcm.py ros_msgs lcm_files
```


- The lcm-gen command is used to generate the language bindings for the LCM message definitions.

```sh
./lcm_batch_processor.sh -p lcm_files -o python_lcm_msgs
```