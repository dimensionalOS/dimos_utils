#!/usr/bin/env python3
"""
Final ROS .msg to LCM .lcm converter
This tool converts ROS message definitions to LCM type specifications
with fixes for array syntax, constant declarations, and proper array length declaration.
"""

import os
import re
import sys
import glob
import logging
from collections import deque, OrderedDict

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger('ros_to_lcm')

def parse_ros_msg_file(filepath):
    """Parse a ROS .msg file and return fields and constants."""
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Remove comments
    content = re.sub(r'#.*$', '', content, flags=re.MULTILINE)
    
    # Split into lines and remove empty lines
    lines = [line.strip() for line in content.split('\n') if line.strip()]
    
    fields = []
    constants = []
    
    for line in lines:
        # Check if line contains constant definition (contains =)
        if '=' in line:
            constants.append(line)
        else:
            fields.append(line)
    
    return fields, constants

def convert_ros_type_to_lcm(ros_type, known_types=None, dependencies=None):
    """
    Convert ROS type to LCM type.
    
    Args:
        ros_type: The ROS type to convert
        known_types: Set of types known to be in the same package
        dependencies: Set to track dependencies between message types
    
    Returns:
        The converted LCM type
    """
    if known_types is None:
        known_types = set()
    if dependencies is None:
        dependencies = set()
        
    # Simple type mapping
    type_map = {
        'bool': 'boolean',
        'int8': 'int8_t',
        'uint8': 'byte',
        'int16': 'int16_t',
        'uint16': 'int16_t',  # LCM doesn't have unsigned types
        'int32': 'int32_t',
        'uint32': 'int32_t',  # LCM doesn't have unsigned types
        'int64': 'int64_t',
        'uint64': 'int64_t',  # LCM doesn't have unsigned types
        'float32': 'float',
        'float64': 'double',
        'string': 'string',
        'char': 'byte',       # deprecated ROS type
        'byte': 'int8_t',     # deprecated ROS type
    }
    
    # Handle arrays: match both fixed and variable length arrays
    array_match = re.match(r'(.+?)(\[\d*\])+', ros_type)
    if array_match:
        base_type = array_match.group(1)
        
        # Convert base type
        if base_type in type_map:
            lcm_base_type = type_map[base_type]
        elif '/' in base_type:
            # Handle full package/message specification
            package, msg_type = base_type.split('/')
            lcm_base_type = f"{package}.{msg_type}"
            dependencies.add(base_type)
        else:
            # Assume it's a custom message type in the same package
            lcm_base_type = base_type
            if lcm_base_type not in known_types:
                dependencies.add(base_type)
        
        # We'll handle array dimensions separately in convert_fields
        return lcm_base_type
    
    # Handle special Header type
    if ros_type == 'Header':
        dependencies.add('std_msgs/Header')
        return 'std_msgs.Header'
        
    # Handle time and duration types
    if ros_type == 'time':
        dependencies.add('std_msgs/Time')
        return 'std_msgs.Time'
        
    if ros_type == 'duration':
        dependencies.add('std_msgs/Duration')
        return 'std_msgs.Duration'
    
    # Check for custom message type with package
    if '/' in ros_type:
        package, msg_type = ros_type.split('/')
        dependencies.add(ros_type)
        return f"{package}.{msg_type}"
    
    # Check for primitive type in map
    if ros_type in type_map:
        return type_map[ros_type]
    
    # Assume it's a custom message type in the same package
    if ros_type not in known_types:
        dependencies.add(ros_type)
    return ros_type

def scan_arrays_and_lengths(ros_fields):
    """
    Scan all fields for arrays and collect information about their dimensions.
    
    Returns:
        array_info: Dict of field_name -> (array dimensions, needed length variables)
        length_fields: Dict of length_var -> length_field_declaration
    """
    array_info = {}
    length_fields = {}
    
    # First pass: Find all fields that could serve as array lengths
    existing_length_fields = {}
    for field in ros_fields:
        parts = field.split()
        if len(parts) < 2:
            continue
            
        ros_type = parts[0]
        field_name = parts[1]
        
        # Only integer types can be array lengths
        if ros_type in ('int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64'):
            existing_length_fields[field_name] = convert_ros_type_to_lcm(ros_type)
    
    # Second pass: Find all arrays and their dimensions
    for field in ros_fields:
        parts = field.split()
        if len(parts) < 2:
            continue
            
        ros_type = parts[0]
        field_name = parts[1]
        
        # Check for array notation
        array_match = re.search(r'(\[\d*\])+', ros_type)
        if array_match:
            base_type = re.sub(r'(\[\d*\])+', '', ros_type)
            array_dims = re.findall(r'\[([^\]]*)\]', ros_type)
            
            needed_lengths = []
            dimensions = []
            
            for dim in array_dims:
                if dim == '':  # Variable length array
                    length_name = f"{field_name}_length"
                    needed_lengths.append(length_name)
                    dimensions.append(length_name)
                    
                    # Add length field if it doesn't exist
                    if length_name not in existing_length_fields:
                        length_fields[length_name] = f"int32_t {length_name}"
                else:
                    # Fixed size or using another variable
                    dimensions.append(dim)
                    
                    # Check if dimension is a variable reference
                    if not dim.isdigit() and dim not in existing_length_fields:
                        needed_lengths.append(dim)
                        length_fields[dim] = f"int32_t {dim}"
            
            array_info[field_name] = (dimensions, needed_lengths)
    
    return array_info, length_fields

def convert_field_with_array(ros_field, array_info):
    """
    Convert a ROS field with array syntax to LCM syntax
    In LCM, array dimensions come after the field name, not before it.
    """
    parts = ros_field.split()
    if len(parts) < 2:
        return ros_field  # Invalid field, return as is
        
    ros_type = parts[0]
    field_name = parts[1]
    
    # Check for array notation
    array_match = re.search(r'(\[\d*\])+', ros_type)
    if array_match and field_name in array_info:
        base_type = re.sub(r'(\[\d*\])+', '', ros_type)
        dimensions = array_info[field_name][0]
        
        # Convert base type
        lcm_type = convert_ros_type_to_lcm(base_type)
        
        # In LCM, array dimensions go after the field name
        lcm_field = f"{lcm_type} {field_name}"
        
        # Add array dimensions
        for dim in dimensions:
            lcm_field += f"[{dim}]"
        
        return lcm_field
    else:
        # Not an array, just convert the type
        lcm_type = convert_ros_type_to_lcm(ros_type)
        return f"{lcm_type} {field_name}"

def convert_fields(ros_fields, package_name):
    """Convert ROS fields to LCM fields with proper array length handling."""
    # Parse fields to extract custom types (for namespace handling)
    known_types = set()
    dependencies = set()
    
    for field in ros_fields:
        parts = field.split()
        if len(parts) >= 2:
            ros_type = parts[0]
            # Remove array markers to get base type
            base_type = re.sub(r'\[\d*\]', '', ros_type)
            if '/' not in base_type and base_type not in ('bool', 'int8', 'uint8', 'int16', 'uint16', 
                                                      'int32', 'uint32', 'int64', 'uint64', 
                                                      'float32', 'float64', 'string', 'time', 
                                                      'duration', 'char', 'byte', 'Header'):
                known_types.add(base_type)
    
    # Scan for arrays and collect needed length variables
    array_info, length_fields = scan_arrays_and_lengths(ros_fields)
    
    # Process fields with dependency tracking
    orderly_fields = []
    
    # First, add all length fields that arrays depend on
    # Create a dependency graph for array length variables
    field_deps = {}
    for field_name, (_, needed_lengths) in array_info.items():
        field_deps[field_name] = needed_lengths
    
    # Add all length fields first
    for length_name, length_decl in length_fields.items():
        orderly_fields.append(f"    {length_decl};")
    
    # Then add all non-array fields that aren't length fields
    for field in ros_fields:
        parts = field.split()
        if len(parts) < 2:
            continue
            
        field_name = parts[1]
        
        # Skip if field is a length field we already added
        if field_name in length_fields:
            continue
            
        # Process field
        lcm_field = convert_field_with_array(field, array_info)
        
        orderly_fields.append(f"    {lcm_field};")
    
    return orderly_fields, dependencies

def convert_constant(const_line):
    """
    Convert a ROS constant definition to LCM syntax.
    
    In LCM, constants can only be primitive types and have more restrictions.
    """
    parts = const_line.split('=', 1)
    if len(parts) < 2:
        return None  # Skip invalid lines
        
    type_and_name = parts[0].strip().split()
    if len(type_and_name) < 2:
        return None  # Skip invalid lines
        
    const_type = type_and_name[0]
    const_name = type_and_name[1]
    const_value = parts[1].strip()
    
    # Handle type conversions
    type_map = {
        'bool': 'boolean',
        'int8': 'int8_t',
        'uint8': 'int8_t',  # LCM doesn't support unsigned types for constants
        'byte': 'int8_t',   # 'byte' isn't a valid const type in LCM
        'char': 'int8_t',   # 'char' isn't a valid const type in LCM
        'int16': 'int16_t',
        'uint16': 'int16_t',  # LCM doesn't support unsigned types for constants
        'int32': 'int32_t',
        'uint32': 'int32_t',  # LCM doesn't support unsigned types for constants
        'int64': 'int64_t',
        'uint64': 'int64_t',  # LCM doesn't support unsigned types for constants
        'float32': 'float',
        'float64': 'double',
    }
    
    # Skip string constants as they're not supported in LCM
    if const_type == 'string':
        logger.warning(f"LCM doesn't support string constants. Skipping {const_name}.")
        return None
    
    # Convert type
    if const_type in type_map:
        lcm_type = type_map[const_type]
    else:
        logger.warning(f"Unsupported constant type '{const_type}' for '{const_name}'. Using int32_t instead.")
        lcm_type = 'int32_t'
    
    return f"    const {lcm_type} {const_name} = {const_value};"

def convert_constants(ros_constants):
    """Convert ROS constants to LCM constants."""
    lcm_constants = []
    
    for constant in ros_constants:
        lcm_constant = convert_constant(constant)
        if lcm_constant:
            lcm_constants.append(lcm_constant)
    
    return lcm_constants

def extract_package_and_type(ros_msg_path):
    """Extract package name and type name from ROS msg path."""
    # Try to determine package name from path structure
    path_parts = ros_msg_path.split(os.sep)
    if 'msg' in path_parts:
        # Standard ROS structure: .../my_package/msg/MyType.msg
        msg_index = path_parts.index('msg')
        if msg_index > 0:
            package_name = path_parts[msg_index - 1]
        else:
            package_name = "unknown_package"
    else:
        package_name = "unknown_package"
    
    # Get type name from filename without extension
    type_name = os.path.splitext(os.path.basename(ros_msg_path))[0]
    
    return package_name, type_name

def convert_ros_msg_to_lcm(ros_msg_path, output_dir=None, converted_types=None, conversion_queue=None):
    """Convert a ROS .msg file to an LCM .lcm file."""
    if converted_types is None:
        converted_types = {}
    if conversion_queue is None:
        conversion_queue = deque()
    
    # Extract package and type names
    package_name, type_name = extract_package_and_type(ros_msg_path)
    
    # Check if already converted
    type_key = f"{package_name}/{type_name}"
    if type_key in converted_types:
        return converted_types[type_key], set()
    
    # Extract fields and constants from ROS msg file
    ros_fields, ros_constants = parse_ros_msg_file(ros_msg_path)
    
    # Convert fields and constants
    lcm_fields, dependencies = convert_fields(ros_fields, package_name)
    lcm_constants = convert_constants(ros_constants)
    
    # Build LCM content
    lcm_content = [
        f"package {package_name};",
        "",
        f"struct {type_name} {{",
    ]
    
    if lcm_constants:
        lcm_content.extend(lcm_constants)
        if lcm_fields:
            lcm_content.append("")  # Add separator line
    
    lcm_content.extend(lcm_fields)
    lcm_content.append("}")
    
    lcm_content_str = "\n".join(lcm_content)
    
    # Write to output file - use package_name as prefix to avoid name collisions
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"{package_name}_{type_name}.lcm")
    else:
        output_path = f"{package_name}_{type_name}.lcm"
        
    with open(output_path, 'w') as f:
        f.write(lcm_content_str)
    
    logger.info(f"Converted {ros_msg_path} to {output_path}")
    
    # Add dependencies to conversion queue
    for dep in dependencies:
        if dep not in converted_types and dep not in conversion_queue:
            conversion_queue.append(dep)
    
    # Mark this type as converted - use the namespaced filename format
    converted_types[type_key] = output_path
    # Also add an entry for just the type name for backward compatibility
    converted_types[type_name] = output_path
    
    return output_path, dependencies

def find_ros_msg_files(directory):
    """Find all ROS .msg files in the directory and its subdirectories."""
    return glob.glob(os.path.join(directory, "**", "*.msg"), recursive=True)

def find_ros_msg_file(type_name, search_paths):
    """Find a ROS message file by its type name."""
    if '/' in type_name:
        # Full package/message specification
        package, msg = type_name.split('/')
        for path in search_paths:
            # Look for standard ROS structure
            msg_file = os.path.join(path, package, "msg", f"{msg}.msg")
            if os.path.exists(msg_file):
                return msg_file
    else:
        # Just message name, look in all packages
        for path in search_paths:
            if os.path.isdir(path):
                for package_dir in os.listdir(path):
                    msg_dir = os.path.join(path, package_dir, "msg")
                    if os.path.isdir(msg_dir):
                        msg_file = os.path.join(msg_dir, f"{type_name}.msg")
                        if os.path.exists(msg_file):
                            return msg_file
    
    # Special handling for Header
    if type_name == 'Header' or type_name == 'std_msgs/Header':
        # Create a temporary file for std_msgs/Header
        tmp_file = os.path.join(os.getcwd(), "Header.msg")
        with open(tmp_file, 'w') as f:
            f.write("""# Standard metadata for higher-level stamped data types.
uint32 seq
time stamp
string frame_id""")
        return tmp_file
    
    return None

def convert_ros_msgs(input_path, output_dir=None, search_paths=None):
    """
    Convert ROS message files to LCM, handling dependencies.
    
    Args:
        input_path: Path to a ROS message file or directory
        output_dir: Directory to write the output to
        search_paths: List of directories to search for dependencies
    
    Returns:
        Dictionary of converted types
    """
    if search_paths is None:
        search_paths = []
        
    # Add input path to search paths if it's a directory
    if os.path.isdir(input_path) and input_path not in search_paths:
        search_paths.append(input_path)
    
    # Find all message files
    if os.path.isdir(input_path):
        msg_files = find_ros_msg_files(input_path)
    else:
        msg_files = [input_path]
    
    # Dictionary to track converted types
    converted_types = {}
    
    # Queue for types that need to be converted
    conversion_queue = deque(msg_files)
    
    # Process all types in the queue
    while conversion_queue:
        # Get next type to convert
        current = conversion_queue.popleft()
        
        # Check if it's a file path or a type name
        if os.path.exists(current):
            # It's a file path
            ros_msg_path = current
        else:
            # It's a type name, find the file
            ros_msg_path = find_ros_msg_file(current, search_paths)
            if not ros_msg_path:
                logger.warning(f"Could not find message file for type {current}")
                continue
        
        # Convert the message
        try:
            _, deps = convert_ros_msg_to_lcm(ros_msg_path, output_dir, converted_types, conversion_queue)
        except Exception as e:
            logger.error(f"Error converting {ros_msg_path}: {e}")
            continue
    
    return converted_types

def main():
    if len(sys.argv) < 2:
        print("Usage: ros_to_lcm.py <ros_msg_file_or_directory> [output_directory] [search_path1] [search_path2] ...")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None
    search_paths = sys.argv[3:] if len(sys.argv) > 3 else []
    
    try:
        convert_ros_msgs(input_path, output_dir, search_paths)
        print("Conversion complete!")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()