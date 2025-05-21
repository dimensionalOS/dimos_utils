#!/usr/bin/env python3
import argparse
import lcm
import time
import math
import xml.etree.ElementTree as ET
import numpy as np
from threading import Thread, Lock
from lcm_msgs.sensor_msgs import JointState
from lcm_msgs.geometry_msgs import Transform, TransformStamped, Vector3, Quaternion
from lcm_msgs.tf2_msgs import TFMessage
from lcm_msgs.std_msgs import Header

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    # Ensure we have proper initialization
    if math.isnan(q.w):
        q.w = 1.0
    if math.isnan(q.x):
        q.x = 0.0
    if math.isnan(q.y):
        q.y = 0.0
    if math.isnan(q.z):
        q.z = 0.0
        
    return q

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions.
    """
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
    
    q = Quaternion()
    q.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    q.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    q.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    q.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return q

def transform_multiply(t1, t2):
    """
    Multiply two transforms.
    """
    t = Transform()
    
    # Rotate the translation part
    t.translation = Vector3()
    t.translation.x = t1.translation.x + t1.rotation.w * t1.rotation.w * t2.translation.x + \
                      2.0 * t1.rotation.y * t1.rotation.z * t2.translation.x - \
                      2.0 * t1.rotation.w * t1.rotation.z * t2.translation.y + \
                      2.0 * t1.rotation.w * t1.rotation.y * t2.translation.z + \
                      t1.rotation.x * t1.rotation.x * t2.translation.x + \
                      2.0 * t1.rotation.x * t1.rotation.y * t2.translation.y + \
                      2.0 * t1.rotation.x * t1.rotation.z * t2.translation.z - \
                      t1.rotation.z * t1.rotation.z * t2.translation.x - \
                      t1.rotation.y * t1.rotation.y * t2.translation.x
    
    t.translation.y = t1.translation.y + 2.0 * t1.rotation.w * t1.rotation.z * t2.translation.x + \
                      t1.rotation.w * t1.rotation.w * t2.translation.y - \
                      2.0 * t1.rotation.w * t1.rotation.x * t2.translation.z + \
                      t1.rotation.y * t1.rotation.y * t2.translation.y + \
                      2.0 * t1.rotation.y * t1.rotation.z * t2.translation.z - \
                      t1.rotation.z * t1.rotation.z * t2.translation.y + \
                      2.0 * t1.rotation.x * t1.rotation.y * t2.translation.x - \
                      t1.rotation.x * t1.rotation.x * t2.translation.y + \
                      2.0 * t1.rotation.x * t1.rotation.z * t2.translation.z
    
    t.translation.z = t1.translation.z - 2.0 * t1.rotation.w * t1.rotation.y * t2.translation.x + \
                      2.0 * t1.rotation.w * t1.rotation.x * t2.translation.y + \
                      t1.rotation.w * t1.rotation.w * t2.translation.z + \
                      2.0 * t1.rotation.x * t1.rotation.z * t2.translation.x - \
                      t1.rotation.x * t1.rotation.x * t2.translation.z + \
                      2.0 * t1.rotation.y * t1.rotation.z * t2.translation.y - \
                      t1.rotation.y * t1.rotation.y * t2.translation.z + \
                      t1.rotation.z * t1.rotation.z * t2.translation.z
    
    # Multiply the rotations
    t.rotation = quaternion_multiply(t1.rotation, t2.rotation)
    
    return t

class Joint:
    def __init__(self, name, parent, child, origin_xyz, origin_rpy, axis, joint_type, limit_lower=None, limit_upper=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.origin_xyz = origin_xyz
        self.origin_rpy = origin_rpy
        self.axis = axis
        self.type = joint_type
        self.position = 0.0
        self.limit_lower = limit_lower
        self.limit_upper = limit_upper

class Link:
    def __init__(self, name):
        self.name = name

class URDF:
    def __init__(self, urdf_path):
        self.joints = {}
        self.links = {}
        self.link_map = {}
        self.parse_urdf(urdf_path)
        
    def parse_urdf(self, urdf_path):
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # Parse the links
        for link_elem in root.findall('.//link'):
            link_name = link_elem.get('name')
            self.links[link_name] = Link(link_name)
        
        # Parse the joints
        for joint_elem in root.findall('.//joint'):
            joint_name = joint_elem.get('name')
            joint_type = joint_elem.get('type')
            
            parent_elem = joint_elem.find('parent')
            child_elem = joint_elem.find('child')
            
            if parent_elem is None or child_elem is None:
                continue
                
            parent_link = parent_elem.get('link')
            child_link = child_elem.get('link')
            
            # Store the relationship between parent and child links
            self.link_map[child_link] = joint_name
            
            # Parse the origin
            origin_elem = joint_elem.find('origin')
            origin_xyz = [0.0, 0.0, 0.0]
            origin_rpy = [0.0, 0.0, 0.0]
            
            if origin_elem is not None:
                if 'xyz' in origin_elem.attrib:
                    origin_xyz = [float(x) for x in origin_elem.get('xyz').split()]
                if 'rpy' in origin_elem.attrib:
                    origin_rpy = [float(x) for x in origin_elem.get('rpy').split()]
            
            # Parse the axis
            axis = [1.0, 0.0, 0.0]  # Default axis is along X
            axis_elem = joint_elem.find('axis')
            if axis_elem is not None and 'xyz' in axis_elem.attrib:
                axis = [float(x) for x in axis_elem.get('xyz').split()]
            
            # Parse joint limits
            limit_lower = None
            limit_upper = None
            limit_elem = joint_elem.find('limit')
            if limit_elem is not None:
                if 'lower' in limit_elem.attrib:
                    limit_lower = float(limit_elem.get('lower'))
                if 'upper' in limit_elem.attrib:
                    limit_upper = float(limit_elem.get('upper'))
            
            self.joints[joint_name] = Joint(
                joint_name, parent_link, child_link, 
                origin_xyz, origin_rpy, axis, joint_type,
                limit_lower, limit_upper
            )
    
    def get_root_link(self):
        # Find the link that has no parent
        for link_name in self.links:
            if link_name not in self.link_map:
                return link_name
        return list(self.links.keys())[0]  # Default to first link if no root found
    
    def get_link_tree(self):
        # Create tree structure from root
        root_link = self.get_root_link()
        tree = {root_link: []}
        
        # Function to recursively build the tree
        def build_tree(parent_link):
            children = []
            for joint_name, joint in self.joints.items():
                if joint.parent == parent_link:
                    child_link = joint.child
                    children.append(child_link)
                    if child_link not in tree:
                        tree[child_link] = build_tree(child_link)
            return children
        
        # Build the tree starting from root
        tree[root_link] = build_tree(root_link)
        return tree, root_link

class RobotStatePublisher:
    def __init__(self, urdf_path, fixed_frame="world", publish_rate=50.0, enforce_limits=True):
        self.urdf = URDF(urdf_path)
        self.fixed_frame = fixed_frame
        self.publish_rate = publish_rate
        self.enforce_limits = enforce_limits
        self.joint_state = {}
        self.mutex = Lock()
        self.lc = lcm.LCM()
        self.running = True
        
        print(f"Joint limit enforcement is {'enabled' if enforce_limits else 'disabled'}")
        if enforce_limits:
            print("Joint limits from URDF:")
            for joint_name, joint in self.urdf.joints.items():
                if joint.limit_lower is not None or joint.limit_upper is not None:
                    limit_info = []
                    if joint.limit_lower is not None:
                        limit_info.append(f"lower={joint.limit_lower:.3f}")
                    if joint.limit_upper is not None:
                        limit_info.append(f"upper={joint.limit_upper:.3f}")
                    print(f"  {joint_name}: {', '.join(limit_info)}")
        
        # Start LCM handling thread
        self.lcm_thread = Thread(target=self._lcm_thread)
        self.lcm_thread.daemon = True
        self.lcm_thread.start()
        
        # Start publishing thread
        self.publish_thread = Thread(target=self._publish_transforms)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
    def _lcm_thread(self):
        def joint_state_handler(channel, data):
            try:
                msg = JointState.decode(data)
                print(f"Received joint state update with {msg.name_length} joints")
                
                with self.mutex:
                    for i in range(msg.name_length):
                        if i < len(msg.name) and i < len(msg.position):
                            joint_name = msg.name[i]
                            joint_position = msg.position[i]
                            
                            # Enforce joint limits if enabled
                            if self.enforce_limits and joint_name in self.urdf.joints:
                                joint = self.urdf.joints[joint_name]
                                
                                # Apply lower limit if needed
                                if joint.limit_lower is not None and joint_position < joint.limit_lower:
                                    joint_position = joint.limit_lower
                                    print(f"Limiting {joint_name} to lower bound: {joint_position:.3f}")
                                
                                # Apply upper limit if needed
                                if joint.limit_upper is not None and joint_position > joint.limit_upper:
                                    joint_position = joint.limit_upper
                                    print(f"Limiting {joint_name} to upper bound: {joint_position:.3f}")
                            
                            # Update joint state with potentially limited position
                            self.joint_state[joint_name] = joint_position
            except Exception as e:
                print(f"Error processing joint state message: {e}")
        
        self.lc.subscribe("joint_states#sensor_msgs.JointState", joint_state_handler)
        
        try:
            while self.running:
                self.lc.handle()
        except KeyboardInterrupt:
            pass
    
    def _publish_transforms(self):
        # Get the link tree
        link_tree, root_link = self.urdf.get_link_tree()
        print(f"URDF root link: {root_link}")
        print(f"URDF joint count: {len(self.urdf.joints)}")
        for joint_name in self.urdf.joints:
            print(f"  Joint: {joint_name}")
        
        try:
            while self.running:
                # Compute transforms
                transforms = []
                
                # Always add a transform from fixed_frame to root_link (if they're different)
                if self.fixed_frame != root_link:
                    transform = TransformStamped()
                    transform.header = self._get_time()
                    transform.header.frame_id = self.fixed_frame
                    transform.child_frame_id = root_link
                    
                    transform.transform = Transform()
                    transform.transform.translation = Vector3()
                    transform.transform.rotation = Quaternion()
                    transform.transform.translation.x = 0.0
                    transform.transform.translation.y = 0.0
                    transform.transform.translation.z = 0.0
                    transform.transform.rotation.x = 0.0
                    transform.transform.rotation.y = 0.0
                    transform.transform.rotation.z = 0.0
                    transform.transform.rotation.w = 1.0
                    
                    transforms.append(transform)
                
                # Add transforms for all other joints
                with self.mutex:
                    # Copy the joint state to avoid lock contention
                    joint_state_copy = self.joint_state.copy()
                
                for joint_name, joint in self.urdf.joints.items():
                    # Get joint position (default to 0 if not found)
                    position = joint_state_copy.get(joint_name, 0.0)
                    
                    # Create transform message
                    transform = TransformStamped()
                    transform.header = self._get_time()
                    transform.header.frame_id = joint.parent
                    transform.child_frame_id = joint.child
                    
                    transform.transform = Transform()
                    transform.transform.translation = Vector3()
                    transform.transform.rotation = Quaternion()
                    
                    # Set translation from joint origin
                    transform.transform.translation.x = joint.origin_xyz[0]
                    transform.transform.translation.y = joint.origin_xyz[1]
                    transform.transform.translation.z = joint.origin_xyz[2]
                    
                    # Set rotation for fixed transforms (from RPY)
                    q_fixed = quaternion_from_euler(
                        joint.origin_rpy[0], joint.origin_rpy[1], joint.origin_rpy[2]
                    )
                    
                    # For rotational joints, apply the joint value as a rotation around the axis
                    if joint.type == "revolute" or joint.type == "continuous":
                        ax, ay, az = joint.axis
                        angle = position
                        
                        # Create quaternion for the joint rotation
                        s = math.sin(angle/2)
                        q_joint = Quaternion()
                        q_joint.x = ax * s
                        q_joint.y = ay * s
                        q_joint.z = az * s
                        q_joint.w = math.cos(angle/2)
                        
                        # Combine the fixed transform with the joint transform
                        transform.transform.rotation = quaternion_multiply(q_fixed, q_joint)
                    
                    # For prismatic joints, translate along the axis
                    elif joint.type == "prismatic":
                        ax, ay, az = joint.axis
                        distance = position
                        
                        # Create rotation matrix from fixed quaternion
                        # This is needed to correctly orient the prismatic joint axis
                        qw, qx, qy, qz = q_fixed.w, q_fixed.x, q_fixed.y, q_fixed.z
                        
                        # Rotate the axis vector by the fixed quaternion
                        # Formula for rotating a vector v by quaternion q: q*v*q^-1 (simplified)
                        dx = (ax * (qw*qw + qx*qx - qy*qy - qz*qz) + 
                              ay * (2*qx*qy - 2*qw*qz) + 
                              az * (2*qx*qz + 2*qw*qy)) * distance
                        
                        dy = (ax * (2*qx*qy + 2*qw*qz) + 
                              ay * (qw*qw - qx*qx + qy*qy - qz*qz) + 
                              az * (2*qy*qz - 2*qw*qx)) * distance
                        
                        dz = (ax * (2*qx*qz - 2*qw*qy) + 
                              ay * (2*qy*qz + 2*qw*qx) + 
                              az * (qw*qw - qx*qx - qy*qy + qz*qz)) * distance
                        
                        # Apply the rotated translation
                        transform.transform.translation.x += dx
                        transform.transform.translation.y += dy
                        transform.transform.translation.z += dz
                        transform.transform.rotation = q_fixed
                    
                    # For fixed joints, just use the fixed rotation
                    else:  # "fixed" or other types
                        transform.transform.rotation = q_fixed
                    
                    transforms.append(transform)
                
                # Create and publish TFMessage
                tf_msg = TFMessage()
                tf_msg.transforms = transforms
                tf_msg.transforms_length = len(transforms)
                
                # Ensure all transforms are properly initialized
                for i, transform in enumerate(tf_msg.transforms):
                    # Ensure transform.transform is initialized
                    if not hasattr(transform, 'transform') or transform.transform is None:
                        transform.transform = Transform()
                    
                    # Ensure translation is initialized
                    if not hasattr(transform.transform, 'translation') or transform.transform.translation is None:
                        transform.transform.translation = Vector3()
                    
                    # Ensure rotation is initialized with proper identity quaternion
                    if not hasattr(transform.transform, 'rotation') or transform.transform.rotation is None:
                        transform.transform.rotation = Quaternion()
                        transform.transform.rotation.w = 1.0
                
                try:
                    self.lc.publish("tf#tf2_msgs.TFMessage", tf_msg.encode())
                    if len(transforms) > 0:
                        print(f"Published {len(transforms)} transforms")
                except Exception as e:
                    print(f"Error publishing transforms: {e}")
                
                # Sleep to maintain publishing rate
                time.sleep(1.0 / self.publish_rate)
        
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Error in transform publishing thread: {e}")
            import traceback
            traceback.print_exc()
    
    def _get_time(self):
        # Create a timestamp
        header = Header()
        header.seq = 0  # Sequence number
        header.stamp.sec = int(time.time())  # Seconds since epoch (integer required)
        header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
        header.frame_id = self.fixed_frame
        return header
    
    def stop(self):
        self.running = False
        self.lcm_thread.join(1.0)
        self.publish_thread.join(1.0)

def main():
    parser = argparse.ArgumentParser(description='LCM Robot State Publisher')
    parser.add_argument('--urdf', type=str, help='Path to URDF file')
    parser.add_argument('--fixed-frame', type=str, default='world', help='Name of the fixed frame')
    parser.add_argument('--rate', type=float, default=50.0, help='Publishing rate in Hz')
    parser.add_argument('--enforce-limits', type=bool, default=True, help='Enforce joint limits from URDF')
    
    args = parser.parse_args()
    
    # If URDF is not specified, look for common locations
    urdf_path = args.urdf
    if not urdf_path:
        # Try to find a URDF file in common locations
        potential_paths = [
            "../assets/devkit_base_descr.urdf",
        ]
        
        for path in potential_paths:
            try:
                with open(path, 'r') as f:
                    urdf_path = path
                    print(f"Found URDF file at: {urdf_path}")
                    break
            except FileNotFoundError:
                continue
    
    if not urdf_path:
        print("Error: No URDF file specified and none found in common locations.")
        print("Please specify a URDF file using the --urdf argument.")
        return
    
    try:
        publisher = RobotStatePublisher(
            urdf_path, 
            args.fixed_frame, 
            args.rate,
            enforce_limits=args.enforce_limits
        )
        
        print(f"Robot state publisher started. Publishing tf data at {args.rate} Hz.")
        print(f"Using URDF file: {urdf_path}")
        print("Press Ctrl+C to exit.")
        
        # Keep the main thread alive
        while True:
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        print("\nShutting down robot state publisher.")

if __name__ == "__main__":
    main()