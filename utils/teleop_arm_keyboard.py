#!/usr/bin/env python

from __future__ import print_function

import threading
import sys
import argparse
import time
from select import select
import lcm
from lcm_msgs.geometry_msgs import Twist, Vector3, TwistStamped
from lcm_msgs import std_msgs

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Robot Arm Teleop Keyboard Control
---------------------------
Position Control:
   w : +x (forward)
   s : -x (backward)
   a : +y (left)
   d : -y (right)
   e : +z (up)
   q : -z (down)

Orientation Control:
   i : +pitch (up)
   k : -pitch (down)
   j : +yaw (left)
   l : -yaw (right)
   u : +roll (counterclockwise)
   o : -roll (clockwise)

Speed Control:
   z/x : increase/decrease overall speed by 0.1
   c/v : increase/decrease position speed by 0.1
   b/n : increase/decrease orientation speed by 0.1

CTRL-C to quit
"""

# Key bindings for position and orientation control
# Format: (x, y, z, roll, pitch, yaw)
moveBindings = {
    'w': (1, 0, 0, 0, 0, 0),    # +x (forward)
    's': (-1, 0, 0, 0, 0, 0),   # -x (backward)
    'a': (0, 1, 0, 0, 0, 0),    # +y (left)
    'd': (0, -1, 0, 0, 0, 0),   # -y (right)
    'e': (0, 0, 1, 0, 0, 0),    # +z (up)
    'q': (0, 0, -1, 0, 0, 0),   # -z (down)
    'i': (0, 0, 0, 0, 1, 0),    # +pitch (up)
    'k': (0, 0, 0, 0, -1, 0),   # -pitch (down)
    'j': (0, 0, 0, 0, 0, 1),    # +yaw (left)
    'l': (0, 0, 0, 0, 0, -1),   # -yaw (right)
    'u': (0, 0, 0, 1, 0, 0),    # +roll (counterclockwise)
    'o': (0, 0, 0, -1, 0, 0),   # -roll (clockwise)
}

# Speed adjustment bindings
# Format: (overall scale, position scale, orientation scale)
speedBindings = {
    'z': (0.1, 0, 0),      # Increase overall speed
    'x': (-0.1, 0, 0),     # Decrease overall speed
    'c': (0, 0.1, 0),      # Increase position speed
    'v': (0, -0.1, 0),     # Decrease position speed
    'b': (0, 0, 0.1),      # Increase orientation speed
    'n': (0, 0, -0.1),     # Decrease orientation speed
}


class PublishThread(threading.Thread):
    def __init__(self, rate, topic_name, stamped=False, frame_id=""):
        super(PublishThread, self).__init__()
        self.lc = lcm.LCM()
        self.topic_name = topic_name
        # Position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        # Orientation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Speed scaling
        self.pos_speed = 0.5
        self.rot_speed = 0.5
        self.condition = threading.Condition()
        self.done = False
        self.stamped = stamped
        self.frame_id = frame_id

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, z, roll, pitch, yaw, pos_speed, rot_speed):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.pos_speed = pos_speed
        self.rot_speed = rot_speed
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        if self.stamped:
            twist_stamped_msg = TwistStamped()
            twist_stamped_msg.header = std_msgs.Header()
            twist_stamped_msg.header.frame_id = self.frame_id
            twist_stamped_msg.twist = Twist()
            twist_stamped_msg.twist.linear = Vector3()
            twist_stamped_msg.twist.angular = Vector3()
            twist = twist_stamped_msg.twist
            msg = twist_stamped_msg
        else:
            twist_msg = Twist()
            twist_msg.linear = Vector3()
            twist_msg.angular = Vector3()
            twist = twist_msg
            msg = twist_msg
        
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            if self.stamped:
                # Update timestamp
                current_time = time.time()
                twist_stamped_msg.header.stamp.sec = int(current_time)
                twist_stamped_msg.header.stamp.nsec = int((current_time - int(current_time)) * 1e9)
            
            # Copy state into twist message.
            # Linear portion is for position
            twist.linear.x = self.x * self.pos_speed
            twist.linear.y = self.y * self.pos_speed
            twist.linear.z = self.z * self.pos_speed
            # Angular portion is for orientation
            twist.angular.x = self.roll * self.rot_speed
            twist.angular.y = self.pitch * self.rot_speed
            twist.angular.z = self.yaw * self.rot_speed

            self.condition.release()

            # Publish.
            self.lc.publish(self.topic_name, msg.encode())

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.lc.publish(self.topic_name, msg.encode())


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(pos_speed, rot_speed):
    return "currently:\tposition speed %s\torientation speed %s " % (pos_speed, rot_speed)

if __name__=="__main__":
    settings = saveTerminalSettings()

    # Setup command line arguments
    parser = argparse.ArgumentParser(description='Keyboard teleop for Robot Arm Position Control')
    parser.add_argument('--pos_speed', type=float, default=0.5,
                        help='Initial position velocity (default: 0.5)')
    parser.add_argument('--rot_speed', type=float, default=0.5,
                        help='Initial orientation velocity (default: 0.5)')
    parser.add_argument('--max_speed', type=float, default=1.0,
                        help='Max velocity (default: 1.0)')
    parser.add_argument('--repeat_rate', type=float, default=0.0,
                        help='Rate at which to repeat the last command (0 for no repeat, default: 0.0)')
    parser.add_argument('--key_timeout', type=float, default=0.1,
                        help='Timeout for key detection (default: 0.1)')
    parser.add_argument('--topic', type=str, default='cmd_pos_diff#geometry_msgs.Twist',
                        help='LCM topic name (default: cmd_pos_diff#geometry_msgs.Twist)')
    parser.add_argument('--stamped', action='store_true',
                        help='Use TwistStamped message instead of Twist')
    parser.add_argument('--frame_id', type=str, default='',
                        help='Frame ID for TwistStamped messages (default: empty string)')
    
    args = parser.parse_args()
    
    pos_speed = args.pos_speed
    rot_speed = args.rot_speed
    max_speed = args.max_speed
    repeat = args.repeat_rate
    key_timeout = args.key_timeout
    topic_name = args.topic
    stamped = args.stamped
    frame_id = args.frame_id
    
    # Update topic name if using stamped message
    if stamped and 'Twist' in topic_name and not 'TwistStamped' in topic_name:
        topic_name = topic_name.replace('Twist', 'TwistStamped')

    pub_thread = PublishThread(repeat, topic_name, stamped, frame_id)

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
    status = 0

    try:
        pub_thread.update(x, y, z, roll, pitch, yaw, pos_speed, rot_speed)

        print(msg)
        print(vels(pos_speed, rot_speed))
        # Keep track of the previous key pressed
        prev_key = ''
        
        while(1):
            key = getKey(settings, key_timeout)
            
            # If no key is pressed but a key was pressed before, reset movement
            if key == '' and prev_key != '':
                x = 0
                y = 0
                z = 0
                roll = 0
                pitch = 0
                yaw = 0
                pub_thread.update(x, y, z, roll, pitch, yaw, pos_speed, rot_speed)
                prev_key = ''
                continue
                
            # Skip if no key change
            if key == '' and prev_key == '':
                continue
                
            # Update the previous key
            prev_key = key
            
            # Handle movement keys
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                roll = moveBindings[key][3]
                pitch = moveBindings[key][4]
                yaw = moveBindings[key][5]
            
            # Handle speed adjustment keys
            elif key in speedBindings.keys():
                # Apply overall speed adjustment
                if speedBindings[key][0] != 0:
                    pos_speed = max(0.1, min(max_speed, pos_speed + speedBindings[key][0]))
                    rot_speed = max(0.1, min(max_speed, rot_speed + speedBindings[key][0]))
                
                # Apply position speed adjustment
                if speedBindings[key][1] != 0:
                    pos_speed = max(0.1, min(max_speed, pos_speed + speedBindings[key][1]))
                
                # Apply orientation speed adjustment
                if speedBindings[key][2] != 0:
                    rot_speed = max(0.1, min(max_speed, rot_speed + speedBindings[key][2]))
                
                if pos_speed == max_speed:
                    print("Position speed limit reached!")
                if rot_speed == max_speed:
                    print("Orientation speed limit reached!")
                
                print(vels(pos_speed, rot_speed))
                
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            
            # Handle stop or exit
            else:
                # Exit on CTRL-C
                if (key == '\x03'):
                    break
                    
                # Set all movement to zero for any other key
                x = 0
                y = 0
                z = 0
                roll = 0
                pitch = 0
                yaw = 0

            pub_thread.update(x, y, z, roll, pitch, yaw, pos_speed, rot_speed)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
