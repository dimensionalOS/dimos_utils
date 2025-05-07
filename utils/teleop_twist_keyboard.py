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
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate, topic_name, stamped=False, frame_id=""):
        super(PublishThread, self).__init__()
        self.lc = lcm.LCM()
        self.topic_name = topic_name
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
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

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
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
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

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

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    # Setup command line arguments
    parser = argparse.ArgumentParser(description='Keyboard teleop for LCM')
    parser.add_argument('--speed', type=float, default=0.5,
                        help='Initial linear velocity (default: 0.5)')
    parser.add_argument('--turn', type=float, default=1.0,
                        help='Initial angular velocity (default: 1.0)')
    parser.add_argument('--speed_limit', type=float, default=1000.0,
                        help='Max linear velocity (default: 1000.0)')
    parser.add_argument('--turn_limit', type=float, default=1000.0,
                        help='Max angular velocity (default: 1000.0)')
    parser.add_argument('--repeat_rate', type=float, default=0.0,
                        help='Rate at which to repeat the last command (0 for no repeat, default: 0.0)')
    parser.add_argument('--key_timeout', type=float, default=0.5,
                        help='Timeout for key detection (default: 0.5)')
    parser.add_argument('--topic', type=str, default='cmd_vel#geometry_msgs.Twist',
                        help='LCM topic name (default: cmd_vel#geometry_msgs.Twist)')
    parser.add_argument('--stamped', action='store_true',
                        help='Use TwistStamped message instead of Twist')
    parser.add_argument('--frame_id', type=str, default='',
                        help='Frame ID for TwistStamped messages (default: empty string)')
    
    args = parser.parse_args()
    
    speed = args.speed
    turn = args.turn
    speed_limit = args.speed_limit
    turn_limit = args.turn_limit
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
    th = 0
    status = 0

    try:
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)