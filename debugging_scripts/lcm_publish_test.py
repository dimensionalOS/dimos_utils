import lcm
from lcm_msgs.geometry_msgs import Twist, Vector3
import time


msg2 = Vector3()
msg2.x = 1
msg2.y = 1
msg2.z = 1
msg3 = Twist()
msg3.linear = Vector3()
msg3.linear.x = 1
msg3.linear.y = 1
msg3.linear.z = 1

lc = lcm.LCM()
lc.publish("thing1_vector3#geometry_msgs.Vector3", msg2.encode())
lc.publish("thing1_twist#geometry_msgs.Twist", msg3.encode())

while True:
    msg2.x += 1
    msg2.y += 1
    msg2.z += 1
    lc.publish("thing1_vector3#geometry_msgs.Vector3", msg2.encode())
    msg3.linear.x += 1
    msg3.linear.y += 1
    msg3.linear.z += 1
    lc.publish("thing1_twist#geometry_msgs.Twist", msg3.encode())
    time.sleep(.1)