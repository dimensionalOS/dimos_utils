import lcm
from geometry_msgs import Twist, Vector3
import time


msg2 = Vector3()
msg2.x = 1
msg2.y = 1
msg2.z = 1

lc = lcm.LCM()
lc.publish("thing1_vector3", msg2.encode())

while True:
    msg2.x += 1
    msg2.y += 1
    msg2.z += 1
    lc.publish("thing1_vector3", msg2.encode())
    time.sleep(.1)