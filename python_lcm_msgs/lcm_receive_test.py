import lcm
from geometry_msgs import Twist, Vector3

def my_handler(channel, data):
    if channel == "TWIST" or channel == "TWIST2":
        msg = Twist.decode(data)
        print("Received message on channel:", channel)
        print("Linear velocity:", msg.linear.x, msg.linear.y, msg.linear.z)
        print("Angular velocity:", msg.angular.x, msg.angular.y, msg.angular.z)
    elif channel == "VECTOR" or channel == "thing1_vector3":
        msg = Vector3.decode(data)
        print("Received message on channel:", channel)
        print("Vector:", msg.x, msg.y, msg.z)

lc = lcm.LCM()
lc.subscribe(".*", my_handler)
print("Waiting for messages...")

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    print("Exiting...")
except lcm.LCMError as e:
    print("LCM error:", e)
except Exception as e:
    print("An error occurred:", e)
finally:
    lc.unsubscribe("TWIST")
    print("Unsubscribed from channel.")
    lc.close()