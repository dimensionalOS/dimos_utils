import lcm
from lcm_msgs.geometry_msgs import Twist, Vector3
from lcm_msgs.sensor_msgs import JointState
from lcm_msgs.tf2_msgs import TFMessage

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
    elif channel == "joint_states":
        msg = JointState.decode(data)
        print("Received message on channel:", channel)
        print("Joint names:", msg.name)
        print("Joint positions:", msg.position)
        print("Joint velocities:", msg.velocity)
        print("Joint efforts:", msg.effort)
    elif channel == "tf":
        msg = TFMessage.decode(data)
        print("Received message on channel:", channel)
        print("TF Message:")
        for transform in msg.transforms:
            print(transform.header.stamp, transform.header.frame_id, transform.child_frame_id, transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

lc = lcm.LCM()
lc.subscribe(".*", my_handler)
print("Waiting for messages...")

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    print("Exiting...")
except Exception as e:
    print("An error occurred:", e)
finally:
    lc.unsubscribe(".*")
    print("Unsubscribed from channel.")
    lc.close()