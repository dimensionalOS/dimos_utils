import time
import lcm
import tkinter as tk
from tkinter import ttk
import random
from lcm_msgs.sensor_msgs import JointState

def update_positions():
    # Update position values from sliders
    for i, slider in enumerate(sliders):
        msg.position[i] = slider.get()
    
    # Update header timestamp
    msg.header.stamp.sec = int(time.time())
    msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
    # Publish the updated joint state
    lc.publish("joint_states#sensor_msgs.JointState", msg.encode())
    
    # Schedule the next update
    root.after(10, update_positions)  # 10ms = 0.01s

def reset_positions():
    # Reset all sliders to 0.0
    for slider in sliders:
        slider.set(0.0)

def randomize_positions():
    # Set each slider to a random value between -1.0 and 1.0
    for slider in sliders:
        random_value = random.uniform(-1.0, 1.0)
        slider.set(random_value)

# Create the LCM message
msg = JointState()
msg.header.stamp.sec = int(time.time())
msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
msg.header.frame_id = "base_link"
msg.name = ["pillar_platform_joint", "pan_tilt_pan_joint", "pan_tilt_head_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"]
msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
msg.velocity = []  # Empty list by default
msg.effort = []    # Empty list by default
msg.name_length = len(msg.name)
msg.position_length = len(msg.position)
msg.velocity_length = len(msg.velocity)
msg.effort_length = len(msg.effort)

# Initialize LCM
lc = lcm.LCM()

# Create GUI window
root = tk.Tk()
root.title("Joint Position Control")

# Function factory to create update functions for each slider
def create_update_func(label):
    return lambda val: label.config(text=f"{float(val):.2f}")

# Create main frame
main_frame = ttk.Frame(root, padding="10")
main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Create sliders for each joint
sliders = []
labels = []
for i, joint_name in enumerate(msg.name):
    # Create a frame for each joint
    frame = ttk.Frame(main_frame, padding="5")
    frame.grid(row=i, column=0, sticky=(tk.W, tk.E))
    
    # Add label with joint name
    ttk.Label(frame, text=joint_name, width=20).grid(column=0, row=0, sticky=tk.W, padx=5)
    
    # Add slider
    slider = ttk.Scale(frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, length=300)
    slider.grid(column=1, row=0, sticky=(tk.W, tk.E))
    slider.set(0.0)  # Initial value
    
    # Add value label
    value_label = ttk.Label(frame, text="0.00", width=5)
    value_label.grid(column=2, row=0, sticky=tk.E, padx=5)
    
    # Create update function and bind to slider
    update_func = create_update_func(value_label)
    slider.config(command=update_func)
    
    sliders.append(slider)
    labels.append(value_label)

# Add control buttons
button_frame = ttk.Frame(main_frame, padding="5")
button_frame.grid(row=len(msg.name), column=0, sticky=(tk.W, tk.E))

# Reset button
reset_button = ttk.Button(button_frame, text="Reset Positions", command=reset_positions)
reset_button.grid(column=0, row=0, padx=5, pady=10)

# Randomize button
randomize_button = ttk.Button(button_frame, text="Randomize Positions", command=randomize_positions)
randomize_button.grid(column=1, row=0, padx=5, pady=10)

# Quit button
quit_button = ttk.Button(button_frame, text="Quit", command=root.destroy)
quit_button.grid(column=2, row=0, padx=5, pady=10)

# Start the update loop
update_positions()

# Start the GUI event loop
root.mainloop()