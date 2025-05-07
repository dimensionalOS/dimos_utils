#!/usr/bin/env python3
"""
LCM Video Publisher - Publishes frames from an MP4 file as sensor_msgs/Image messages
"""

import os
import sys
import time
import argparse
import cv2
import numpy as np
import lcm
from datetime import datetime

# Set up the path to access the ROS message definitions
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# Import the standard message headers
from std_msgs.Header import Header
from sensor_msgs.Image import Image

def cv2_to_sensor_msgs_image(cv_image, encoding="bgr8", seq=0):
    """
    Convert an OpenCV image to a sensor_msgs/Image message
    
    Args:
        cv_image: OpenCV image (numpy array)
        encoding: Image encoding (default: bgr8)
        seq: Sequence number (default: 0)
    
    Returns:
        sensor_msgs/Image message
    """
    # Create an Image message
    img_msg = Image()
    
    # Create a header
    img_msg.header = Header()
    img_msg.header.seq = seq
    img_msg.header.stamp = int(time.time() * 1e9)  # Current time in nanoseconds
    img_msg.header.frame_id = "world"
    
    # Set image properties
    height, width, channels = cv_image.shape
    img_msg.height = height
    img_msg.width = width
    img_msg.encoding = encoding
    img_msg.is_bigendian = 0  # Typically 0 for Intel/AMD
    
    # Set step and data
    img_msg.step = width * channels  # Full row length in bytes
    img_msg.data = cv_image.tobytes()  # Convert image to byte array
    img_msg.data_length = len(img_msg.data)  # Required by the LCM type definition
    
    return img_msg

def publish_video_frames(video_path, topic_name, loop=True, fps=30, num_frames=60, 
                     color_mode='bgr8', grayscale=False):
    """
    Publish frames from a video file as sensor_msgs/Image messages
    
    Args:
        video_path: Path to the MP4 file
        topic_name: LCM topic name to publish on
        loop: Whether to loop the video
        fps: Frames per second to publish
        num_frames: Number of frames to publish (will loop within these frames)
        color_mode: Color mode ("bgr8" or "rgb8" or "bgra8")
        grayscale: If True, convert to grayscale
    """
    # Check if the file exists
    if not os.path.exists(video_path):
        print(f"Error: File not found: {video_path}")
        return
    
    # Open the video file
    video = cv2.VideoCapture(video_path)
    if not video.isOpened():
        print(f"Error: Unable to open video file: {video_path}")
        return
    
    # Get video properties
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    original_fps = video.get(cv2.CAP_PROP_FPS)
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"Video info: {width}x{height}, {original_fps} fps, {total_frames} frames")
    
    # Calculate frames to use
    frames_to_use = min(num_frames, total_frames)
    print(f"Using {frames_to_use} frames from the video")
    
    # Read the frames we want to use
    print("Loading frames...")
    frames = []
    for i in range(frames_to_use):
        ret, frame = video.read()
        if not ret:
            break
            
        # Apply color conversion if needed
        if grayscale:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Add the channel dimension for consistency
            frame = frame[:, :, np.newaxis]
            color_mode = 'mono8'
        elif color_mode == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif color_mode == 'bgra8':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        # bgr8 is the default from OpenCV
            
        frames.append(frame)
    
    # Release the video file
    video.release()
    
    # Calculate timing
    frame_delay = 1.0 / fps  # Time between frames in seconds
    
    # Create an LCM instance
    lc = lcm.LCM()
    
    print(f"Publishing to topic '{topic_name}' at {fps} fps")
    print("Press Ctrl+C to stop")
    
    frame_index = 0
    start_time = time.time()
    frames_published = 0
    seq = 0  # Sequence number for header
    
    try:
        while True:
            # Get the next frame (with looping)
            frame = frames[frame_index]
            frame_index = (frame_index + 1) % len(frames)
            
            # Convert to sensor_msgs/Image with incrementing sequence number
            img_msg = cv2_to_sensor_msgs_image(frame, encoding=color_mode, seq=seq)
            seq += 1
            
            # Publish the message
            lc.publish(topic_name, img_msg.encode())
            frames_published += 1
            
            # Calculate elapsed time for FPS control
            elapsed = time.time() - start_time
            if elapsed >= 1.0:  # Print stats every second
                print(f"Published {frames_published} frames in {elapsed:.2f}s ({frames_published/elapsed:.1f} fps)")
                start_time = time.time()
                frames_published = 0
            
            # Sleep to maintain the desired FPS
            time.sleep(frame_delay)
            
            # Break if not looping and we've gone through all frames
            if not loop and frame_index == 0:
                print("Reached end of video, exiting")
                break
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    print("Publisher stopped")

def main():
    parser = argparse.ArgumentParser(description='Publish video frames as LCM sensor_msgs/Image messages')
    parser.add_argument('video_path', help='Path to the MP4 file')
    parser.add_argument('topic_name', help='LCM topic name to publish on')
    parser.add_argument('--fps', type=float, default=30.0, help='Frames per second to publish (default: 30)')
    parser.add_argument('--no-loop', action='store_false', dest='loop', help='Do not loop the video')
    parser.add_argument('--frames', type=int, default=60, help='Number of frames to use (default: 60)')
    parser.add_argument('--color', choices=['bgr8', 'rgb8', 'bgra8'], default='bgr8', 
                        help='Color mode (default: bgr8)')
    parser.add_argument('--grayscale', action='store_true', help='Convert to grayscale')
    args = parser.parse_args()
    
    publish_video_frames(
        args.video_path, 
        args.topic_name, 
        loop=args.loop, 
        fps=args.fps, 
        num_frames=args.frames,
        color_mode=args.color,
        grayscale=args.grayscale
    )

if __name__ == "__main__":
    main()