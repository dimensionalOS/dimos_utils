#!/usr/bin/env python3
"""
LCM Image Viewer (Qt version) - Subscribes to LCM Image messages and displays them in real time
using PyQt for a more reliable GUI experience
"""

import os
import sys
import time
import argparse
import threading
import numpy as np
import cv2
import lcm
from datetime import datetime

# Set up the path to access the ROS message definitions
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# Import the standard message headers
from sensor_msgs.Image import Image

# Import Qt components - ensure these are installed
try:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QGridLayout
    from PyQt5.QtGui import QImage, QPixmap
    from PyQt5.QtCore import Qt, QTimer
except ImportError:
    print("Error: PyQt5 is required for this viewer. Install with:")
    print("pip install PyQt5")
    sys.exit(1)

class ImageWidget(QLabel):
    """Widget for displaying an image with overlays"""
    
    def __init__(self, parent=None, topic_name=""):
        super().__init__(parent)
        self.topic_name = topic_name
        self.fps = 0
        self.frame_count = 0
        self.setText(f"Waiting for images on\n{topic_name}")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("QLabel { background-color: black; color: white; }")
    
    def update_image(self, img, fps):
        """Update the displayed image and FPS"""
        self.fps = fps
        self.frame_count += 1
        
        height, width = img.shape[0], img.shape[1]
        
        # Add FPS overlay to the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        img_with_overlay = img.copy()
        cv2.putText(img_with_overlay, f"FPS: {fps:.1f}", (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # If this is a grayscale image, convert to RGB for Qt
        if len(img.shape) == 2 or img.shape[2] == 1:
            img_with_overlay = cv2.cvtColor(img_with_overlay, cv2.COLOR_GRAY2RGB)
        elif img.shape[2] == 3:
            img_with_overlay = cv2.cvtColor(img_with_overlay, cv2.COLOR_BGR2RGB)
        elif img.shape[2] == 4:
            img_with_overlay = cv2.cvtColor(img_with_overlay, cv2.COLOR_BGRA2RGBA)
            
        # Create QImage from numpy array
        bytes_per_line = img_with_overlay.shape[1] * img_with_overlay.shape[2]
        q_img = QImage(img_with_overlay.data, width, height, bytes_per_line, QImage.Format_RGB888)
        
        # Create pixmap and set to label
        pixmap = QPixmap.fromImage(q_img)
        self.setPixmap(pixmap)
        self.adjustSize()

class LCMImageViewer(QMainWindow):
    """
    A Qt-based viewer for LCM image messages that supports multiple topics and formats
    """
    
    def __init__(self, topics=None, window_names=None):
        """
        Initialize the LCM image viewer
        
        Args:
            topics: List of LCM topics to subscribe to
            window_names: Custom window names (if None, will use topic names)
        """
        super().__init__()
        
        # Set up topics
        self.topics = topics if topics else []
        
        # Create window names (either use custom names or topic names)
        if window_names and len(window_names) == len(topics):
            self.window_names = window_names
        else:
            self.window_names = [f"LCM Viewer: {topic}" for topic in self.topics]
        
        # Set up the GUI
        self.setWindowTitle("LCM Image Viewer")
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        
        # Create grid layout for multiple image widgets
        self.layout = QGridLayout(self.central_widget)
        
        # Create image widgets for each topic
        self.image_widgets = {}
        rows = int(np.ceil(np.sqrt(len(self.topics))))
        cols = int(np.ceil(len(self.topics) / rows))
        
        for i, topic in enumerate(self.topics):
            row = i // cols
            col = i % cols
            widget = ImageWidget(topic_name=topic)
            self.image_widgets[topic] = widget
            self.layout.addWidget(widget, row, col)
        
        # Initialize LCM
        self.lc = lcm.LCM()
        
        # Track the latest images
        self.latest_images = {topic: None for topic in self.topics}
        self.image_lock = threading.Lock()
        
        # Stats tracking
        self.frame_counts = {topic: 0 for topic in self.topics}
        self.start_times = {topic: None for topic in self.topics}
        self.last_frame_times = {topic: None for topic in self.topics}
        
        # Subscribe to all topics
        self.subscriptions = []
        for topic in self.topics:
            sub = self.lc.subscribe(topic, self.on_image)
            self.subscriptions.append(sub)
        
        # Set up timer for updating GUI and handling LCM messages
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(30)  # Update every 33ms (about 30 FPS)
        
        # Set up LCM handling thread
        self.running = True
        self.lcm_thread = threading.Thread(target=self.lcm_loop)
        self.lcm_thread.daemon = True
    
    def start(self):
        """Start the viewer"""
        self.lcm_thread.start()
        self.show()
        print(f"Listening for LCM messages on topics: {', '.join(self.topics)}")
        
    def lcm_loop(self):
        """Thread to handle LCM messages"""
        try:
            while self.running:
                self.lc.handle_timeout(100)  # Timeout in ms
        except Exception as e:
            print(f"LCM thread error: {e}")
    
    def on_image(self, channel, data):
        """
        Callback when an LCM image message is received
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
        """
        if channel not in self.topics:
            return
            
        try:
            # Decode the image message
            msg = Image.decode(data)
            
            # Track timing for FPS calculation
            current_time = time.time()
            if self.start_times[channel] is None:
                self.start_times[channel] = current_time
                
            self.last_frame_times[channel] = current_time
            self.frame_counts[channel] += 1
            
            # Convert the data to a numpy array based on encoding
            encoding = msg.encoding
            
            # Get image dimensions
            height = msg.height
            width = msg.width
            
            try:
                # Validate dimensions before reshaping
                if msg.data_length <= 0 or height <= 0 or width <= 0:
                    raise ValueError(f"Invalid image dimensions: {width}x{height}, data_length={msg.data_length}")
                
                # Make sure we have enough data for the specified dimensions
                expected_size = 0
                if encoding.startswith('mono') or encoding == 'uint8':
                    expected_size = height * width
                elif encoding in ['rgb8', 'bgr8']:
                    expected_size = height * width * 3
                elif encoding == 'bgra8' or encoding == 'rgba8':
                    expected_size = height * width * 4
                else:
                    # Default to bgr8
                    expected_size = height * width * 3
                    
                if len(msg.data) < expected_size:
                    raise ValueError(f"Image data too small: got {len(msg.data)} bytes, expected {expected_size}")
                
                # Convert bytes to numpy array
                if encoding.startswith('mono') or encoding == 'uint8':
                    # Grayscale image
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
                elif encoding in ['rgb8', 'bgr8']:
                    # 3-channel color image
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                elif encoding == 'bgra8' or encoding == 'rgba8':
                    # 4-channel color image with alpha
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 4))
                else:
                    # Default to bgr8
                    print(f"Warning: Unknown encoding: {encoding}, defaulting to bgr8")
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
            except ValueError as e:
                print(f"Error converting image data: {e}")
                print(f"Message info: {width}x{height}, encoding={encoding}, data_length={msg.data_length}")
                return  # Skip this frame
            
            # Store the image
            with self.image_lock:
                self.latest_images[channel] = img
                
        except Exception as e:
            print(f"Error processing image on channel {channel}: {e}")
            import traceback
            traceback.print_exc()
    
    def update_displays(self):
        """Update all image displays with latest data"""
        # Process LCM messages (backup method if thread fails)
        try:
            self.lc.handle_timeout(0)
        except:
            pass
            
        # Make a copy of the latest images to avoid threading issues
        images_to_display = {}
        with self.image_lock:
            for topic in self.topics:
                images_to_display[topic] = self.latest_images[topic]
        
        # Update each image widget
        for topic, img in images_to_display.items():
            if img is not None:
                # Calculate FPS
                fps = 0
                if self.frame_counts[topic] > 0 and self.start_times[topic] is not None:
                    elapsed = time.time() - self.start_times[topic]
                    fps = self.frame_counts[topic] / elapsed if elapsed > 0 else 0
                
                # Update the widget
                self.image_widgets[topic].update_image(img, fps)
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.stop()
        event.accept()
        
    def stop(self):
        """Stop the viewer"""
        self.running = False
        
        # Clean up LCM
        for sub in self.subscriptions:
            try:
                self.lc.unsubscribe(sub)
            except:
                pass
        
        # Stop timer
        self.update_timer.stop()
        
        # Report statistics
        print("\nPerformance Summary:")
        for topic in self.topics:
            if self.frame_counts[topic] > 0 and self.start_times[topic] is not None:
                elapsed = time.time() - self.start_times[topic]
                avg_fps = self.frame_counts[topic] / elapsed if elapsed > 0 else 0
                print(f"Topic {topic}: Received {self.frame_counts[topic]} frames "
                      f"in {elapsed:.1f}s ({avg_fps:.1f} fps)")

def main():
    parser = argparse.ArgumentParser(description='View LCM Image messages with Qt GUI')
    parser.add_argument('topics', nargs='+', help='LCM topics to subscribe to')
    parser.add_argument('--names', nargs='+', help='Custom window names (optional)')
    args = parser.parse_args()
    
    # Validate provided topics
    if not args.topics:
        print("Error: You must specify at least one topic to subscribe to")
        parser.print_help()
        return 1
        
    # Validate window names if provided
    if args.names and len(args.names) != len(args.topics):
        print(f"Error: Number of window names ({len(args.names)}) must match number of topics ({len(args.topics)})")
        parser.print_help()
        return 1
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Print information
    print(f"Starting LCM Image Viewer for {len(args.topics)} topic(s):")
    for i, topic in enumerate(args.topics):
        window_name = args.names[i] if args.names and i < len(args.names) else f"LCM Viewer: {topic}"
        print(f"  {topic} -> {window_name}")
    
    # Create and show the viewer
    viewer = LCMImageViewer(topics=args.topics, window_names=args.names)
    viewer.start()
    
    # Run the application
    return_code = app.exec_()
    
    # Clean up
    viewer.stop()
    print("Viewer stopped cleanly")
    
    return return_code

if __name__ == "__main__":
    sys.exit(main())