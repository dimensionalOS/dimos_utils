#!/usr/bin/env python3
"""
Simple LCM monitor that listens for all messages on tf and tf_static channels.
This helps diagnose if messages are being broadcasted properly.
"""

import sys
import time
import lcm
import traceback

def my_handler(channel, data):
    """Print information about received LCM messages"""
    print(f"Received message on channel: {channel}")
    print(f"  Message size: {len(data)} bytes")
    # Print first few bytes as hex
    print(f"  First 16 bytes: {data[:16].hex()}")

def main():
    print("=== LCM Message Monitor ===")
    print("Listening for ANY LCM messages on tf and tf_static channels")
    print("This will help diagnose if transforms are being broadcasted at all")
    
    try:
        # Create LCM instance
        lc = lcm.LCM()
        
        # Subscribe to tf channels with wildcard handlers that don't try to decode
        lc.subscribe("tf", my_handler)
        lc.subscribe("tf_static", my_handler)
        print("Subscribed to 'tf' and 'tf_static' channels")
        
        # Also try with # symbol which is sometimes used in LCM channel naming
        lc.subscribe("tf#tf2_msgs.TFMessage", my_handler)
        lc.subscribe("tf_static#tf2_msgs.TFMessage", my_handler)
        print("Subscribed to 'tf#tf2_msgs.TFMessage' and 'tf_static#tf2_msgs.TFMessage' channels")
        
        print("\nWaiting for messages...")
        print("(Press Ctrl+C to exit)")
        
        # Main loop
        message_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 60:  # Run for up to 60 seconds
            # Process any incoming messages
            lc.handle_timeout(100)  # 100ms timeout
            
            # Brief sleep to avoid hammering the CPU
            time.sleep(0.01)
        
        print("\nNo messages received after 60 seconds.")
        
    except KeyboardInterrupt:
        print("\nExiting due to keyboard interrupt")
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
