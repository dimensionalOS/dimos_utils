#!/usr/bin/env python3
"""
Joystick node for LCM similar to ROS's Joy node.
This script reads joystick/gamepad inputs and publishes them to LCM.
It also supports calibration to handle different controller types.
"""

import argparse
import json
import os
import sys
import time
import pygame
import lcm
import numpy as np
from pathlib import Path

# Import LCM messages
try:
    from lcm_msgs.sensor_msgs import Joy
    from lcm_msgs.std_msgs import Header
except ImportError:
    print("Error: lcm_msgs not found. Make sure python_lcm_msgs is installed.")
    sys.exit(1)

class JoyNode:
    """Joystick node that publishes controller inputs to LCM."""
    
    DEFAULT_CONFIG_PATH = os.path.expanduser("~/.joy_lcm_config.json")
    DEFAULT_UPDATE_RATE = 50  # Hz
    DEFAULT_DEADZONE = 0.05
    DEFAULT_TOPIC = "joy#sensor_msgs.Joy"
    
    def __init__(self, config_path=None, update_rate=None):
        """Initialize the joystick node."""
        self.config_path = config_path or self.DEFAULT_CONFIG_PATH
        self.update_rate = update_rate or self.DEFAULT_UPDATE_RATE
        self.running = False
        self.joy = None
        self.joystick_id = 0
        self.config = None
        self.lc = lcm.LCM()
        
        # Initialize pygame for joystick handling
        pygame.init()
        pygame.joystick.init()
    
    def get_available_joysticks(self):
        """Get a list of available joysticks."""
        joysticks = []
        for i in range(pygame.joystick.get_count()):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            joysticks.append({
                'id': i,
                'name': joy.get_name(),
                'axes': joy.get_numaxes(),
                'buttons': joy.get_numbuttons(),
                'hats': joy.get_numhats()
            })
            joy.quit()  # Close the joystick
        return joysticks
    
    def list_joysticks(self):
        """List available joysticks and return selected joystick ID."""
        joysticks = self.get_available_joysticks()
        
        if not joysticks:
            print("No joysticks found. Please connect a controller and try again.")
            sys.exit(1)
        
        print("\nAvailable joysticks:")
        for joy in joysticks:
            print(f"  [{joy['id']}] {joy['name']} - {joy['axes']} axes, {joy['buttons']} buttons, {joy['hats']} hats")
        
        if len(joysticks) == 1:
            print(f"\nAutoselecting the only available joystick: {joysticks[0]['name']}")
            return joysticks[0]['id']
        
        # Let the user select a joystick if multiple are available
        selected = input("\nSelect a joystick by ID: ")
        try:
            selected_id = int(selected)
            if selected_id < 0 or selected_id >= len(joysticks):
                raise ValueError
            return selected_id
        except ValueError:
            print("Invalid selection. Please run again and select a valid joystick ID.")
            sys.exit(1)
    
    def initialize_joystick(self, joystick_id=None):
        """Initialize the selected joystick."""
        if joystick_id is None:
            joystick_id = self.list_joysticks()
        
        if pygame.joystick.get_count() <= joystick_id:
            print(f"Error: Joystick with ID {joystick_id} not found.")
            sys.exit(1)
        
        self.joystick_id = joystick_id
        self.joy = pygame.joystick.Joystick(joystick_id)
        self.joy.init()
        print(f"Initialized joystick: {self.joy.get_name()}")
        
        # Create or load config
        self.load_config()
    
    def load_config(self):
        """Load joystick configuration from file."""
        if not os.path.exists(self.config_path):
            print(f"Configuration file not found at {self.config_path}")
            print("Using default configuration. Run with --calibrate to create a custom configuration.")
            self.config = self.create_default_config()
            return
        
        try:
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
            
            # Check if config exists for this joystick
            joy_name = self.joy.get_name()
            if joy_name not in self.config:
                print(f"No configuration found for {joy_name}")
                print("Using default configuration. Run with --calibrate to create a custom configuration.")
                self.config[joy_name] = self.create_default_config_for_joystick()
                self.save_config()
            else:
                print(f"Loaded configuration for {joy_name}")
        except Exception as e:
            print(f"Error loading configuration: {e}")
            print("Using default configuration. Run with --calibrate to create a custom configuration.")
            self.config = self.create_default_config()
    
    def create_default_config(self):
        """Create a default configuration dictionary."""
        config = {}
        joy_name = self.joy.get_name()
        config[joy_name] = self.create_default_config_for_joystick()
        return config
    
    def create_default_config_for_joystick(self):
        """Create default configuration for the current joystick.
        
        Uses a best-guess mapping based on common game controller layouts.
        For Xbox-like controllers, this should provide a reasonable starting point.
        """
        axes_count = self.joy.get_numaxes()
        buttons_count = self.joy.get_numbuttons()
        
        # Start with empty mapping arrays
        axes_map = [-1] * 8  # Map for 8 standard axes
        axes_inverted = [False] * 8
        buttons_map = [-1] * 11  # Map for 11 standard buttons
        
        # Try to make a reasonable default mapping based on common controller layouts
        if axes_count >= 6:  # Typical for Xbox/PlayStation style controllers
            # Common mapping for Xbox-like controllers
            axes_map = [0, 1, 2, 3, 4, 5, -1, -1]  # First 6 mapped to sticks and triggers
            # Y axes are typically inverted on controllers (positive is down)
            axes_inverted = [False, True, False, True, False, False, False, False]
        
        if buttons_count >= 10:  # Typical for Xbox/PlayStation style controllers
            # Common mapping for first 10 buttons
            buttons_map = list(range(10))
            
        # Ensure arrays are the right length
        while len(axes_map) < 8:
            axes_map.append(-1)
        while len(axes_inverted) < 8:
            axes_inverted.append(False)
        while len(buttons_map) < 11:
            buttons_map.append(-1)
            
        return {
            "axes_map": axes_map,
            "axes_inverted": axes_inverted,
            "buttons_map": buttons_map,
            "deadzone": self.DEFAULT_DEADZONE
        }
    
    def save_config(self):
        """Save configuration to file."""
        os.makedirs(os.path.dirname(os.path.abspath(self.config_path)), exist_ok=True)
        with open(self.config_path, 'w') as f:
            json.dump(self.config, f, indent=4)
        print(f"Configuration saved to {self.config_path}")
    
    def calibrate(self):
        """Run the joystick calibration routine."""
        print("\n=== Joystick Calibration ===")
        print("This will guide you through calibrating your joystick.")
        print("Follow the prompts to move each axis and press each button.")
        
        joy_name = self.joy.get_name()
        joystick_config = {
            "axes_map": [],
            "axes_inverted": [],
            "buttons_map": [],
            "deadzone": self.DEFAULT_DEADZONE
        }
        
        # Calibrate axes
        print("\n--- Axis Calibration ---")
        print(f"Your joystick has {self.joy.get_numaxes()} axes and {self.joy.get_numbuttons()} buttons.")
        
        # Initialize with neutral positions
        pygame.event.pump()
        neutral_positions = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        print("Initial neutral positions:", [round(pos, 3) for pos in neutral_positions])
        
        # Create a list to track the max changes for each physical axis
        max_recorded_changes = [0.0] * self.joy.get_numaxes()
        # Reset axes map to ensure we start fresh
        joystick_config["axes_map"] = []
        joystick_config["axes_inverted"] = []
        
        for axis_idx in range(6):  # We first calibrate for the 6 analog stick axes
            if axis_idx >= len(self.get_logical_axis_name(0)):
                break
                
            # Ask the user to move a specific logical axis
            logical_name = self.get_logical_axis_name(axis_idx)
            print(f"\nCalibrating {logical_name} (index {axis_idx}):")
            print(f"  Move {logical_name} fully in both directions and then return to center.")
            print("  Press ENTER when ready, then move the axis, then press ENTER again when done.")
            input()
            
            # Monitor changes for a few seconds
            start_time = time.time()
            end_time = start_time + 3.0  # Monitor for 3 seconds
            
            print("  Recording axis movements for 3 seconds...")
            while time.time() < end_time:
                pygame.event.pump()
                current_positions = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
                changes = [abs(current - neutral) for current, neutral in 
                          zip(current_positions, neutral_positions)]
                
                # Update max changes recorded for each axis
                for i, change in enumerate(changes):
                    max_recorded_changes[i] = max(max_recorded_changes[i], change)
                
                time.sleep(0.05)  # Poll at 20Hz
            
            # Show the user all detected movements
            print("  Max changes detected:")
            for i, change in enumerate(max_recorded_changes):
                if change > 0.1:  # Lower threshold to show more potential axes
                    print(f"    Physical axis {i}: {change:.3f}")
            
            # Let user select which physical axis maps to this logical axis
            if max(max_recorded_changes) < 0.1:  # Lowered threshold for detection
                print("  No significant axis movement detected.")
                print("  Please manually enter the physical axis number to map:")
                try:
                    physical_idx = int(input("  Physical axis number (or -1 to skip): "))
                    if physical_idx >= self.joy.get_numaxes() or physical_idx < -1:
                        raise ValueError
                except ValueError:
                    print("  Invalid input. Skipping this axis.")
                    physical_idx = -1
            else:
                # Suggest the one with max change but let user confirm/override
                suggested_idx = max_recorded_changes.index(max(max_recorded_changes))
                print(f"  Suggested mapping: physical axis {suggested_idx}")
                manual_input = input("  Press ENTER to accept or enter a different physical axis number: ")
                
                if manual_input.strip():
                    try:
                        physical_idx = int(manual_input)
                        if physical_idx >= self.joy.get_numaxes() or physical_idx < -1:
                            raise ValueError
                    except ValueError:
                        print("  Invalid input. Using suggested mapping.")
                        physical_idx = suggested_idx
                else:
                    physical_idx = suggested_idx
            
            # Ask if the axis is inverted
            if physical_idx >= 0:
                invert_input = input("  Is this axis inverted? (y/n, default=n): ").lower()
                is_inverted = invert_input.startswith('y')
                
                joystick_config["axes_map"].append(physical_idx)
                joystick_config["axes_inverted"].append(is_inverted)
                print(f"  Mapped {logical_name} to physical axis {physical_idx} (inverted: {is_inverted})")
            else:
                joystick_config["axes_map"].append(-1)  # Use -1 to indicate unmapped axis
                joystick_config["axes_inverted"].append(False)
                print(f"  {logical_name} will be unmapped")
            
            # Reset max changes for the selected axis to avoid re-detecting it
            if physical_idx >= 0:
                max_recorded_changes[physical_idx] = 0.0
        
        # Handle D-pad (cross key) which can be buttons or axes depending on controller
        print("\n--- D-pad/Cross Key Calibration ---")
        print("The D-pad may be represented as axes (Xbox/Linux) or buttons (PlayStation).")
        print("This calibration will detect both types automatically.")
        
        # Initialize D-pad configuration
        if "dpad" not in joystick_config:
            joystick_config["dpad"] = {}
        
        # D-pad Left detection
        print("\nCalibrating D-pad LEFT:")
        print("  Press and hold the LEFT D-pad button, then release it.")
        print("  Press ENTER when ready.")
        input()
        
        # Track initial state
        pygame.event.pump()
        initial_axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        initial_buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        
        print("  Waiting for LEFT D-pad press...")
        left_axis_idx = -1
        left_axis_value = 0
        left_button_idx = -1
        
        # Monitor for changes for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:  # 5 seconds to press
            pygame.event.pump()
            
            # Check for axis changes
            for i in range(self.joy.get_numaxes()):
                current_value = self.joy.get_axis(i)
                change = current_value - initial_axes[i]
                
                # If significant negative change (left direction)
                if change < -0.5 and abs(change) > abs(left_axis_value):
                    left_axis_idx = i
                    left_axis_value = change
            
            # Check for button presses
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i) and not initial_buttons[i]:
                    left_button_idx = i
                    break
            
            # If we found a button or significant axis change, break early
            if left_button_idx >= 0 or left_axis_value < -0.5:
                break
                
            time.sleep(0.05)
            
        # Report what was detected
        if left_axis_idx >= 0 and left_axis_value < -0.5:
            print(f"  Detected LEFT on axis {left_axis_idx} with value {left_axis_value:.2f}")
        elif left_button_idx >= 0:
            print(f"  Detected LEFT on button {left_button_idx}")
        else:
            print("  No LEFT D-pad input detected. D-pad LEFT will be unmapped.")
            
        # D-pad Right detection    
        print("\nCalibrating D-pad RIGHT:")
        print("  Press and hold the RIGHT D-pad button, then release it.")
        print("  Press ENTER when ready.")
        input()
        
        # Track initial state
        pygame.event.pump()
        initial_axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        initial_buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        
        print("  Waiting for RIGHT D-pad press...")
        right_axis_idx = -1
        right_axis_value = 0
        right_button_idx = -1
        
        # Monitor for changes for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:  # 5 seconds to press
            pygame.event.pump()
            
            # Check for axis changes
            for i in range(self.joy.get_numaxes()):
                current_value = self.joy.get_axis(i)
                change = current_value - initial_axes[i]
                
                # If significant positive change (right direction)
                if change > 0.5 and abs(change) > abs(right_axis_value):
                    right_axis_idx = i
                    right_axis_value = change
            
            # Check for button presses
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i) and not initial_buttons[i]:
                    right_button_idx = i
                    break
            
            # If we found a button or significant axis change, break early
            if right_button_idx >= 0 or right_axis_value > 0.5:
                break
                
            time.sleep(0.05)
            
        # Report what was detected
        if right_axis_idx >= 0 and right_axis_value > 0.5:
            print(f"  Detected RIGHT on axis {right_axis_idx} with value {right_axis_value:.2f}")
        elif right_button_idx >= 0:
            print(f"  Detected RIGHT on button {right_button_idx}")
        else:
            print("  No RIGHT D-pad input detected. D-pad RIGHT will be unmapped.")
        
        # Store left/right configuration
        if (left_axis_idx >= 0 and right_axis_idx >= 0 and left_axis_idx == right_axis_idx):
            # It's an axis with -1 for left, +1 for right
            joystick_config["dpad"]["left_right_type"] = "axis"
            joystick_config["dpad"]["left_right_axis"] = left_axis_idx
            # Add to axes_map array
            joystick_config["axes_map"].append(left_axis_idx)
            joystick_config["axes_inverted"].append(False)  # No inversion needed
            print(f"  D-pad LEFT/RIGHT mapped to axis {left_axis_idx}")
        elif left_button_idx >= 0 and right_button_idx >= 0:
            # It's buttons
            joystick_config["dpad"]["left_right_type"] = "buttons"
            joystick_config["dpad"]["left_button"] = left_button_idx
            joystick_config["dpad"]["right_button"] = right_button_idx
            # Add placeholder to axes_map
            joystick_config["axes_map"].append(-1)
            joystick_config["axes_inverted"].append(False)
            print(f"  D-pad LEFT/RIGHT mapped to buttons {left_button_idx}/{right_button_idx}")
        else:
            print("  Incomplete LEFT/RIGHT D-pad mapping detected. Will be unmapped.")
            joystick_config["dpad"]["left_right_type"] = "none"
            # Add placeholder to axes_map
            joystick_config["axes_map"].append(-1)
            joystick_config["axes_inverted"].append(False)
            
        # D-pad Up detection
        print("\nCalibrating D-pad UP:")
        print("  Press and hold the UP D-pad button, then release it.")
        print("  Press ENTER when ready.")
        input()
        
        # Track initial state
        pygame.event.pump()
        initial_axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        initial_buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        
        print("  Waiting for UP D-pad press...")
        up_axis_idx = -1
        up_axis_value = 0
        up_button_idx = -1
        
        # Monitor for changes for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:  # 5 seconds to press
            pygame.event.pump()
            
            # Check for axis changes
            for i in range(self.joy.get_numaxes()):
                current_value = self.joy.get_axis(i)
                change = current_value - initial_axes[i]
                
                # If significant negative change (up direction, usually negative)
                if change < -0.5 and abs(change) > abs(up_axis_value):
                    up_axis_idx = i
                    up_axis_value = change
            
            # Check for button presses
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i) and not initial_buttons[i]:
                    up_button_idx = i
                    break
            
            # If we found a button or significant axis change, break early
            if up_button_idx >= 0 or up_axis_value < -0.5:
                break
                
            time.sleep(0.05)
            
        # Report what was detected
        if up_axis_idx >= 0 and up_axis_value < -0.5:
            print(f"  Detected UP on axis {up_axis_idx} with value {up_axis_value:.2f}")
        elif up_button_idx >= 0:
            print(f"  Detected UP on button {up_button_idx}")
        else:
            print("  No UP D-pad input detected. D-pad UP will be unmapped.")
            
        # D-pad Down detection    
        print("\nCalibrating D-pad DOWN:")
        print("  Press and hold the DOWN D-pad button, then release it.")
        print("  Press ENTER when ready.")
        input()
        
        # Track initial state
        pygame.event.pump()
        initial_axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        initial_buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]
        
        print("  Waiting for DOWN D-pad press...")
        down_axis_idx = -1
        down_axis_value = 0
        down_button_idx = -1
        
        # Monitor for changes for a few seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:  # 5 seconds to press
            pygame.event.pump()
            
            # Check for axis changes
            for i in range(self.joy.get_numaxes()):
                current_value = self.joy.get_axis(i)
                change = current_value - initial_axes[i]
                
                # If significant positive change (down direction)
                if change > 0.5 and abs(change) > abs(down_axis_value):
                    down_axis_idx = i
                    down_axis_value = change
            
            # Check for button presses
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i) and not initial_buttons[i]:
                    down_button_idx = i
                    break
            
            # If we found a button or significant axis change, break early
            if down_button_idx >= 0 or down_axis_value > 0.5:
                break
                
            time.sleep(0.05)
            
        # Report what was detected
        if down_axis_idx >= 0 and down_axis_value > 0.5:
            print(f"  Detected DOWN on axis {down_axis_idx} with value {down_axis_value:.2f}")
        elif down_button_idx >= 0:
            print(f"  Detected DOWN on button {down_button_idx}")
        else:
            print("  No DOWN D-pad input detected. D-pad DOWN will be unmapped.")
        
        # Store up/down configuration
        if (up_axis_idx >= 0 and down_axis_idx >= 0 and up_axis_idx == down_axis_idx):
            # It's an axis with -1 for up, +1 for down
            joystick_config["dpad"]["up_down_type"] = "axis"
            joystick_config["dpad"]["up_down_axis"] = up_axis_idx
            # Add to axes_map array
            joystick_config["axes_map"].append(up_axis_idx)
            joystick_config["axes_inverted"].append(False)  # No inversion needed
            print(f"  D-pad UP/DOWN mapped to axis {up_axis_idx}")
        elif up_button_idx >= 0 and down_button_idx >= 0:
            # It's buttons
            joystick_config["dpad"]["up_down_type"] = "buttons"
            joystick_config["dpad"]["up_button"] = up_button_idx
            joystick_config["dpad"]["down_button"] = down_button_idx
            # Add placeholder to axes_map
            joystick_config["axes_map"].append(-1)
            joystick_config["axes_inverted"].append(False)
            print(f"  D-pad UP/DOWN mapped to buttons {up_button_idx}/{down_button_idx}")
        else:
            print("  Incomplete UP/DOWN D-pad mapping detected. Will be unmapped.")
            joystick_config["dpad"]["up_down_type"] = "none"
            # Add placeholder to axes_map
            joystick_config["axes_map"].append(-1)
            joystick_config["axes_inverted"].append(False)
            
        # Calibrate standard buttons
        print("\n--- Button Calibration ---")
        print(f"Your joystick has {self.joy.get_numbuttons()} buttons.")
        print("Will now calibrate all 11 standard buttons.")
        
        # Reset buttons_map to ensure we start fresh
        joystick_config["buttons_map"] = []
        
        # Get the full list of standard button names upfront
        standard_button_count = 11  # We'll calibrate all 11 standard buttons
        button_names = [self.get_logical_button_name(i) for i in range(standard_button_count)]
        
        print("\nButtons to calibrate:")
        for i, name in enumerate(button_names):
            print(f"  {i}: {name}")
        
        print("\nPress ENTER to begin button calibration")
        input()
        
        # Initial button calibration prompt
        for button_idx in range(standard_button_count):
            logical_name = button_names[button_idx]
            print(f"\nCalibrating {logical_name} button (index {button_idx}):")
            print(f"  Press {logical_name} button and then release it.")
            print("  Press ENTER when ready, then press the button.")
            input()
            
            # Wait for a button press
            pressed_button = self.wait_for_button_press(timeout=5.0)
            if pressed_button is None:
                print("  No button press detected.")
                print("  Please manually enter the physical button number to map:")
                try:
                    physical_idx = int(input("  Physical button number (or -1 to skip): "))
                    if physical_idx >= self.joy.get_numbuttons() or physical_idx < -1:
                        raise ValueError
                except ValueError:
                    print("  Invalid input. Skipping this button.")
                    physical_idx = -1
            else:
                print(f"  Detected press on physical button {pressed_button}")
                
                # Let user confirm or override
                manual_input = input("  Press ENTER to accept or enter a different physical button number: ")
                
                if manual_input.strip():
                    try:
                        physical_idx = int(manual_input)
                        if physical_idx >= self.joy.get_numbuttons() or physical_idx < -1:
                            raise ValueError
                    except ValueError:
                        print("  Invalid input. Using detected button.")
                        physical_idx = pressed_button
                else:
                    physical_idx = pressed_button
            
            if physical_idx >= 0:
                joystick_config["buttons_map"].append(physical_idx)
                print(f"  Mapped {logical_name} to physical button {physical_idx}")
            else:
                joystick_config["buttons_map"].append(-1)  # Use -1 to indicate unmapped button
                print(f"  {logical_name} will be unmapped")
                
            # Show progress
            print(f"  Progress: {button_idx + 1}/{standard_button_count} buttons calibrated")
        
        # Set deadzone
        print("\n--- Deadzone Calibration ---")
        print("Let's set a deadzone value to ignore small unintentional axis movements.")
        print("Leave your controller untouched and press ENTER.")
        input()
        
        pygame.event.pump()
        neutral_noise = [abs(self.joy.get_axis(i) - neutral_positions[i]) 
                        for i in range(self.joy.get_numaxes())]
        suggested_deadzone = max(0.05, max(neutral_noise) * 2)  # Minimum 0.05, or double the max observed noise
        
        print(f"Suggested deadzone: {suggested_deadzone:.3f}")
        custom_deadzone = input("Enter a custom deadzone value or press ENTER to accept the suggestion: ")
        
        if custom_deadzone.strip():
            try:
                joystick_config["deadzone"] = float(custom_deadzone)
            except ValueError:
                print("Invalid input. Using suggested deadzone.")
                joystick_config["deadzone"] = suggested_deadzone
        else:
            joystick_config["deadzone"] = suggested_deadzone
        
        # Save the configuration
        if self.config is None:
            self.config = {}
        self.config[joy_name] = joystick_config
        self.save_config()
        
        print("\nCalibration complete!")
        print(f"Configuration saved for {joy_name}.")
        return True
    
    def wait_for_button_press(self, timeout=5.0):
        """Wait for a button press and return the button index."""
        print("  Waiting for button press...")
        
        # First, make sure all buttons are released to avoid false detection
        waiting_for_release = True
        release_start_time = time.time()
        release_timeout = 2.0  # Wait up to 2 seconds for all buttons to be released
        
        while waiting_for_release and time.time() - release_start_time < release_timeout:
            pygame.event.pump()
            any_pressed = False
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i):
                    any_pressed = True
                    break
            
            if not any_pressed:
                waiting_for_release = False
            else:
                time.sleep(0.05)
        
        # Now wait for a button press
        start_time = time.time()
        while time.time() - start_time < timeout:
            pygame.event.pump()
            for i in range(self.joy.get_numbuttons()):
                if self.joy.get_button(i):
                    print(f"  Button {i} pressed")
                    # Wait for release to avoid multiple detections
                    release_wait_start = time.time()
                    while time.time() - release_wait_start < 1.0:  # Wait up to 1 second for release
                        pygame.event.pump()
                        if not self.joy.get_button(i):
                            break
                        time.sleep(0.05)
                    return i
            time.sleep(0.05)
        
        print("  Timeout: No button press detected.")
        return None
    
    def get_logical_axis_name(self, index):
        """Get a human-readable name for a logical axis according to standard convention."""
        axis_names = [
            "Left/Right Axis stick left",  # 0
            "Up/Down Axis stick left",     # 1
            "Left/Right Axis stick right", # 2
            "Up/Down Axis stick right",    # 3
            "RT",                          # 4
            "LT",                          # 5
            "cross key left/right",        # 6
            "cross key up/down"            # 7
        ]
        if index < len(axis_names):
            return axis_names[index]
        return f"Axis {index}"
    
    def get_logical_button_name(self, index):
        """Get a human-readable name for a logical button according to standard convention."""
        button_names = [
            "A",                # 0
            "B",                # 1
            "X",                # 2
            "Y",                # 3
            "LB",               # 4
            "RB",               # 5
            "back",             # 6
            "start",            # 7
            "power",            # 8
            "Button stick left", # 9
            "Button stick right" # 10
        ]
        if index < len(button_names):
            return button_names[index]
        return f"Button {index}"
    
    def publish_joy_data(self):
        """Read joystick data and publish to LCM."""
        msg = Joy()
        
        # Set header
        msg.header = Header()
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
        msg.header.frame_id = ""
        
        # Prepare arrays
        joy_name = self.joy.get_name()
        joystick_config = self.config[joy_name]
        
        # Read raw values
        pygame.event.pump()
        
        # Map axes using the configuration
        axes_count = self.joy.get_numaxes()
        
        # Create a list with 8 elements (for all 8 standard axes)
        axes_raw = [0.0] * 8
        
        # Process regular axes
        for logical_idx, physical_idx in enumerate(joystick_config["axes_map"]):
            if logical_idx >= len(axes_raw):
                break
                
            if physical_idx >= 0 and physical_idx < axes_count:
                value = self.joy.get_axis(physical_idx)
                # Apply deadzone
                if abs(value) < joystick_config["deadzone"]:
                    value = 0.0
                # Apply inversion if configured
                if logical_idx < len(joystick_config["axes_inverted"]) and joystick_config["axes_inverted"][logical_idx]:
                    value = -value
                
                axes_raw[logical_idx] = value
        
        # Process D-pad inputs if configured
        if "dpad" in joystick_config:
            # Handle left/right D-pad
            if "left_right_type" in joystick_config["dpad"]:
                if joystick_config["dpad"]["left_right_type"] == "axis":
                    # D-pad is an axis
                    axis_idx = joystick_config["dpad"]["left_right_axis"]
                    if axis_idx >= 0 and axis_idx < axes_count:
                        axes_raw[6] = self.joy.get_axis(axis_idx)
                elif joystick_config["dpad"]["left_right_type"] == "buttons":
                    # D-pad is buttons
                    left_btn = joystick_config["dpad"]["left_button"]
                    right_btn = joystick_config["dpad"]["right_button"]
                    
                    # Convert button presses to axis values (-1.0 for left, 1.0 for right)
                    if left_btn >= 0 and left_btn < self.joy.get_numbuttons() and self.joy.get_button(left_btn):
                        axes_raw[6] = -1.0
                    elif right_btn >= 0 and right_btn < self.joy.get_numbuttons() and self.joy.get_button(right_btn):
                        axes_raw[6] = 1.0
                    else:
                        axes_raw[6] = 0.0
            
            # Handle up/down D-pad
            if "up_down_type" in joystick_config["dpad"]:
                if joystick_config["dpad"]["up_down_type"] == "axis":
                    # D-pad is an axis
                    axis_idx = joystick_config["dpad"]["up_down_axis"]
                    if axis_idx >= 0 and axis_idx < axes_count:
                        axes_raw[7] = self.joy.get_axis(axis_idx)
                elif joystick_config["dpad"]["up_down_type"] == "buttons":
                    # D-pad is buttons
                    up_btn = joystick_config["dpad"]["up_button"]
                    down_btn = joystick_config["dpad"]["down_button"]
                    
                    # Initial axis value (0.0 if no buttons pressed)
                    axes_raw[7] = 0.0
                    
                    # Convert button presses to axis values (-1.0 for up, 1.0 for down)
                    if up_btn >= 0 and up_btn < self.joy.get_numbuttons() and self.joy.get_button(up_btn):
                        axes_raw[7] = -1.0
                    elif down_btn >= 0 and down_btn < self.joy.get_numbuttons() and self.joy.get_button(down_btn):
                        axes_raw[7] = 1.0
                    
                    # Apply inversion if configured for this axis
                    if 7 < len(joystick_config["axes_inverted"]) and joystick_config["axes_inverted"][7]:
                        axes_raw[7] = -axes_raw[7]
        
        # For backward compatibility with configs that might have dpad_buttons
        elif "dpad_buttons" in joystick_config:
            # Handle left/right D-pad
            if "left_right" in joystick_config["dpad_buttons"]:
                left_btn = joystick_config["dpad_buttons"]["left_right"]["left"]
                right_btn = joystick_config["dpad_buttons"]["left_right"]["right"]
                
                # Convert button presses to axis values (-1.0 for left, 1.0 for right)
                if left_btn >= 0 and left_btn < self.joy.get_numbuttons() and self.joy.get_button(left_btn):
                    axes_raw[6] = -1.0
                elif right_btn >= 0 and right_btn < self.joy.get_numbuttons() and self.joy.get_button(right_btn):
                    axes_raw[6] = 1.0
                else:
                    axes_raw[6] = 0.0
            
            # Handle up/down D-pad
            if "up_down" in joystick_config["dpad_buttons"]:
                up_btn = joystick_config["dpad_buttons"]["up_down"]["up"]
                down_btn = joystick_config["dpad_buttons"]["up_down"]["down"]
                
                # Initial axis value (0.0 if no buttons pressed)
                axes_raw[7] = 0.0
                
                # Convert button presses to axis values (-1.0 for up, 1.0 for down)
                if up_btn >= 0 and up_btn < self.joy.get_numbuttons() and self.joy.get_button(up_btn):
                    axes_raw[7] = -1.0
                elif down_btn >= 0 and down_btn < self.joy.get_numbuttons() and self.joy.get_button(down_btn):
                    axes_raw[7] = 1.0
                
                # Apply inversion if configured for this axis
                if 7 < len(joystick_config["axes_inverted"]) and joystick_config["axes_inverted"][7]:
                    axes_raw[7] = -axes_raw[7]
        
        # Map buttons using the configuration
        buttons_count = self.joy.get_numbuttons()
        
        # Create a list with 11 elements (for all 11 standard buttons)
        buttons_raw = [0] * 11
        
        for logical_idx, physical_idx in enumerate(joystick_config["buttons_map"]):
            if logical_idx >= len(buttons_raw):
                break
                
            if physical_idx >= 0 and physical_idx < buttons_count:
                value = 1 if self.joy.get_button(physical_idx) else 0
                buttons_raw[logical_idx] = value
        
        # Set the message fields
        msg.axes = axes_raw
        msg.buttons = buttons_raw
        msg.axes_length = len(msg.axes)
        msg.buttons_length = len(msg.buttons)
        
        # Publish to LCM
        self.lc.publish(self.DEFAULT_TOPIC, msg.encode())
    
    def run(self):
        """Main loop to read and publish joystick data."""
        self.running = True
        print(f"Publishing joystick data to '{self.DEFAULT_TOPIC}' at {self.update_rate} Hz")
        print("Press Ctrl+C to stop")
        
        # Show configured mappings
        joy_name = self.joy.get_name()
        joystick_config = self.config[joy_name]
        
        print("\nCurrent configuration:")
        
        # Show axes mapping
        print("Axes mapping:")
        for i, physical_idx in enumerate(joystick_config["axes_map"]):
            if i < 6:  # Regular axes
                if physical_idx >= 0:
                    inverted = " (inverted)" if joystick_config["axes_inverted"][i] else ""
                    print(f"  {self.get_logical_axis_name(i)} -> Physical axis {physical_idx}{inverted}")
                else:
                    print(f"  {self.get_logical_axis_name(i)} -> Unmapped")
        
        # Show D-pad mapping
        if "dpad" in joystick_config:
            print("\nD-pad mapping:")
            if "left_right_type" in joystick_config["dpad"]:
                if joystick_config["dpad"]["left_right_type"] == "axis":
                    axis_idx = joystick_config["dpad"]["left_right_axis"]
                    print(f"  LEFT/RIGHT -> Axis {axis_idx}")
                elif joystick_config["dpad"]["left_right_type"] == "buttons":
                    left_btn = joystick_config["dpad"]["left_button"]
                    right_btn = joystick_config["dpad"]["right_button"]
                    print(f"  LEFT -> Button {left_btn}, RIGHT -> Button {right_btn}")
                
            if "up_down_type" in joystick_config["dpad"]:
                if joystick_config["dpad"]["up_down_type"] == "axis":
                    axis_idx = joystick_config["dpad"]["up_down_axis"]
                    print(f"  UP/DOWN -> Axis {axis_idx}")
                elif joystick_config["dpad"]["up_down_type"] == "buttons":
                    up_btn = joystick_config["dpad"]["up_button"]
                    down_btn = joystick_config["dpad"]["down_button"]
                    print(f"  UP -> Button {up_btn}, DOWN -> Button {down_btn}")
        
        # Show button mapping
        print("\nButton mapping:")
        for i, physical_idx in enumerate(joystick_config["buttons_map"]):
            if i < len(joystick_config["buttons_map"]):
                if physical_idx >= 0:
                    print(f"  {self.get_logical_button_name(i)} -> Physical button {physical_idx}")
                else:
                    print(f"  {self.get_logical_button_name(i)} -> Unmapped")
        
        period = 1.0 / self.update_rate
        try:
            while self.running:
                start_time = time.time()
                
                self.publish_joy_data()
                
                # Sleep to maintain the desired rate
                elapsed = time.time() - start_time
                sleep_time = max(0, period - elapsed)
                time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.joy.quit()
            pygame.quit()

def main():
    """Entry point for the joy node."""
    parser = argparse.ArgumentParser(description='LCM Joystick node similar to ROS Joy')
    parser.add_argument('--calibrate', action='store_true', help='Run joystick calibration')
    parser.add_argument('--config', type=str, help='Path to joystick configuration file')
    parser.add_argument('--rate', type=int, help='Publishing rate in Hz (default: 50)')
    parser.add_argument('--list', action='store_true', help='List available joysticks and exit')
    args = parser.parse_args()
    
    joy_node = JoyNode(config_path=args.config, update_rate=args.rate)
    
    # Just list joysticks if requested
    if args.list:
        joysticks = joy_node.get_available_joysticks()
        if not joysticks:
            print("No joysticks found.")
        else:
            print("Available joysticks:")
            for joy in joysticks:
                print(f"  [{joy['id']}] {joy['name']} - {joy['axes']} axes, {joy['buttons']} buttons, {joy['hats']} hats")
        return
    
    joy_node.initialize_joystick()
    
    # Run calibration if requested
    if args.calibrate:
        joy_node.calibrate()
    
    # Run the main loop
    joy_node.run()

if __name__ == '__main__':
    main()
