import serial
import time
import re
import math
from tabulate import tabulate
import threading
import zmq
import json

class SensorData:
    def __init__(self, moving_average_window=3, calibration_window=100):
        self.sensor_data_window = []
        self.sensor_moving_averages = []
        self.calibration_data = []
        self.sensor_calibrated_averages = []
        self.cluster_displacements = {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []}
        self.cluster_angles = [0, 0, 240, 120]
        self.magnet_dist = .025
        self.cluster_distances = [0, self.magnet_dist, self.magnet_dist, self.magnet_dist]
        self.cluster_torques = {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []}
        self.forces = []
        self.torques = []
        self.moving_average_window = moving_average_window
        self.calibration_window = calibration_window
        self.calibrated = False
    
    def process_data(self, values):
        if any(x < 9000 or x > 21000 for x in values) or len(values) != 16:
            # print("Invalid values:", values)
            return
        self.sensor_data_window.append(values)
        if len(self.sensor_data_window) > self.calibration_window:
            self.sensor_data_window.pop(0)
        if len(self.sensor_data_window) == self.calibration_window and not self.calibrated:
            self.calibration_data = self.sensor_data_window.copy()
            self.sensor_calibrated_averages = [sum(x) / len(x) for x in zip(*self.calibration_data)]
            self.calibrated = True
            print("Calibrated Averages:", self.sensor_calibrated_averages)
        if len(self.sensor_data_window) > self.moving_average_window:
            self.sensor_moving_averages = [sum(x) / len(x) for x in zip(*self.sensor_data_window[-self.moving_average_window:])]
            if self.calibrated:
                self.sensor_moving_averages = [x - self.sensor_calibrated_averages[i] for i, x in enumerate(self.sensor_moving_averages)]
                self.calculate_displacements()

    def calculate_displacements(self):
        if len(self.sensor_moving_averages) != 16:
            return
        clusters = {
            "Middle": self.sensor_moving_averages[0:4],
            "Top": self.sensor_moving_averages[4:8],
            "Bottom Right": self.sensor_moving_averages[8:12],
            "Bottom Left": self.sensor_moving_averages[12:]
        }
        force_x = 0
        force_y = 0
        force_z = 0
        torque_x = 0
        torque_y = 0
        torque_z = 0
        for i, (name, cluster) in enumerate(clusters.items()):
            local_x_displacement = cluster[1] - cluster[3]
            local_y_displacement = cluster[0] - cluster[2]
            local_z_displacement = sum(cluster) / len(cluster)
            x_displacement = local_x_displacement * math.cos(math.radians(self.cluster_angles[i])) - local_y_displacement * math.sin(math.radians(self.cluster_angles[i]))
            y_displacement = local_x_displacement * math.sin(math.radians(self.cluster_angles[i])) + local_y_displacement * math.cos(math.radians(self.cluster_angles[i]))
            force_x += x_displacement
            force_y += y_displacement
            force_z += local_z_displacement
            position_vector = []
            position_vector.append(self.cluster_distances[i] * -math.sin(math.radians(self.cluster_angles[i])))
            position_vector.append(self.cluster_distances[i] * math.cos(math.radians(self.cluster_angles[i])))
            position_vector.append(0)
            # print(name, position_vector)
            local_torque_x = position_vector[1] * force_z - position_vector[2] * force_y
            local_torque_y = position_vector[2] * force_x - position_vector[0] * force_z
            local_torque_z = position_vector[0] * force_y - position_vector[1] * force_x
            # if name == "Middle":
                # print(tabulate([[local_torque_x, local_torque_y, local_torque_z]], headers=["Torque X", "Torque Y", "Torque Z"]))
                # print(tabulate([[x_displacement, y_displacement, local_z_displacement]], headers=["X Displacement", "Y Displacement", "Z Displacement"]))
            torque_x += local_torque_x
            torque_y += local_torque_y
            torque_z += local_torque_z
            self.cluster_displacements[name].append([x_displacement, y_displacement, local_z_displacement])
            if len(self.cluster_displacements[name]) > 100:
                self.cluster_displacements[name].pop(0)
            self.cluster_torques[name].append([torque_x, torque_y, torque_z])
            if len(self.cluster_torques[name]) > 100:
                self.cluster_torques[name].pop(0)
            # force_z = force_z / 4
        self.forces.append([force_x, force_y, force_z])
        if len(self.forces) > 100:
            self.forces.pop(0)
        self.torques.append([torque_x, torque_y, torque_z])
        if len(self.torques) > 100:
            self.torques.pop(0)
        print(tabulate([[force_x, force_y, force_z]], headers=["Force X", "Force Y", "Force Z"]))
        # print(tabulate([[torque_x, torque_y, torque_z]], headers=["Torque X", "Torque Y", "Torque Z"]))
    
    def get_data_dict(self):
        """Return all relevant data as a dictionary for transmission"""
        return {
            'sensor_data_window': self.sensor_data_window,
            'cluster_displacements': self.cluster_displacements,
            'cluster_torques': self.cluster_torques,
            'forces': self.forces,
            'torques': self.torques,
            'calibrated': self.calibrated
        }

class SerialReader:
    def __init__(self, sensor_data, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.sensor_data = sensor_data
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = True
        self.connect_serial()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Give the serial connection time to stabilize
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False

    def read_data(self):
        """Read a single data point from serial port"""
        if self.ser is None or not self.ser.is_open:
            if not self.connect_serial():
                time.sleep(3)  # Wait before retrying
                return
                
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip('\r\n,')
                if line:
                    values = [int(pair) for pair in re.split(r'[\t\n\r,]+', line)]
                    self.sensor_data.process_data(values)
        except (serial.SerialException, OSError) as e:
            print("Serial error:", e)
            if self.ser:
                self.ser.close()
            self.ser = None
            
    def stop(self):
        """Stop the serial reader"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


def main():
    # ZMQ setup for IPC
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5555")
    
    sensor_data = SensorData()
    serial_reader = SerialReader(sensor_data, port="/dev/ttyACM0")
    
    print("Starting data acquisition. Press Ctrl+C to stop.")
    
    try:
        while serial_reader.running:
            serial_reader.read_data()
            
            # Send data to visualization process
            data_dict = sensor_data.get_data_dict()
            socket.send_json(data_dict)
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
    
    except KeyboardInterrupt:
        print("Stopping data acquisition...")
    finally:
        serial_reader.stop()
        socket.close()
        context.term()
        print("Data acquisition stopped.")

if __name__ == "__main__":
    main()