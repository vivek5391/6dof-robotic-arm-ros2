#!/usr/bin/env python3
import os
import json
import glob
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper_joint']
CALIB_PATH = '/home/vivek/robot_ws/src/arm_description/config/servo_calibration.json'


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)

        self.port_param = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.ser = None
        self.last_cmd = ''

        self.home_offsets = [0, 0, 0, 0, 0, 0]
        self.directions   = [1, 1, 1, 1, 1, 1]
        self.min_deg      = [0, 60, 25, 0, 30, 61]
        self.min_deg      = [0, 60, 25, 0, 30, 90]
        self.max_deg      = [180, 165, 160, 180, 150, 127]
        self.load_calibration()

        self.connect_serial()
        self.sub = self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_timer(2.0, self.periodic_check)

    def load_calibration(self):
        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH, 'r') as f:
                    data = json.load(f)
                self.home_offsets = data.get('home_offsets_deg', self.home_offsets)
                self.directions   = data.get('directions', self.directions)
                self.min_deg      = data.get('min_deg', self.min_deg)
                self.max_deg      = data.get('max_deg', self.max_deg)
            except Exception:
                pass

    def find_serial_port(self):
        if self.port_param != 'auto':
            return self.port_param
        ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        return ports[0] if ports else None

    def connect_serial(self):
        if not HAS_SERIAL:
            return

        port = self.find_serial_port()
        if not port:
            return

        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=0.05)
            self.get_logger().info(f'Connected to Arduino/ESP32 on {port} @ {self.baudrate} baud!')
        except Exception:
            self.ser = None

    def periodic_check(self):
        self.load_calibration()
        if HAS_SERIAL and self.ser is None:
            self.connect_serial()

    def on_joint_states(self, msg: JointState):
        name_map = dict(zip(msg.name, msg.position))
        if not all(j in name_map for j in JOINT_ORDER):
            return

        angles_deg = []
        for idx, j in enumerate(JOINT_ORDER):
            raw_deg = np.degrees(name_map[j]) * self.directions[idx]
            deg = int(round(90.0 + self.home_offsets[idx] + raw_deg))
            deg = max(self.min_deg[idx], min(self.max_deg[idx], deg))
            angles_deg.append(deg)

        cmd = f'{angles_deg[0]},{angles_deg[1]},{angles_deg[2]},{angles_deg[3]},{angles_deg[4]},{angles_deg[5]}\n'
        if cmd == self.last_cmd:
            return
        self.last_cmd = cmd

        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(cmd.encode('ascii'))
            except Exception:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
