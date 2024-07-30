#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import sys
import select
import termios
import tty

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_setpoints')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Subscriber for camera image
        self.camera_subscription = self.create_subscription(
            Image, '/x500_mono_cam/camera/image_raw', self.camera_callback, 10)
        
        self.bridge = CvBridge()

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -2.3
        self.setpoints = [
            (0.0, 0.0, self.takeoff_height,  1.5708 ),               
            (-3.0, 0.0, self.takeoff_height, 1.5708),       
            (-3.0, 4.8, -1.3, 3.14159),
            (-5.0, 4.8, -1.3, 3.14159),
            (-5.5, 4.8, -1.1, 3.14159),    
            (-5.5, 3.5, -1.1, 3.14159),     
            (0.0, 0.0, self.takeoff_height, 1.5708) 
        ]
        self.current_setpoint_index = 0
        self.hover_duration = 15  # Time to hover in seconds
        self.hover_start_time = None

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.2, self.timer_callback)  # Publish every 200ms (5Hz)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        self.get_logger().info(f'Current position: x={vehicle_local_position.x}, y={vehicle_local_position.y}, z={vehicle_local_position.z}')

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def camera_callback(self, msg):
        """Callback function for camera image topic subscriber."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Drone Camera", cv_image)
        cv2.waitKey(1)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint with yaw."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [0.0, 0.0, 0.0]  # Set the desired velocity components
        msg.yaw = yaw  # Set the desired yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoint: {[x, y, z]}, yaw: {yaw}")

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw: float):
        """Publish the trajectory setpoint with velocity components."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]  # Use NaN to indicate no position setpoint
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoint: {[vx, vy, vz]}, yaw: {yaw}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter > 10:
            if self.current_setpoint_index < len(self.setpoints):
                sp = self.setpoints[self.current_setpoint_index]
                # Calcular velocidade desejada com base na distância para o próximo ponto
                vx = (sp[0] - self.vehicle_local_position.x)* 0.3   # Fator de escala reduzido para diminuir a velocidade
                vy = (sp[1] - self.vehicle_local_position.y)* 0.3  
                vz = (sp[2] - self.vehicle_local_position.z)* 0.3  
                self.publish_velocity_setpoint(vx, vy, vz, sp[3])
                if (abs(self.vehicle_local_position.x - sp[0]) < 0.5 and
                        abs(self.vehicle_local_position.y - sp[1]) < 0.5 and
                        abs(self.vehicle_local_position.z - sp[2]) < 0.5):
                    self.get_logger().info(f'Moved to setpoint: {sp}')
                    # Se o setpoint atual for o segundo (hover), iniciar hover
                    if self.current_setpoint_index == 1:
                        if self.hover_start_time is None:
                            self.hover_start_time = time.time()
                        if time.time() - self.hover_start_time >= self.hover_duration:
                            self.current_setpoint_index += 1
                        else:
                            self.publish_position_setpoint(sp[0], sp[1], sp[2], sp[3])
                    else:
                        self.current_setpoint_index += 1
            else:
                self.land()
                self.disarm()
                rclpy.shutdown()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main(args=None) -> None:
    print('Waiting for key press (f) to start offboard control node...')
    # Configura o terminal para ler entrada sem bloqueio
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        while True:
            if is_data():
                c = sys.stdin.read(1)
                if c == 'f':
                    print('Key f pressed, starting offboard control node...')
                    rclpy.init(args=args)
                    offboard_control = OffboardControl()
                    rclpy.spin(offboard_control)
                    offboard_control.destroy_node()
                    rclpy.shutdown()
                    break
                elif c == '\x1b':  # Se 'Esc' for pressionado, encerra o loop
                    break
    finally:
        # Restaura as configurações do terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
