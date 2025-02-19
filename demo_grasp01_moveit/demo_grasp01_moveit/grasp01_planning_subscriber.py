#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest
from rclpy.qos import QoSProfile
import myactuator_rmd_py as rmd
import time
import math
import threading

class GRASP01(Node):
    def __init__(self):
        super().__init__('grasp01_subscriber')
        self.joint_positions = []  # This will store the joint positions list
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            MotionPlanRequest,
            '/motion_plan_request',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to /motion_plan_request.")

    def listener_callback(self, msg):
        # Extract joint positions from the first goal constraint
        positions = []
        if msg.goal_constraints:
            constraint = msg.goal_constraints[0]
            for jc in constraint.joint_constraints:
                positions.append(jc.position)
        self.joint_positions = positions
        self.get_logger().info("Updated joint positions: " + str(positions))
    
    def get_joint_positions(self):
        # Returns the stored joint positions
        return self.joint_positions

class RMD_Motor_Control:
    """Class with the control logic for RMD motors.
       This class initializes six motors and sends commands to them using received joint positions.
    """
    def __init__(self):
        self.logger = rclpy.logging.get_logger("MotorController")
        self.driver = rmd.CanDriver("can0")
        # Store expected IDs along with the motor object.
        self.motors = [
            (1, rmd.ActuatorInterface(self.driver, 1)),
            (2, rmd.ActuatorInterface(self.driver, 2)),
            (3, rmd.ActuatorInterface(self.driver, 3)),
            (4, rmd.ActuatorInterface(self.driver, 4)),
            (5, rmd.ActuatorInterface(self.driver, 5)),
            (6, rmd.ActuatorInterface(self.driver, 6))
        ]
        self.accleration = rmd.actuator_state.AccelerationType.POSITION_PLANNING_ACCELERATION
        self.deceleration = rmd.actuator_state.AccelerationType.POSITION_PLANNING_DECELERATION
        self.initialize_motors()

    def initialize_motors(self):
        for expected_id, motor in self.motors:
            try:
                initial_pos = motor.getMultiTurnAngle()
                self.logger.info(f"Motor {expected_id} Initial Position: {initial_pos}")
                time.sleep(0.1)  # Increased delay between commands
                
                motor.setEncoderZero(int(initial_pos))
                self.logger.info(f"Motor {expected_id} encoder zero explicitly set to {initial_pos}")
                
                motor.setCurrentPositionAsEncoderZero()
                self.logger.info(f"Motor {expected_id} current position set as zero.")
                
                zero_offset = motor.getMultiTurnEncoderZeroOffset()
                self.logger.info(f"Motor {expected_id} Zero Offset (Post Set): {zero_offset}")
                
                self.logger.info("Automatically confirming encoder zero values. Proceeding with motor reset.")
                motor.reset()
                self.logger.info(f"Motor {expected_id} has been reset after encoder zero setup.")
                time.sleep(0.1)
            except rmd.ProtocolException as e:
                self.logger.error(f"Error setting zero for Motor {expected_id}: {e}")
            except Exception as e:
                self.logger.error(f"Unexpected error with Motor {expected_id}: {e}")

    def rad_to_deg(self, positions):
        """Convert a list of positions from radians to degrees."""
        return [math.degrees(position) for position in positions]

    def send_pos_motors(self, positions):
        """
        Send motor commands to all motors.
        'positions' is a list of positions (in degrees) corresponding to each motor.
        """
        # Set acceleration parameters for all motors
        for expected_id, motor in self.motors:
            motor.setAcceleration(1500, self.accleration)
            motor.setAcceleration(1450, self.deceleration)
        
        # Send position command to each motor
        for idx, (expected_id, motor) in enumerate(self.motors):
            try:
                target = positions[idx]
                motor.sendPositionAbsoluteSetpoint(target, 110)
                self.logger.info(f"Motor {expected_id} command sent with position: {target}")
            except IndexError:
                self.logger.warn(f"No position provided for Motor {expected_id}.")
        time.sleep(1)

    def shutdown_motors(self):
            """Shut down all motors by sending a shutdown command."""
            for expected_id, motor in self.motors:
                try:
                    motor.shutdownMotor()
                    self.logger.info(f"Motor {expected_id} powered down.")
                except Exception as e:
                    self.logger.error(f"Error powering down Motor {expected_id}: {e}")

def positions_changed(prev, current, tol=1e-3):
    """Return True if any joint position differs by more than tol."""
    if prev is None:
        return True
    if len(prev) != len(current):
        return True
    for a, b in zip(prev, current):
        if abs(a - b) > tol:
            return True
    return False

def main(args=None):
    rclpy.init(args=args)
    
    # Instantiate the subscriber node and motor controller.
    plan_node = GRASP01()
    motor_ctrl = RMD_Motor_Control()

    # Run the ROS2 subscriber in a separate thread.
    spinner_thread = threading.Thread(target=rclpy.spin, args=(plan_node,))
    spinner_thread.start()

    prev_positions = None
    try:
        # Main loop: check for new joint positions and send commands only when they change.
        while rclpy.ok():
            positions = plan_node.get_joint_positions()
            if positions:
                positions_deg = motor_ctrl.rad_to_deg(positions)
                if positions_changed(prev_positions, positions_deg):
                    motor_ctrl.send_pos_motors(positions_deg)
                    prev_positions = positions_deg.copy()
                else:
                    plan_node.get_logger().info("No significant change in joint positions. Command not sent.")
            time.sleep(0.1)
    except KeyboardInterrupt:
        plan_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        motor_ctrl.shutdown_motors()
        plan_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            plan_node.get_logger().warn("rclpy.shutdown() raised an exception: " + str(e))
        spinner_thread.join()
if __name__ == '__main__':
    main()
