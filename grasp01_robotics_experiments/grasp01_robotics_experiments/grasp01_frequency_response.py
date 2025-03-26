#!/usr/bin/env python3
"""
A script to test the frequency bandwidth of a robotic arm's response using MoveItPy.
"""
import os
import time
import numpy as np
import rclpy
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
import csv
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer

class BandwidthTest(Node):
    def __init__(self):
        super().__init__('bandwidth_test')
        self.logger = get_logger("bandwidth_test")
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("grasp01_arm")
        
        # For actual position feedback
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.actual_positions = []
        self.commanded_positions = []

    def joint_callback(self, msg):
        # Placeholder to store actual joint positions if needed
        pass

    def get_actual_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'ee_link', rclpy.time.Time())
            return trans.transform.translation.z
        except Exception as e:
            self.logger.error(f"TF lookup failed: {e}")
            return None

    def plan_and_execute_pose(self, pose_goal):
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ee_link")
        plan_result = self.arm.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])
            return True
        return False

    def run_frequency_test(self, frequencies, amplitude=0.05, duration_per_freq=5.0):
        nominal_pose = PoseStamped()
        nominal_pose.header.frame_id = "base_link"
        nominal_pose.pose.position.x = 0.391
        nominal_pose.pose.position.y = -0.107
        nominal_pose.pose.position.z = -0.301
        nominal_pose.pose.orientation.x = 0.223
        nominal_pose.pose.orientation.y = 0.029
        nominal_pose.pose.orientation.z = 0.037
        nominal_pose.pose.orientation.w = 0.974

        # Move to nominal pose first
        self.plan_and_execute_pose(nominal_pose)
        time.sleep(5.0)  # Wait to settle

        for freq in frequencies:
            self.logger.info(f"Testing frequency: {freq} Hz")
            start_time = time.time()
            self.commanded_positions = []
            self.actual_positions = []

            while time.time() - start_time < duration_per_freq:
                t = time.time() - start_time
                z_command = -0.301 + amplitude * np.sin(2 * np.pi * freq * t)
                
                # Update pose
                pose_goal = PoseStamped()
                pose_goal.header.frame_id = "base_link"
                pose_goal.pose.position.x = 0.391
                pose_goal.pose.position.y = -0.107
                pose_goal.pose.position.z = z_command
                pose_goal.pose.orientation = nominal_pose.pose.orientation

                # Send command
                self.plan_and_execute_pose(pose_goal)

                # Log data
                actual_z = self.get_actual_pose()
                if actual_z is not None:
                    self.commanded_positions.append((t, z_command))
                    self.actual_positions.append((t, actual_z))

                # Control update rate (e.g., 50 Hz)
                time.sleep(0.02)

            # Save data for this frequency
            self.save_data(freq)

    def save_data(self, freq):
    # Construct the absolute path
        directory = os.path.expanduser("~/grasp/Documents")
        filename = f'freq_response_{freq}Hz.csv'
        filepath = os.path.join(directory, filename)
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'commanded_z', 'actual_z'])
            for (t_cmd, z_cmd), (t_act, z_act) in zip(self.commanded_positions, self.actual_positions):
                writer.writerow([t_cmd, z_cmd, z_act])

def main():
    rclpy.init()
    node = BandwidthTest()
    frequencies = [0.1]
    node.run_frequency_test(frequencies)
    rclpy.shutdown()

if __name__ == "__main__":
    main()