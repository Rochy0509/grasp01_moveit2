import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf_transformations

class RobotPoseMover(Node):
    def __init__(self):
        super().__init__('robot_pose_mover')
        self.logger = get_logger("robot_pose_mover")
        
        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("grasp01_arm")
        
        # Initialize TF listener for getting robot poses
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait briefly to ensure MoveIt and TF are initialized
        time.sleep(1.0)

    def cleanup(self):
        """Clean up resources when the node is shutting down"""
        pass  # No camera resources to clean up in this version

    def get_robot_pose(self):
        """Get the current transform from base_link to ee_link"""
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'ee_link', rclpy.time.Time())
            
            # Convert to transformation matrix
            translation = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]
            
            rotation = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]
            
            # Create 4x4 transformation matrix
            matrix = tf_transformations.compose_matrix(
                translate=translation,
                angles=tf_transformations.euler_from_quaternion(rotation)
            )
            
            return matrix
        except Exception as e:
            self.logger.error(f"TF lookup failed: {e}")
            return None

    def plan_and_execute_pose(self, pose_goal):
        """Plan and move to a specific pose"""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="ee_link")
        
        self.logger.info("Planning trajectory")
        plan_result = self.arm.plan()
        
        if plan_result:
            self.logger.info("Executing plan")
            self.moveit.execute(plan_result.trajectory, controllers=[])
            return True
        else:
            self.logger.error("Planning failed")
            return False

    def create_test_poses(self):
        """Create a list of diverse test poses"""
        poses = []
        
        # Pose 1: Starting pose
        pose1 = PoseStamped()
        pose1.header.frame_id = "base_link"
        pose1.pose.position.x = -0.437
        pose1.pose.position.y = -0.269
        pose1.pose.position.z = -0.080
        pose1.pose.orientation.x = 0.192
        pose1.pose.orientation.y = 0.257
        pose1.pose.orientation.z = 0.923
        pose1.pose.orientation.w = -0.215
        poses.append(pose1)
        
        # Pose 2: Move in X direction
        pose2 = PoseStamped()
        pose2.header.frame_id = "base_link"
        pose2.pose.position.x = -0.471  # +10cm in X
        pose2.pose.position.y = -0.024
        pose2.pose.position.z = -0.123
        pose2.pose.orientation.x = 0.231
        pose2.pose.orientation.y = 0.397
        pose2.pose.orientation.z = 0.879
        pose2.pose.orientation.w = 0.124
        poses.append(pose2)
        
        # Pose 3: Move in Y direction
        pose3 = PoseStamped()
        pose3.header.frame_id = "base_link"
        pose3.pose.position.x = -0.536
        pose3.pose.position.y = -0.086  # +10cm in Y
        pose3.pose.position.z = -0.272
        pose3.pose.orientation.x = -0.174
        pose3.pose.orientation.y = 0.179
        pose3.pose.orientation.z = 0.964
        pose3.pose.orientation.w = 0.089
        poses.append(pose3)
        
        # Pose 4: Move in Z direction
        pose4 = PoseStamped()
        pose4.header.frame_id = "base_link"
        pose4.pose.position.x = -0.551
        pose4.pose.position.y = -0.231
        pose4.pose.position.z = -0.230  # +10cm in Z
        pose4.pose.orientation.x = -0.342
        pose4.pose.orientation.y = 0.326
        pose4.pose.orientation.z = 0.877
        pose4.pose.orientation.w = -0.085
        poses.append(pose4)
        
        # Pose 5: Change orientation
        pose5 = PoseStamped()
        pose5.header.frame_id = "base_link"
        pose5.pose.position.x = 0.055
        pose5.pose.position.y = -0.096
        pose5.pose.position.z = -0.663
        pose5.pose.orientation.x = 0.727
        pose5.pose.orientation.y = -0.232
        pose5.pose.orientation.z = -0.625
        pose5.pose.orientation.w = -0.165
        poses.append(pose5)
        
        return poses

    def run(self):
        """Execute the sequence of test poses"""
        self.logger.info("Starting robot pose movement sequence")
        
        # Get and log initial pose
        initial_pose = self.get_robot_pose()
        if initial_pose is not None:
            self.logger.info(f"Initial robot pose:\n{initial_pose}")
        
        # Create test poses
        test_poses = self.create_test_poses()
        
        # Move to each pose
        for i, pose in enumerate(test_poses):
            self.logger.info(f"Moving to pose {i+1}/{len(test_poses)}")
            pose.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
            
            # Log target pose details
            self.logger.info(f"Target pose {i+1}:")
            self.logger.info(f"Position: x={pose.pose.position.x:.3f}, "
                           f"y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")
            self.logger.info(f"Orientation: x={pose.pose.orientation.x:.3f}, "
                           f"y={pose.pose.orientation.y:.3f}, z={pose.pose.orientation.z:.3f}, "
                           f"w={pose.pose.orientation.w:.3f}")
            
            # Execute movement
            success = self.plan_and_execute_pose(pose)
            
            if success:
                # Wait for robot to settle
                time.sleep(2.0)
                
                # Verify current pose
                current_pose = self.get_robot_pose()
                if current_pose is not None:
                    self.logger.info(f"Reached pose {i+1}:\n{current_pose}")
                else:
                    self.logger.warn(f"Could not verify pose {i+1} position")
            else:
                self.logger.error(f"Failed to move to pose {i+1}")
                continue  # Continue to next pose despite failure
            
            # Small delay between poses
            time.sleep(1.0)
        
        self.logger.info("Pose movement sequence completed")

def main():
    rclpy.init()
    
    try:
        mover = RobotPoseMover()
        mover.run()
    except Exception as e:
        print(f"Error in pose movement: {e}")
    finally:
        # Cleanup
        if 'mover' in locals():
            mover.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main()