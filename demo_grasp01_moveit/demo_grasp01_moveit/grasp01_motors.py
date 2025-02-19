#!/usr/bin/env python3
import rclpy
from demo_grasp01_moveit.grasp01_planning_subscriber import GRASP01

def main(args=None):
    rclpy.init(args=args)
    node = GRASP01()
    
    try:
        # Spin the node indefinitely; the callback will print updates as they come in.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
