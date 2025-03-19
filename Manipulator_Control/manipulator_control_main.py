from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control_node')

        # Initialize the robotic arm
        self.bot = InterbotixManipulatorXS("px150", "arm", "gripper")
        self.bot.arm.go_to_sleep_pose()

        # Subscriber to the depth sensor topic
        self.subscription = self.create_subscription(
            Point, 
            '/depth_sensor/coordinates',  # ROS2 topic where depth sensor publishes coordinates
            self.listener_callback, 
            10)

        self.target_coordinates = None

    def listener_callback(self, msg):
        # Receive coordinates from the depth sensor
        self.target_coordinates = msg
        self.get_logger().info(f'Received coordinates: {self.target_coordinates}')

        # Move the arm to the target position
        self.move_arm_to_position(self.target_coordinates)

    def move_arm_to_position(self, coordinates):
        # Move the arm to the target position based on coordinates
        self.bot.arm.set_ee_pose_components(x=coordinates.x, y=coordinates.y, z=coordinates.z, roll=0.0, pitch=0.0, yaw=0.0)

        time.sleep(2)  # Allow some time for the arm to reach the target position

        # Grasp the object
        self.bot.gripper.grasp()
        self.get_logger().info("Grasping the object...")
        time.sleep(2)  # Simulate time for grasping

        # Move to another predefined position (destination)
        self.move_arm_to_drop_position()

    def move_arm_to_drop_position(self):
        # Drop the block by moving to another position (example coordinates for drop)
        drop_position = [0.5, 0.2, 0.15]  # Example coordinates for drop position
        self.bot.arm.set_ee_pose_components(x=drop_position[0], y=drop_position[1], z=drop_position[2], roll=0.0, pitch=0.0, yaw=0.0)

        time.sleep(2)  # Simulate time to move to the drop position

        # Release the object
        self.bot.gripper.release()
        self.get_logger().info("Object released.")

        # Return to default position (Home)
        self.bot.arm.go_to_sleep_pose()
        self.get_logger().info("Arm returned to default position.")

def main(args=None):
    rclpy.init(args=args)
    arm_control_node = ArmControl()

    try:
        rclpy.spin(arm_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        arm_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
