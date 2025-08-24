#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class SimpleJointStatePublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Create timer to publish joint states at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Define the joint names from the URDF
        self.joint_names = [
            'husky_robot_model__front_left_wheel_joint',
            'husky_robot_model__front_right_wheel_joint',
            'husky_robot_model__rear_left_wheel_joint',
            'husky_robot_model__rear_right_wheel_joint',
            'husky_robot_model__pan_gimbal_joint',
            'camera_box_joint'
        ]
        
        self.get_logger().info('Simple Joint State Publisher started')

    def publish_joint_states(self):
        msg = JointState()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Set joint names and positions (all zeros for default position)
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    joint_state_publisher = SimpleJointStatePublisher()
    
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
