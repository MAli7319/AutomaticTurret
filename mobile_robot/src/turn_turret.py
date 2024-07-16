import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64


class CommandToJointState(Node):


    def __init__(self):

        super().__init__('command_to_joint_state')

        self.joint_name = self.declare_parameter('joint_name', 'turret_joint').value
        self.target_pos = self.declare_parameter('target_position', [1.8]).value
        self.seconds = self.declare_parameter('seconds', 0).value
        self.nanoseconds = self.declare_parameter('nanoseconds', 0).value
        self.points = {positions:}

        self.joint_trajectory = JointTrajectory()
        self.joint_trajectory.joint_names.append(self.joint_name)
        self.joint_trajectory.points.append()
        self.joint_state.velocity.append(0.0)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        print(1)

    

# ros2 topic pub --once /turret_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['turret_joint'], points: [{positions: [0.1], velocities: [0.1], accelerations: [0.7], time_from_start: {sec: 1, nanosec: 0}}]}"


def main(args=None):
    rclpy.init(args=args)
    command_to_joint_state = CommandToJointState()
    rclpy.spin(command_to_joint_state)
    


if __name__ == '__main__':
    main()

"""

import threading

import rclpy
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('position_velocity_publisher')
pub = node.create_publisher(JointState, 'joint_command', 10)

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

joint_state_position = JointState()
joint_state_velocity = JointState()

joint_state_position.name = ["joint1"]
joint_state_velocity.name = ["turret_joint"]
joint_state_position.position = [0.2,0.2,0.2]
joint_state_velocity.velocity = [1.0]

rate = node.create_rate(10)
try:
    while rclpy.ok():
        pub.publish(joint_state_position)
        pub.publish(joint_state_velocity)
        rate.sleep()
except KeyboardInterrupt:
    pass
rclpy.shutdown()
thread.join()

"""
