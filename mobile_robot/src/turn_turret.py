import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header, String


## TWO IMPORTANT CONSTANTS TO REMEMBER
IMAGE_WIDTH = 640    
CAMERA_FOV = 90


TARGET_X1 = 0
TARGET_X2 = 0


class TurretRotationCommand(Node):


    def __init__(self):

        super().__init__('turret_rotation_command_py')

        self.turret_pose_publisher = self.create_publisher(JointTrajectory, '/turret_controller/joint_trajectory', 1)
        self.target_coords_subscriber = self.create_subscription(String, '/target_coordinates', self.target_coords_callback, 10)
        self.check = 0

        self.frame_id = "base_link"

        self.timer_period = 5.0  
        self.duration_sec = 2 
        self.duration_nanosec = 0.5 * 1e9
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.turret_positions = []
        self.turret_positions.append([0.0]) 
        # Keep track of the current trajectory we are executing
        self.index = 0

     

    def calculate_turning_angle(self, x1, x2):

        target_center = (x1 + x2) // 2
        difference = ((IMAGE_WIDTH // 2 - target_center) * CAMERA_FOV // 2) / (IMAGE_WIDTH // 2)
        turning_angle = round(3.14 * difference / 180, 2)

        return turning_angle


    def target_coords_callback(self, coords):

        TARGET_X1 = int(coords.data.split(",")[0])
        TARGET_X2 = int(coords.data.split(",")[1])

        self.turning_angle = self.calculate_turning_angle(TARGET_X1, TARGET_X2)
        self.turret_positions.clear()
        self.turret_positions.append([self.turning_angle]) 


    def timer_callback(self):

        msg_turret = JointTrajectory()
        msg_turret.header = Header()  
        msg_turret.header.frame_id = self.frame_id  
        msg_turret.joint_names = ['turret_joint']

        point_turret = JointTrajectoryPoint()
        point_turret.positions = self.turret_positions[self.index]
        point_turret.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  

        msg_turret.points.append(point_turret)
        self.turret_pose_publisher.publish(msg_turret)

        # Reset the index
        if self.index == len(self.turret_positions) - 1:
            self.index = 0
        else:
            self.index = self.index + 1

    

# ros2 topic pub --once /turret_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['turret_joint'], points: [{positions: [0.1], velocities: [0.1], accelerations: [0.7], time_from_start: {sec: 1, nanosec: 0}}]}"


def main(args=None):

    rclpy.init(args=args)
    turret_rotation_command = TurretRotationCommand()
    rclpy.spin(turret_rotation_command)
    turret_rotation_command.destroy_node()
    rclpy.shutdown()


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
