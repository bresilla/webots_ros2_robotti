import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist

class RobottiDriver:
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.FRONT_WHEEL_RADIUS = 0.38

        self.left_front_wheel = self.__robot.getDevice("left_front_wheel_joint_motor")
        self.left_front_wheel.setPosition(float('inf'))
        self.left_front_wheel.setVelocity(0.0)
        self.right_front_wheel = self.__robot.getDevice("right_front_wheel_joint_motor")
        self.right_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setVelocity(0.0)
        self.left_rear_wheel = self.__robot.getDevice("left_rear_wheel_joint_motor")
        self.left_rear_wheel.setPosition(float('inf'))
        self.left_rear_wheel.setVelocity(0.0)
        self.right_rear_wheel = self.__robot.getDevice("right_rear_wheel_joint_motor")
        self.right_rear_wheel.setPosition(float('inf'))
        self.right_rear_wheel.setVelocity(0.0)


        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('robotti_node')
        self.__node.create_subscription(AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, 1)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_ackermann_callback(self, message):
        self.set_speed(message.speed)
        self.set_steering_angle(message.angular.z)

    def __cmd_vel_callback(self, message):
        self.set_speed(message.linear.x)
        self.set_steering_angle(message.angular.z)


    def set_speed(self, kmh):
        if kmh > 30.0:
            kmh = 30.0
        self.speed = kmh
        # print(f"setting speed to {kmh} km/h")
        self.__node.get_logger().info(f'setting speed to {kmh} km/h')
        front_ang_vel = kmh * 1000.0 / 3600.0 / self.FRONT_WHEEL_RADIUS
        rear_ang_vel = kmh * 1000.0 / 3600.0 / self.FRONT_WHEEL_RADIUS
        # set motor rotation speed
        self.left_front_wheel.setVelocity(front_ang_vel)
        self.right_front_wheel.setVelocity(front_ang_vel)
        self.left_rear_wheel.setVelocity(rear_ang_vel)
        self.right_rear_wheel.setVelocity(rear_ang_vel)


    def set_steering_angle(self, wheel_angle):
        pass
        # self.steering_angle = min(0.94, max(-0.94, wheel_angle))
        # self.left_steer.setPosition(self.steering_angle)
        # self.right_steer.setPosition(self.steering_angle)