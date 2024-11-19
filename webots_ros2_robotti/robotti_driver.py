import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from webots_ros2_msgs.msg import FloatStamped
from std_msgs.msg import Float32

class RobottiDriver:
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.FRONT_WHEEL_RADIUS = 0.7
        self.WHEEL_SEPARATION = 3.0
        self.name = self.__robot.getName()

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


        self.gps_main_msg = NavSatFix()
        self.gps_aux_msg = NavSatFix()
        self.heading = Float32()

        self.__node.create_subscription(NavSatFix, self.name + '/gps', self.main_callback, 10)
        self.gps_main = self.__node.create_publisher(NavSatFix, self.name + "/gnss/fix", 10)
        self.__node.create_subscription(NavSatFix, self.name + '/gps_aux', self.aux_callback, 10)
        self.gps_aux = self.__node.create_publisher(NavSatFix, self.name + "/gnss/fix_aux", 10)
        self.__node.create_subscription(FloatStamped, self.name + '/compass/bearing', self.compass_callback, 10)
        self.compass = self.__node.create_publisher(Float32, self.name + "/gnss/heading", 10)
        self.__node.create_subscription(AckermannDrive, self.name + '/cmd_ackermann', self.__cmd_ackermann_callback, 1)
        self.__node.create_subscription(Twist, self.name + '/cmd_vel', self.__cmd_vel_callback, 1)

        #timer
        self.__node.create_timer(0.001, self.timer_callback)

    def main_callback(self, msg):
        msg.header.frame_id = "gps_main"
        self.gps_main_msg = msg
        # self.gps_main.publish(self.gps_main_msg)

    def aux_callback(self, msg):
        msg.header.frame_id = "gps_aux"
        self.gps_aux_msg = msg
        # self.gps_aux.publish(self.gps_aux_msg)

    def compass_callback(self, msg):
        self.heading.data = msg.data
        # self.compass.publish(self.heading)

    def timer_callback(self):
        self.gps_aux.publish(self.gps_aux_msg)
        self.gps_main.publish(self.gps_main_msg)
        # reverse heading to match ROS convention
        self.heading.data = -self.heading.data
        self.compass.publish(self.heading)


    def __cmd_ackermann_callback(self, message):
        left, right = self.calculate_wheel_velocities(message.speed, message.steering_angle)
        self.left_front_wheel.setVelocity(left)
        self.right_front_wheel.setVelocity(right)

    def __cmd_vel_callback(self, message):
        left, right = self.calculate_wheel_velocities(message.linear.x, message.angular.z)
        self.left_front_wheel.setVelocity(left)
        self.right_front_wheel.setVelocity(right)


    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        left_wheel_velocity = (linear_velocity - (self.WHEEL_SEPARATION * angular_velocity) / 2) / self.FRONT_WHEEL_RADIUS
        right_wheel_velocity = (linear_velocity + (self.WHEEL_SEPARATION * angular_velocity) / 2) / self.FRONT_WHEEL_RADIUS
        return left_wheel_velocity, right_wheel_velocity
