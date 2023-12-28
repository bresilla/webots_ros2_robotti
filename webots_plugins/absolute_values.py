from rosgraph_msgs.msg import Clock
import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    from handy_msgs.srv import Spawn
    handy_msgs_found = True
except ImportError:
    handy_msgs_found = False

class Node:
    def init(self, webots_node, properties):
        global handy_msgs_found
        rclpy.init(args=None)
        self.__clock = Clock()

        self.node = rclpy.create_node('plugin')
        self.node.get_logger().info('  - properties: ' + str(properties))
        self.robot = webots_node.robot
        self.super = webots_node.robot.getSelf()
        self.root = webots_node.robot.getRoot()
        self.node.get_logger().info('  - robot: ' + str(self.robot.getName()))
        self.node.get_logger().info('  - timestep: ' + str(int(self.robot.getBasicTimeStep())))
        self.node.get_logger().info('  - supervisor? ' + str(self.robot.getSupervisor()))

        self.node.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        self.clock_pub = self.node.create_publisher(Clock, 'custom_clock', 1)
        self.state_pub = self.node.create_publisher(Odometry, '/sim/pose', 1)

        self.node.create_timer(1.0, self.__timer_callback)
        self.string_pub = self.node.create_publisher(String, '/sim/string', 10)
        if handy_msgs_found:
            self.node.create_service(Spawn, '/sim/spawn_object', self.__spoawn_object_callback)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.publish()

    def __timer_callback(self):
        self.string_pub.publish(String(data='Hello World!'))

    def __clock_callback(self, msg):
        self.__clock = msg

    def publish(self):
        msg = Odometry()
        msg.header.stamp = self.__clock.clock
        msg.header.frame_id = "abs"
        msg.pose.pose.position.x = self.super.getPosition()[0]
        msg.pose.pose.position.y = self.super.getPosition()[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        theta = np.arctan2(self.super.getOrientation()[3], self.super.getOrientation()[0])
        msg.pose.pose.orientation.z = np.sin(theta / 2)
        msg.pose.pose.orientation.w = np.cos(theta / 2)
        self.state_pub.publish(msg)
        self.clock_pub.publish(self.__clock)

    def __spoawn_object_callback(self, request, response):
        pose = request.translation.data
        while len(pose) < 3:
            pose.append(0.1)
        rota = request.rotation.data
        while len(rota) < 4:
            rota.append(1)
        size = request.size.data
        while len(size) < 3:
            size.append(0.1)
        color = request.color.data
        while len(color) < 3:
            color.append(1)
        name = request.name.data if request.name.data else "object"
        # self.node.get_logger().info(f" ------> spawn object at {pose} with size {size}")
        def_name = f"DEF {name} Solid {{ \
                        translation {pose[0]} {pose[1]} {pose[2]+size[2]/2} \
                        rotation {rota[0]} {rota[1]} {rota[2]} {rota[3]} \
                        children [ \
                            Shape {{ \
                                appearance Appearance {{ \
                                    material Material {{ \
                                        diffuseColor {color[0]} {color[1]} {color[2]} \
                                    }} \
                                }} \
                                geometry Box {{ \
                                    size {size[0]} {size[1]} {size[2]} \
                                }} \
                            }} \
                        ] \
                        }}"
        childrem_field = self.root.getField('children')
        childrem_field.importMFNodeFromString(-1, def_name)
        response.message = f"Object spawned at {pose} with size {size}"
        return response