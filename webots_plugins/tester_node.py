import rclpy
from std_msgs.msg import String

class Node:
    # The `init` method is called only once the driver is initialized. You will always get two arguments in the `init` method.
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node. However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('test_node')

        # This will print the parameter from the URDF file. `{ 'parameterExample': 'someValue' }`
        self.properties = properties

        # The robot property allows you to access the Standard Webots API.
        # See: https://cyberbotics.com/doc/reference/robot
        self.robot = webots_node.robot
        self.__node.get_logger().info('  - robot: ' + str(self.robot.getName()))
        self.__node.get_logger().info('  - timestep: ' + str(int(self.robot.getBasicTimeStep())))

        # The robot property allows you to access the Supervisor Webots API if the robot is a Supervisor.
        # See: https://cyberbotics.com/doc/reference/supervisor
        self.super = webots_node.robot.getSelf()
        self.__node.get_logger().info('  - supervisor? ' + str(self.robot.getSupervisor()))
        

        self.__node.create_timer(1.0, self.timer_callback)
        self.string_publisher = self.__node.create_publisher(String, '/sim/string', 10)

    def timer_callback(self):
        # self.node.get_logger().info('INFO')
        self.string_publisher.publish(String(data=f' --- {str(self.properties)} --- '))

    def step(self):
        # print('step')
        rclpy.spin_once(self.__node, timeout_sec=0)



import sys
sys.dont_write_bytecode = True
import numpy as np
import sys
import cv2
import logging
import math
import matplotlib.pyplot as plt
from controller import Supervisor, Display, Keyboard


logging.basicConfig(level=logging.INFO, format="%(levelname)s :: %(message)s")

class Machine:
    def __init__(self) -> None:
        self.__s = {"IDLE": self.__idle}
        self.curr_state = "IDLE"

    def __idle(self):
        print(sys._getframe().f_code.co_name)

    def __call__(self, states):
        self.__s = states
        self.curr_state = list(self.__s.keys())[0]

    @property
    def state(self):
        return self.curr_state

    @state.setter
    def state(self, state):
        if state not in self.__s:
            raise Exception("Value not in STATES")
        print(f"Entering Machine State: {state}")
        self.curr_state = state

    def run(self):
        self.__s[self.curr_state]()


characteristics = {
    "e-puck": {
        "Wheel Radius": 20.0,  # mm
        "Axle Length": 57.0,  # mm
        "Max Rotation": 6.28,
        "Max Velocity": 6.28,
        "Precision": 0.001,
        "Controller": "UNCYCLE",
    },
    "pioneer": {
        "Wheel Radius": 97.5,  # mm
        "Axle Length": 340.0,  # mm
        "Max Rotation": 12.3,
        "Max Velocity": 512,
        "Precision": 0.01,
        "Controller": "UNCYCLE",
    },
}


class MyBot(Supervisor, Machine):
    def __init__(self, robot, pose=[0, 0, 0], reset=True):
        super().__init__()
        self.log = logging.getLogger()
        self.timestep = int(self.getBasicTimeStep())
        self.counter = 0
        # self.timestep = 16
        self.super = self.getSelf()
        self.charecteristics = characteristics[robot]
        self.rotation = self.charecteristics["Max Rotation"]
        self.velocity = self.charecteristics["Max Velocity"]
        self.precision = self.charecteristics["Precision"]
        # reset physics
        if reset:
            self.reset()
        # variabes
        self.pose = pose
        self.goal = [0, 0]
        self.controls = [0, 0]
        self.dest = None
        self.path = None
        self.objects = {}
        self.halt = False
        # private
        self.__teleop_motion = [0, 0]
        self.__prev_enc = [0.000001, 0.000001]

        self.machine = Machine()
        self.color_map = {
            'blue': [0,0,1],
            'green': [0,1,0],
            'red': [1,0,0],
            'cyan': [0,1,1],
            'magenta': [1,0,1],
            'yellow': [1,1,0],
            'white': [1,1,1],
            'black': [0,0,0],
        }
        self.color_hsv = {
            'blue': ([120, 100, 100], [150, 255, 255]),
            'green': ([35, 120, 50], [85, 255, 255]),
            'red': ([0, 120, 50], [10, 255, 255]),
            'yellow': ([25, 120, 50], [35, 255, 255]),
            'magenta': ([130, 120, 50], [150, 255, 255]),
            'cyan': ([75, 120, 50], [95, 255, 255]),
            'white': ([0, 0, 150], [180, 30, 255]),
            'black': ([0, 0, 0], [180, 255, 30]),
        }

    # call funciton (very pythony)
    def __call__(self, camera=True, lidar=True):
        print("MYBOT")

    def reset(self):
        self.simulationResetPhysics()
        self.simulationReset()
        self.step(self.timestep)
        self.enableMotors()
        self.enableEncoders()
        self.step(self.timestep)

    def pause(self):
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)

    def play(self):
        self.simulationSetMode(self.SIMULATION_MODE_REAL_TIME)

    # initialize robot devices
    def initializeDevices(
        self,
        motors=True,
        keyboard=False,
        display=False,
    ):
        self.step(self.timestep)
        if motors:
            self.enableMotors()
        if keyboard:
            self.enableKeyboard()
        if display:
            self.enableDisplay()

    # initialize robot sensors
    def initializeSensors(
        self,
        position=True,
        encoders=True,
        camera=False,
        lidar=False,
        range=False,
        gps=False,
        imu=False,
        gyro=False,
        accel=False,
        compass=False,
        proxi=False,
        light=False,
        ground=False,
    ):
        # internal start
        self.step(self.timestep)
        # initialize precise location
        if position:
            self.pose = [
                self.super.getPosition()[0],
                self.super.getPosition()[1],
                np.arctan2(
                    self.super.getOrientation()[3], self.super.getOrientation()[0]
                ),
            ]
            self.hist = np.array([[self.pose[0], self.pose[1]]])
        if encoders:
            self.enableEncoders()
        if camera:
            self.enableCamera()
        if lidar:
            self.enableLidar()
        if range:
            self.enableRange()
        if gps:
            self.enableGPS()
        if imu:
            self.enableIMU()
        if gyro:
            self.enableGyro()
        if accel:
            self.enableAccel()
        if compass:
            self.enableCompass()
        if proxi:
            self.enableProximitySensor()
        if light:
            self.enableLightSensor()
        if ground:
            self.enableGroundSensor()

    # initialize motors
    def enableMotors(self):
        self.step(self.timestep)
        self.leftMotor = self.getDevice("left wheel")
        self.rightMotor = self.getDevice("right wheel")
        self.leftMotor.setPosition(float("+inf"))
        self.rightMotor.setPosition(float("+inf"))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

    # enable encoders
    # offset issue -> https://stackoverflow.com/questions/61150174/reset-webots-position-sensor
    def enableEncoders(self, names=["left wheel sensor"]):
        self.step(self.timestep)
        self.leftEncoder = self.getDevice("left wheel sensor")
        self.leftEncoder.enable(self.timestep)
        self.leftEncoder_offset = self.leftEncoder.getValue()
        self.rightEncoder = self.getDevice("right wheel sensor")
        self.rightEncoder.enable(self.timestep)
        self.rightEncoder_offset = self.rightEncoder.getValue()

    # enable lidar
    def enableLidar(self, name="lidar"):
        self.lidar = self.getDevice(name)
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

    # enable GPS
    def enableGPS(self, name="gps"):
        self.gps = self.getDevice(name)
        self.gps.enable(self.timestep)

    # enable compass
    def enableCompass(self, name="compass"):
        self.compass = self.getDevice(name)
        self.compass.enable(self.timestep)

    # enable IMU
    def enableIMU(self, name="inertial unit"):
        self.imu = self.getDevice(name)
        self.imu.enable(self.timestep)

    # enable accelerometer
    def enableAccel(self, name="accelerometer"):
        self.accel = self.getDevice(name)
        self.accel.enable(self.timestep)

    # enable gyro
    def enableGyro(self, name="gyro"):
        self.gyro = self.getDevice(name)
        self.gyro.enable(self.timestep)

    # enable camera
    def enableCamera(self, name="camera1"):
        self.camera = self.getDevice(name)
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)

    # enable range
    def enableRange(self, name="range-finder"):
        self.range = self.getDevice(name)
        self.range.enable(self.timestep)

    # enable ground sensor
    def enableGroundSensor(self, name="gs"):
        self.ground = []
        for i in range(3):
            self.ground.append(self.getDevice(name + str(i)))
            self.ground[i].enable(self.timestep)

    # enable proximity sensors
    def enableProximitySensor(self, name="ps"):
        self.ps = []
        for i in range(8):
            self.ps.append(self.getDevice(name + str(i)))
            self.ps[i].enable(self.timestep)

    # enable light sensors
    def enableLightSensor(self, name="ls"):
        self.ls = []
        for i in range(8):
            self.ls.append(self.getDevice(name + str(i)))
            self.ls[i].enable(self.timestep)

    # enable display
    def enableDisplay(self, name="map"):
        self.display = self.getDevice(name)

    # shome image to display
    def showDisplay(self, image_path):
        maper = cv2.rotate(
            cv2.flip(cv2.imread(image_path), 1), cv2.ROTATE_90_COUNTERCLOCKWISE
        )
        imageRef = self.display.imageNew(
            width=self.display.getWidth(),
            height=self.display.getHeight(),
            data=maper.tolist(),
            format=Display.RGB,
        )
        self.display.imagePaste(imageRef, 0, 0)

    # enable keyboard
    def enableKeyboard(self):
        self.keyboard = Keyboard()
        self.keyboard.enable(self.timestep)

    # teleop
    def teleop(
        self, update_kinematics=False, reset_rotation=True, reset_velocity=False
    ):
        key = self.keyboard.getKey()
        if key != -1:
            if key == 32:
                self.__teleop_motion = [0, 0]
            elif key == Keyboard.RIGHT:
                self.__teleop_motion[1] = self.__teleop_motion[1] - 1
            elif key == Keyboard.LEFT:
                self.__teleop_motion[1] = self.__teleop_motion[1] + 1
            elif key == Keyboard.DOWN:
                self.__teleop_motion[0] = self.__teleop_motion[0] - 5
            elif key == Keyboard.UP:
                self.__teleop_motion[0] = self.__teleop_motion[0] + 5
        vr, vl = self.inverseKinematics(
            self.__teleop_motion[0], self.__teleop_motion[1]
        )
        self.setMotorSpeed(vr, vl)
        self.__teleop_motion[0] = 0 if reset_velocity else self.__teleop_motion[0]
        self.__teleop_motion[1] = 0 if reset_rotation else self.__teleop_motion[1]
        if update_kinematics:
            self.forwardKinematics_0()

    # wait for keypress
    def wait_keyboard(self):
        while self.keyboard.getKey() != ord("Y"):
            super().step(self.__timestep)

    # set motor speed
    def setMotorSpeed(self, right_speed, left_speed):
        self.leftMotor.setVelocity(left_speed)
        self.rightMotor.setVelocity(right_speed)

    # spawn an object/box and add to object list
    def spawnObject(self, pose=[1, 1], color=[1, 0, 0], name='ball', size=[0.05, 0.05, 0.05], recognition=True):
        root_node = self.getRoot()
        rec_col = f"recognitionColors [ {color[0]} {color[1]} {color[2]} ]" if recognition else ""
        children_field = root_node.getField('children')
        def_name = f"DEF {name} Solid {{ \
                        translation {pose[0]} {pose[1]} {size[2]/2} \
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
                        {rec_col} \
                        }}"
        children_field.importMFNodeFromString(-1, def_name)
        obj_node = self.getFromDef(name)
        self.objects[name] = obj_node
        return obj_node

    def spawnFloorplan(self, floorplan, size_arena):
        # Get the shape of the floorplan
        rows, cols = floorplan.shape
        # Calculate the size of cells in the floorplan in world coordinates
        cell_h = size_arena[1] / rows
        cell_w = size_arena[0] / cols
        self.cell_size = np.array([cell_w, cell_h])
        for i in range(rows):
            for j in range(cols):
                if floorplan[i,j]>0:
                    self.spawnObject(name="obj", pose=((j+.5)*cell_w,(i+.5)*cell_h),size=[cell_w,cell_h,0.1], color=[0,0,1])

    def spawnLandmarks(self, landmarks, size=[0.01, 0.01, 0.2]):
        # Spawn landmarks in Webots
        for i in range(landmarks.shape[1]):
            self.spawnObject(
                pose=landmarks[:, i],
                color=[0, 1, 1],
                name=f"lm_{i}",
                size = size,
                recognition=False,
            )

    def spawnLandmarkIds(self, landmark_ids, size=[0.01, 0.01, 0.2]):
        # Spawn landmarks in Webots
        for color, coord in landmark_ids.items():
            self.spawnMultiCoords([coord], color=color, size=size)

    def spawnMultiRandom(self, n=1, max_range=1, min_range=-1, size=[0.05,0.05,0.05], color="blue"):
        pos = []
        for i in range(n):
            pose = [
                np.random.uniform(min_range, max_range),
                np.random.uniform(min_range, max_range),
            ]
            pos.append(pose)
            self.spawnObject(
                pose=pose,
                size=size,
                color=self.color_map[color],
            )
        return pos

    def spawnMultiCoords(self, coords, size=[0.01, 0.01, 0.2], color="blue"):
        pos = []
        for i in range(len(coords)):
            pose = coords[i]
            pos.append(pose)
            self.spawnObject(
                pose=pose,
                size=size,
                color=self.color_map[color],
            )
        return pos
    
    # add an existing object to objects list
    def addObject(self, name):
        obj_node = self.getFromDef(name)
        self.objects[name] = obj_node
        return obj_node

    # set goal
    # set goal
    def setGoal(self, goto, show=False):
        self.goal = goto
        self.log.info(f"goal  -->\tx: {goto[0]}, \ty: {goto[1]}")
        if(show):
            self.spawnObject(name="wp", pose=goto, size=[0.02,0.02,0.02], color=[1,0,0])

    # go to goal
    def gotoGoal(self, precision=0.05, stop=False):
        finished = False
        # calculate heading to goal
        preheading = math.atan2((self.goal[1]-self.pose[1]), (self.goal[0]-self.pose[0]))
        heading = preheading - self.pose[2]
        heading_corrected = round((math.atan2(math.sin(heading), math.cos(heading))), 4)
        #calculate distance to goal 
        distance = ((self.pose[0] - self.goal[0])**2 + (self.pose[1] - self.goal[1])**2)**0.5
        vel = self.inverseKinematics(64, 2*heading_corrected)
        self.setMotorSpeed(vel[0], vel[1])
        if distance < precision:
            self.log.info(f"GOAL AT CORDINATES {self.goal} REACHED")
            if stop: self.setMotorSpeed(0, 0)
            finished = True
        return distance, heading_corrected, finished

    def stopRobot(self, stop_loop=True):
        self.setMotorSpeed(0, 0)
        if stop_loop:
            self.halt = True

    def getRealPose(self):
        x = self.super.getPosition()[0]
        y = self.super.getPosition()[1]
        o = self.super.getOrientation()
        o = np.arctan2(o[3], o[0])
        return [x, y, o]

    def inverseKinematics(self, vel, rot):
        r = self.charecteristics["Wheel Radius"]
        l = self.charecteristics["Axle Length"]
        # rot = np.clip(rot, -(np.pi), np.pi)
        vr = (vel + (l / 2) * rot) / r
        vl = (vel - (l / 2) * rot) / r
        # if vr > self.charecteristics["Max Velocity"]:
        #     vr = self.charecteristics["Max Velocity"]
        # if vl > self.charecteristics["Max Velocity"]:
        #     vl = self.charecteristics["Max Velocity"]
        return vr, vl

    def forwardKinematics_0(self, update=True):
        self.log.info(f"---- {self.counter} ----")
        x = self.super.getPosition()[0]
        y = self.super.getPosition()[1]
        o = self.super.getOrientation()
        o = np.arctan2(o[3], o[0])
        self.log.info(
            f"pose  -->  \tx: {round(x, 3)}, \ty: {round(y,3)}, \tθ: {round(np.degrees(o), 0)}"
        )
        state = [x, y, o]
        if update:
            self.controls = [
                np.sqrt(
                    (state[0] - self.pose[0]) ** 2 + (state[1] - self.pose[1]) ** 2
                ),
                state[2] - self.pose[2],
            ]
            self.hist = np.append(
                self.hist, np.array([[self.pose[0], self.pose[1]]]), axis=0
            )
            self.pose = state
        return state, self.controls

    def forwardKinematics_1(self, update=True, imu=True):
        self.log.info(f"---- {self.counter} ----")
        x = self.gps.getValues()[0]
        y = self.gps.getValues()[1]
        if imu:
            o = self.imu.getRollPitchYaw()[2]
        else:
            x = self.compass.getValues()[0]
            y = self.compass.getValues()[1]
            radians = np.arctan2(y, x)
            degrees = 360 - ((radians * 180) / np.pi - 90) % 360
            o = np.radians(degrees)
        self.counter += 1
        self.log.info(
            f"pose  -->  \tx: {round(x, 3)}, \ty: {round(y,3)}, \tθ: {round(np.degrees(o), 0)}"
        )
        state = [x, y, o]
        if update:
            self.controls = [
                np.sqrt(
                    (state[0] - self.pose[0]) ** 2 + (state[1] - self.pose[1]) ** 2
                ),
                state[2] - self.pose[2],
            ]
            self.hist = np.append(
                self.hist, np.array([[self.pose[0], self.pose[1]]]), axis=1
            )
            self.pose = state
        return state, self.controls

    def forwardKinematics_2(self, update=True, delta=1, imu=False):
        self.counter += 1
        self.log.info(f"---- {self.counter} ----")
        length = self.charecteristics["Axle Length"] / 1000
        radius = self.charecteristics["Wheel Radius"] / 1000
        right_encoder = self.rightEncoder.getValue() - self.rightEncoder_offset
        left_encoder = self.leftEncoder.getValue() - self.leftEncoder_offset
        __curr_enc = np.asarray([right_encoder, left_encoder]) * radius
        difference = __curr_enc - self.__prev_enc
        vel = (difference[0] + difference[1]) / 2
        rot = (difference[0] - difference[1]) / length
        self.controls = [vel, rot]
        o = self.pose[2] + (rot * delta)
        if imu:
            o = self.imu.getRollPitchYaw()[2]
        x = self.pose[0] + (vel * np.cos(o) * delta)
        y = self.pose[1] + (vel * np.sin(o) * delta)
        self.log.info(
            f"pose  -->  \tx: {round(x, 3)}, \ty: {round(y,3)}, \tθ: {round(np.degrees(o), 0)}"
        )
        state = [x, y, o]
        if update:
            self.controls = [
                np.sqrt(
                    (state[0] - self.pose[0]) ** 2 + (state[1] - self.pose[1]) ** 2
                ),
                state[2] - self.pose[2],
            ]
            self.hist = np.append(
                self.hist, np.array([[self.pose[0], self.pose[1]]]), axis=1
            )
            self.pose = state
        self.__prev_enc = __curr_enc
        return state, self.controls

    def spin(self, direction: str, angle=90, halt=False, delta=5, update_state=False):
        if direction == "left":
            self.setMotorSpeed(self.rotation / delta, -(self.rotation / delta))
        else:
            self.setMotorSpeed(-(self.rotation / delta), self.rotation / delta)
        state = self.pose
        while self.step(self.timestep) != -1:
            if update_state:
                self.forwardKinematics_0()
            i = np.subtract(self.forwardKinematics_0(), state)
            if abs(i[2]) > np.radians(angle):
                if halt:
                    self.halt = True
                self.setMotorSpeed(0, 0)
                break

    def move(self, direction: str, dist=0.5, halt=False, delta=2, update_state=False):
        if direction == "forward":
            self.setMotorSpeed(self.rotation / delta, self.rotation / delta)
        else:
            self.setMotorSpeed(-(self.rotation / delta), -(self.rotation / delta))
        state = self.pose
        while self.step(self.timestep) != -1:
            if update_state:
                self.forwardKinematics_0()
            i = np.subtract(self.forwardKinematics_0(), state)
            if np.hypot(i[0], i[1]) > dist:
                if halt:
                    self.halt = True
                self.setMotorSpeed(0, 0)
                break

    def getGlobalPoint(self, degrees, distance):
        # local cordinates
        lo = np.radians(-degrees - 180)
        lx = distance * np.cos(lo)
        ly = distance * np.sin(lo)
        # global cordinates
        x, y, o = self.pose
        local_cordinates = np.array([lx, ly])
        rotation_matrix = np.array([[np.cos(o), -np.sin(o)], [np.sin(o), np.cos(o)]])
        robot_cordinates = np.array([x, y])
        global_cordinates = (
            np.matmul(rotation_matrix, local_cordinates) + robot_cordinates
        )
        return global_cordinates
 
    def setPath(self, waypoints, show=False, show_ahead=False):
        if show_ahead:
            show = False
            for i in range(len(waypoints)):
                self.spawnObject(name=f"wp_{i}", pose=waypoints[i], size=[0.02,0.02,0.02], color=[1,1,1])
        self.waypoints = waypoints.copy()
        self.nxt_wp, self.waypoints = self.waypoints[0], self.waypoints[1:]
        self.setGoal(self.nxt_wp, show=show)

    def followPath(self, d_th=0.05, show=False):
        distance,_,_ = self.gotoGoal(d_th)
        if distance < d_th:
            if len(self.waypoints) < 1:
                self.log.info(f"FINAL DESTINATION REACHED")
                return(True)
            
            self.log.info(f"HEADING FOR THE NEXT WAYPOINT")
            self.nxt_wp, self.waypoints = self.waypoints[0], self.waypoints[1:]
            self.setGoal(self.nxt_wp, show=show)
            

        return(False)

    def pauseSimulation(self):
        self.simulationSetMode(0)
        self.log.info("Simulation paused")

    def startSimulation(self):
        self.simulationSetMode(1)
        self.log.info("Simulation started")

    def getBlobPositionInImage(self, rgb_image, color_name, min_size=1000):        
        color_hsv = {
            'blue': ([120, 100, 100], [150, 255, 255]),
            'green': ([35, 120, 50], [85, 255, 255]),
            'red': ([0, 120, 50], [10, 255, 255]),
            'yellow': ([25, 120, 50], [35, 255, 255]),
            'magenta': ([130, 120, 50], [150, 255, 255]),
            'cyan': ([75, 120, 50], [95, 255, 255]),
            'white': ([0, 0, 150], [180, 30, 255]),
            'black': ([0, 0, 0], [180, 255, 30]),
        }
        # Check if the specified color name is valid
        if color_name not in color_hsv:
            raise ValueError("Invalid color name")
        # Get the HSV range for the specified color
        lower_bound, upper_bound = self.color_hsv[color_name]
        # Convert image to HSV
        color = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        # Get the mask with the specified color range
        mask = cv2.inRange(color, np.array(lower_bound), np.array(upper_bound))
        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # If contours are found, find the center; otherwise, return x, y as -1
        x = -1
        y = -1
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > min_size:
                moments = cv2.moments(largest_contour)
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
        return x, y, mask

    def getPositionFromBlob(self, rgb_image, depth_image, color_name, negate=False):
        x, y, mask = self.getBlobPositionInImage(rgb_image, color_name)
        if x == -1: return None, None, mask
        fov = self.camera.getFov()
        width = self.camera.getWidth()
        # Calculate the horizontal angle to the object
        angle = fov * (x - width / 2) / width
        # Calculate the distance to the object
        distance = depth_image[y, x]
        #if distans is inf, return None
        if distance == float("inf"): return None, None, mask
        return distance, -angle if negate else angle, mask


    def getCameraImage(self):
        image_bytes = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = np.frombuffer(image_bytes, np.uint8).reshape((height, width, 4))
        image = image[:, :, :3]
        image = image[:, :, ::-1]
        return(image)

    def getDepthImage(self):
        image_c_ptr = self.range.getRangeImage(data_type="buffer")
        image_np = np.ctypeslib.as_array(image_c_ptr, (self.range.getWidth() * self.range.getHeight(),))
        depth = image_np.reshape(self.range.getHeight(), self.range.getWidth())
        return(depth)  

    def showOpenCVImages(self, image, name, depth=False, size=(320, 240), pos=None):
        if not depth:
            # Translate to BGR image
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, size)
        if pos is not None:
            cv2.moveWindow(name, pos[0], pos[1])
        cv2.imshow(name, image)
        cv2.waitKey(10)

    def f(self, x, odo, v=None):
        r"""
        State transition function

        :param x: vehicle state :math:`(x, y, \theta)`
        :type x: array_like(3), ndarray(n,3)
        :param odo: vehicle odometry :math:`(\delta_d, \delta_\theta)`
        :type odo: array_like(2)
        :param v: additive odometry noise, defaults to (0,0)
        :type v: array_like(2), optional
        :return: predicted vehicle state
        :rtype: ndarray(3)

        Predict the next state based on current state and odometry
        value.  ``v`` is a random variable that represents additive
        odometry noise for simulation purposes.

        .. math::

            f: \vec{x}_k, \delta_d, \delta_\theta \mapsto \vec{x}_{k+1}

        For particle filters it is useful to apply the odometry to the
        states of all particles and this can be achieved if ``x`` is a 2D
        array with one state per row.  ``v`` is ignored in this case.

        .. note:: This is the state update equation used for EKF localization.

        :seealso: :meth:`deriv` :meth:`Fx` :meth:`Fv`
        """
        odo = base.getvector(odo, 2)

        if isinstance(x, np.ndarray) and x.ndim == 2:
            # x is Nx3 set of vehicle states, do vectorized form
            # used by particle filter
            dd, dth = odo
            theta = x[:, 2]
            return (
                np.array(x)
                + np.c_[
                    dd * np.cos(theta), dd * np.sin(theta), np.full(theta.shape, dth)
                ]
            )
        else:
            # x is a vector
            x = base.getvector(x, 3)
            dd, dth = odo
            theta = x[2]

            if v is not None:
                v = base.getvector(v, 2)
                dd += v[0]
                dth += v[1]

            return x + np.r_[dd * np.cos(theta), dd * np.sin(theta), dth]

    def Fx(self, x, odo):
        r"""
        Jacobian of state transition function df/dx

        :param x: vehicle state :math:`(x, y, \theta)`
        :type x: array_like(3)
        :param odo: vehicle odometry :math:`(\delta_d, \delta_\theta)`
        :type odo: array_like(2)
        :return: Jacobian matrix
        :rtype: ndarray(3,3)

        Returns the Jacobian matrix :math:`\frac{\partial \vec{f}}{\partial \vec{x}}` for
        the given state and odometry.

        :seealso: :meth:`f` :meth:`deriv` :meth:`Fv`
        """
        dd, dth = odo
        theta = x[2]

        # fmt: off
        J = np.array([
                [1,   0,  -dd * np.sin(theta)],
                [0,   1,   dd * np.cos(theta)],
                [0,   0,   1],
            ])
        # fmt: on
        return J

    def Fv(self, x, odo):
        r"""
        Jacobian of state transition function df/dv

        :param x: vehicle state :math:`(x, y, \theta)`
        :type x: array_like(3)
        :param odo: vehicle odometry :math:`(\delta_d, \delta_\theta)`
        :type odo: array_like(2)
        :return: Jacobian matrix
        :rtype: ndarray(3,2)

        Returns the Jacobian matrix :math:`\frac{\partial \vec{f}}{\partial \vec{v}}` for
        the given state and odometry.

        :seealso:  :meth:`f` :meth:`deriv` :meth:`Fx`
        """
        dd, dth = odo
        theta = x[2]

        # fmt: off
        J = np.array([
                [np.cos(theta),    0],
                [np.sin(theta),    0],
                [0,           1],
            ])
        # fmt: on
        return J

    def plot_xy(self, *args, block=False, **kwargs):
        """
        Plot xy-path from history

        :param block: block until plot dismissed, defaults to False
        :type block: bool, optional
        :param args: positional arguments passed to :meth:`~matplotlib.axes.Axes.plot`
        :param kwargs: keyword arguments passed to :meth:`~matplotlib.axes.Axes.plot`


        The :math:`(x,y)` trajectory from the simulation history is plotted as
        :math:`x` vs :math:`y.

        :seealso: :meth:`run` :meth:`plot_xyt`
        """
        if args is None and "color" not in kwargs:
            kwargs["color"] = "b"
        xyt = self.hist
        plt.plot(xyt[:, 0], xyt[:, 1], *args, **kwargs)
        plt.show(block=block)



class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, motion_model, observation_model):
        self.state = initial_state  # Initial state [x, y, theta]
        self.covariance = initial_covariance  # Initial covariance matrix
        self.motion_model = motion_model
        self.observation_model = observation_model

    def predict(self, dt, u):
        # Predict the next state using the motion model
        self.state, jacobian_motion = self.motion_model(self.state, u, dt)
        # Update the covariance matrix using the motion model's Jacobian
        self.covariance = np.dot(jacobian_motion, np.dot(self.covariance, jacobian_motion.T))

    def update(self, z, landmark, landmark_id):
        # Compute measurement prediction using the observation model
        expected_measurement, jacobian_observation = self.observation_model(self.state, landmark)
        # Get the specific landmark from the dictionary using its ID
        actual_measurement = z[landmark_id]
        # Calculate the difference between the actual and expected measurements
        measurement_residual = np.subtract(actual_measurement, expected_measurement)
        # Update the state based on the measurement residual and the observation model's Jacobian
        kalman_gain = np.dot(self.covariance, np.dot(jacobian_observation.T, np.linalg.inv(
            np.dot(jacobian_observation, np.dot(self.covariance, jacobian_observation.T)) + np.eye(len(actual_measurement))
        )))
        self.state += np.dot(kalman_gain, measurement_residual) 
        # Update the covariance matrix
        self.covariance = np.dot(np.eye(len(self.state)) - np.dot(kalman_gain, jacobian_observation), self.covariance)


# Define the motion model (for simplicity, assume a constant velocity model)
def velocity_model(state, u, dt):
    # State transition function
    F = np.array([[1, 0, u * np.cos(state[2]) * dt],
                  [0, 1, u * np.sin(state[2]) * dt],
                  [0, 0, 1]])
    # Predicted next state
    next_state = np.dot(F, state)
    # Jacobian of the motion model
    jacobian = F
    return next_state, jacobian


# Define the observation model for a landmark
def observation_model(state, landmark):
    # Extract landmark coordinates
    x_landmark, y_landmark = landmark['x'], landmark['y']
    
    # Calculate expected measurement (distance and angle)
    delta_x = x_landmark - state[0]
    delta_y = y_landmark - state[1]
    expected_distance = np.sqrt(delta_x**2 + delta_y**2)
    expected_theta = np.arctan2(delta_y, delta_x) - state[2]
    
    expected_measurement = np.array([expected_distance, expected_theta])
    
    # Jacobian of the observation model
    jacobian = np.array([[-delta_x / expected_distance, -delta_y / expected_distance, 0],
                         [delta_y / (delta_x**2 + delta_y**2), -delta_x / (delta_x**2 + delta_y**2), -1]])
    
    return expected_measurement, jacobian