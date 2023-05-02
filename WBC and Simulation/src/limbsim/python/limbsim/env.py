"""
@file env.py
@package limbsim
@author Jun LI (junlileeds@gmail.com)
@license License BSD-3-Clause
@Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
@date 2021-08-16
"""

try:
    # use standard Python importlib if available (Python>3.7)
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources
import time
import pybullet
import csv


class BulletEnv:
    """This class manages a PyBullet simulation environment and provides utility 
       functions to interact with :py:obj:`RobotWrapper` objects.

    Attributes:
        dt (float): The length of the simulation integration step.
        step_counter (int): The number of times the simulation has been integrated.
        data_rendering (bool): Whether to render camera data at each step.
        objects (list): The list of the PyBullet ids for all the non-robot objects.
        robots (list): The list of the robot wrapper of all added robots.
    """    
    def __init__(self, server=pybullet.GUI, dt=0.001):
        """Initializes the PyBullet client.

        Args:
            server (int, optional): PyBullet server mode. pybullet.GUI creates a 
                graphical frontend using OpenGL while pybullet.DIRECT does not. 
                Defaults to pybullet.GUI.
            dt (float, optional): The length of the simulation integration step.
                Defaults to 0.001.
        """        
        self.dt = dt
        self.step_counter = 0
        self.data_rendering = False
        self.objects = []
        self.robots = []
        self.track_robot = False
        self.robot_to_track = 0

        self.physics_client = pybullet.connect(server, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.0')
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        self.configure_gui()

    def add_robot(self, robot):
        """Adds a :py:obj:`RobotWrapper` object.

        Args:
            robot (:py:obj:`RobotWrapper`): Instance of a robot wrapper.

        Returns:
            robot (:py:obj:`RobotWrapper`): Instance of a robot wrapper.
        """
        self.robots.append(robot)
        return robot

    def add_object_from_urdf(
        self, urdf_path, pos=[0, 0, 0], orn=[0, 0, 0, 1], use_fixed_base=True
    ):
        """Adds an object described by a URDF file.

        Args:
            urdf_path (str): The absolute path of the URDF file.
            pos (list, optional): The initial position of the object in the world 
                frame. Defaults to [0, 0, 0].
            orn (list, optional): The initial orientation of the object in the 
                world frame, expressed in quaternions. Defaults to [0, 0, 0, 1].
            use_fixed_base (bool, optional): Determines if the robot base is fixed 
                or not. Defaults to True.

        Returns:
            [int]: The PyBullet id of the object if added successfully.
        """    
        # Load the object.
        urdfFlags = pybullet.URDF_USE_SELF_COLLISION
        object_id = pybullet.loadURDF(urdf_path, flags=urdfFlags, useFixedBase=use_fixed_base)
        pybullet.resetBasePositionAndOrientation(object_id, pos, orn)
        self.objects.append(object_id)
        return object_id

    def configure_gui(self, data_rendering=False, camera_distance=1, 
        camera_yaw=45, camera_pitch=-30, camera_target_position=[0., 0., 0.7]
    ):
        """Configures GUI.

        Args:
            data_rendering (bool, optional): Whether to render camera data.
            camera_distance (float, optional): Distance from eye to camera 
                target position.
            camera_yaw (float, optional): Camera yaw angle (in degrees) left/right.
            camera_pitch (float, optional): Camera pitch angle (in degrees) up/down.
            camera_target_position (list, optional): Camera focus point.
        """
        self.data_rendering = data_rendering
        if self.data_rendering:
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
        else:
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

        pybullet.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target_position
        )

    def start_video_recording(self, file_name):
        """Starts video recording and save as a mp4 file.

        Args:
            file_name (str): The absolute path of the file to be saved.
        """        
        self.file_name = file_name
        self.log_id = pybullet.startStateLogging(
            pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name
        )

    def stop_video_recording(self):
        """Stops video recording if any.
        """        
        if hasattr(self, "file_name") and hasattr(self, "log_id"):
            pybullet.stopStateLogging(self.log_id)

    def debug(self):
        """Receives space key event to pause the simulation or start it again.
        """
        self.keys = pybullet.getKeyboardEvents()
        # Pause simulation by space key event
        space_key = ord(' ')
        if space_key in self.keys and self.keys[space_key] & pybullet.KEY_WAS_TRIGGERED:
            print("Simulation Paused!")
            print("Press Space key to start again!")
            while True:
                keys = pybullet.getKeyboardEvents()
                if space_key in keys and keys[space_key] & pybullet.KEY_WAS_TRIGGERED:
                    break

    def step(self, sleep=False):
        """Integrates the simulation one step forward.

        Args:
            sleep (bool, optional): Determines if the simulation sleeps for 
                :py:attr:`~dt` seconds at each step. Defaults to False.
        """        
        if sleep:
            time.sleep(self.dt)
        pybullet.stepSimulation()
        self.step_counter += 1

        for robot in self.robots:
            robot.compute_numerical_quantities(self.dt)

        if self.data_rendering:
            pybullet.getCameraImage(320,200)
            
        if self.track_robot == True:
            robot_pos = self.robots[self.robot_to_track].get_base_position()
            lookat = [robot_pos[0], robot_pos[1], robot_pos[2]]
            camInfo = pybullet.getDebugVisualizerCamera()
            distance = camInfo[10]
            pitch = camInfo[9]
            yaw = camInfo[8]
            pybullet.resetDebugVisualizerCamera(distance, yaw, pitch, lookat)

    def print_physics_engine_params(self):
        """Prints the parameters of the physics engine.
        """        
        params = pybullet.getPhysicsEngineParameters(self.physicsClient)
        print("physics_engine_params:")
        for key in params:
            print("    - ", key, ": ", params[key])

    def get_time_since_start(self):
        """Gets the time passed (in seconds) since the simulation starts.
        """
        return self.step_counter * self.dt
        
        
    def trackRobot(self, robot_id, enable_tracking=True):
        """Camera tracks a robot of a given id.
        
        Args:
            robot_id (int): the id of the robot to be tracked (only one robot
                            can be tracked at a time).
                            
            enable_tracking (bool, optional): turns tracking either on or off. 
        """
        self.track_robot = enable_tracking
        self.robot_to_track = robot_id


class BulletEnvWithGround(BulletEnv):
    """This class provides a shortcut to construct a PyBullet simulation 
       environment with a flat ground.
    """    
    def __init__(self, server=pybullet.GUI, dt=0.001):     
        super().__init__(server, dt)
        with importlib_resources.path(__package__, "env.py") as p:
            package_dir = p.parent.absolute()
        plane_urdf = str(
            package_dir / "resources" / "plane_with_restitution.urdf"
        )
        self.add_object_from_urdf(plane_urdf)

        """
        # These objects are used in demonstration: picking box up off table
        boxId = pybullet.loadURDF("/home/will/humanoid_ws/src/arm_ik_wbc/models/block.urdf", [-0.135, -0.353, 0.5], pybullet.getQuaternionFromEuler([0,0,0]), useFixedBase = False)
        pybullet.changeDynamics(boxId, -1, lateralFriction=1)
        tableId = pybullet.loadURDF("/home/will/humanoid_ws/src/arm_ik_wbc/models/table.urdf", [-0.275, -0.203, -0.1], pybullet.getQuaternionFromEuler([0,0,0]), useFixedBase = True)
        pybullet.changeDynamics(tableId, -1, lateralFriction=1)
               
        """
        # These objects are used in demonstration: picking box up off floor
        boxId = pybullet.loadURDF("/home/will/humanoid_ws/src/arm_ik_wbc/models/block.urdf", [-0.136, -0.356, 0.25], pybullet.getQuaternionFromEuler([0,0,0]), useFixedBase = False)
        pybullet.changeDynamics(boxId, -1, lateralFriction=1)
        tableId = pybullet.loadURDF("/home/will/humanoid_ws/src/arm_ik_wbc/models/cube.urdf", [-0.136, -0.35, 0.05], pybullet.getQuaternionFromEuler([0,0,0]), useFixedBase = True)
        pybullet.changeDynamics(tableId, -1, lateralFriction=1)
        
       
       
        #boxId = pybullet.loadURDF("/home/will/humanoid_ws/src/arm_ik_wbc/models/cube.urdf", [-0.136, -0.0, 0.03], pybullet.getQuaternionFromEuler([0,0,0]), useFixedBase = True)
        #pybullet.changeDynamics(boxId, -1, lateralFriction=1)
