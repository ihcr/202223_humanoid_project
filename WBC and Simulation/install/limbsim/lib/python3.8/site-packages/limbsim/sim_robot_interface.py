"""
@file sim_robot_interface.py
@package limbsim
@author Jun LI (junlileeds@gmail.com)
@license License BSD-3-Clause
@Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
@date 2022-05-18
"""

import numpy as np
import pybullet as pyb
import time

from limbsim.joint_controller import JointController

class SimRobotInterface:
    def __init__(self, rob_setting):
        """Initializes the simulation interface of a robot.

        Args:
            rob_setting: a container for configuration parameters.
        """
        self.urdf_filename = rob_setting.urdf_filename
        self.use_fixed_base = rob_setting.use_fixed_base
        self.limb_names = rob_setting.limb_names
        self.limb_endeff_names = rob_setting.limb_endeff_names
        self.joint_names = rob_setting.joint_names
        self.base_init_pos = rob_setting.base_init_pos
        self.base_init_vel = rob_setting.base_init_vel
        self.joint_init_pos = rob_setting.joint_init_pos
        self.joint_init_vel = rob_setting.joint_init_vel

        assert len(self.limb_names) == len(self.limb_endeff_names)
        self.limb_name_endeff_name_map = {}
        self.endeff_name_limb_name_map = {}
        for idx in range(len(self.limb_names)):
            self.limb_name_endeff_name_map[self.limb_names[idx]] = self.limb_endeff_names[idx]
            self.endeff_name_limb_name_map[self.limb_endeff_names[idx]] = self.limb_names[idx]
                       
        if self.use_fixed_base:
            self.nq = len(self.joint_names)
            self.nv = len(self.joint_names)
            self.nj = len(self.joint_names)
            self.ne = len(self.limb_endeff_names)
        else:
            self.nq = len(self.base_init_pos) + len(self.joint_init_pos)
            self.nv = len(self.base_init_vel) + len(self.joint_init_vel)
            self.nj = len(self.joint_names)
            self.ne = len(self.limb_endeff_names)

        self.base_lin_vel_prev = None
        self.base_ang_vel_prev = None
        self.base_lin_acc = np.zeros(3)
        self.base_ang_acc = np.zeros(3)

        self.g = np.array([0., 0., -9.81])

        self.rng = np.random.default_rng()

        self.base_imu_acc_bias = np.zeros(3)
        self.base_imu_gyro_bias = np.zeros(3)
        self.base_imu_acc_thermal = np.zeros(3)
        self.base_imu_gyro_thermal = np.zeros(3)
        self.base_imu_acc_bias_noise = 0.0001          # m/(sec^3*sqrt(Hz))
        self.base_imu_gyro_bias_noise = 0.0000309      # rad/(sec^2*sqrt(Hz))
        self.base_imu_acc_thermal_noise = 0.00001962   # m/(sec^2*sqrt(Hz))
        self.base_imu_gyro_thermal_noise = 0.000000873 # rad/(sec*sqrt(Hz))

        self.robot_id = pyb.loadURDF(
            self.urdf_filename, 
            self.base_init_pos[0:3], 
            self.base_init_pos[3:7], 
            useFixedBase=self.use_fixed_base,
            flags=pyb.URDF_USE_INERTIA_FROM_FILE
        )


        _, self.init_orn_inv = pyb.invertTransform(
            [0,0,0], self.base_init_pos[3:7]
        )

        # Query all the joints.
        num_joints = pyb.getNumJoints(self.robot_id)
        self.joint_name_id_map = {}
        self.joint_id_name_map = {}
        self.link_name_id_map = {}
        self.link_id_name_map = {}
        for idx in range(num_joints): # link id is same it's corresponding joint id.
            joint_info = pyb.getJointInfo(self.robot_id, idx)
            joint_name = joint_info[1].decode('UTF-8')
            link_name = joint_info[12].decode('UTF-8')
            self.joint_name_id_map[joint_name] = idx
            self.joint_id_name_map[idx] = joint_name
            self.link_name_id_map[link_name] = idx
            self.link_id_name_map[idx] = link_name

            pyb.changeDynamics(
                self.robot_id, 
                idx, 
                linearDamping=0.04, 
                angularDamping=0.04, 
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.joint_torque_limits = []
        for idx, jn in enumerate(self.joint_names):
            joint_info = pyb.getJointInfo(
                self.robot_id, self.joint_name_id_map[jn])
            self.joint_torque_limits.append(joint_info[10])

        self.joint_ids = np.array(
            [self.joint_name_id_map[name] for name in self.joint_names]
        )

        # Disable the velocity control on the joints as we use torque control.
        pyb.setJointMotorControlArray(
            self.robot_id, 
            self.joint_ids, 
            pyb.VELOCITY_CONTROL, 
            forces=np.zeros(self.nj)
        )

        # Enable joint force torque sensor in each joint.
        for idx, jn in enumerate(self.joint_names):
            ji = self.joint_name_id_map[jn]
            pyb.enableJointForceTorqueSensor(
                self.robot_id, 
                ji, 
                enableSensor=True
            )

        # Mimic joint torque measurements
        self.applied_joint_torques = np.zeros(self.nj)

        # In pybullet, the contact wrench is measured as a joint. In our case 
        # the joint is fixed joint.
        print(self.limb_endeff_names)
        print(self.joint_name_id_map)
        self.limb_endeff_ids = [
            self.link_name_id_map[name] for name in self.limb_endeff_names]

        self.joint_controller = JointController(
            self.joint_names, self.joint_torque_limits)

        self.reset(
            self.base_init_pos, self.base_init_vel, 
            self.joint_init_pos, self.joint_init_vel
        )

        # These filepaths and joint configurations for data headings are specific to the project_humanoid_robot demonstrations
        self.resultsFile = open("/home/will/Documents/SimulationResults-{}.csv".format(int(time.time())), "a")
        self.resultsFile.write("""Timestamp,LA2 Pos,LA2 V,LA2 RF1,LA2 RF2,LA2 RF3,LA2 RF4,LA2 RF5,LA2 RF6,LA2 MT,,LA1 Pos,LA1 V,LA1 RF1,LA1 RF2,LA1 RF3,LA1 RF4,LA1 RF5,LA1 RF6,LA1 MT,,LK Pos,LK V,LK RF1,LK RF2,LK RF3,LK RF4,LK RF5,LK RF6,LK MT,,LHP Pos,LHP V,LHP RF1,LHP RF2,LHP RF3,LHP RF4,LHP RF5,LHP RF6,LHP MT,,LHY Pos,LHY V,LHY RF1,LHY RF2,LHY RF3,LHY RF4,LHY RF5,LHY RF6,LHY MT,,LHR Pos,LHR V,LHR RF1,LHR RF2,LHR RF3,LHR RF4,LHR RF5,LHR RF6,LHR MT,,LSR Pos,LSR V,LSR RF1,LSR RF2,LSR RF3,LSR RF4,LSR RF5,LSR RF6,LSR MT,,LSY Pos,LSY V,LSY RF1,LSY RF2,LSY RF3,LSY RF4,LSY RF5,LSY RF6,LSY MT,,LSY Pos,LSP V,LSP RF1,LSP RF2,LSP RF3,LSP RF4,LSP RF5,LSP RF6,LSP MT,,LE Pos,LE V,LE RF1,LE RF2,LE RF3,LE RF4,LE RF5,LE RF6,LE MT,,LW1 Pos,LW1 V,LW1 RF1,LW1 RF2,LW1 RF3,LW1 RF4,LW1 RF5,LW1 RF6,LW1 MT,,LW2 Pos,LW2 V,LW2 RF1,LW2 RF2,LW2 RF3,LW2 RF4,LW2 RF5,LW2 RF6,LW2 MT,,R Pos,RSR V,RSR RF1,RSR RF2,RSR RF3,RSR RF4,RSR RF5,RSR RF6,RSR MT,,RSY Pos,RSY V,RSY RF1,RSY RF2,RSY RF3,RSY RF4,RSY RF5,RSY RF6,RSY MT,,RSR Pos,RSR V,RSR RF1,RSR RF2,RSR RF3,RSR RF4,RSR RF5,RSR RF6,RSR MT,,RE Pos,RE V,RE RF1,RE RF2,RE RF3,RE RF4,RE RF5,RE RF6,RE MT,,RW1 Pos,RW1 V,RW1 RF1,RW1 RF2,RW1 RF3,RW1 RF4,RW1 RF5,RW1 RF6,RW1 MT,,RW2 Pos,RW2 V,RW2 RF1,RW2 RF2,RW2 RF3,RW2 RF4,RW2 RF5,RW2 RF6,RW2 MT,,RHR Pos,RHR V,RHR RF1,RHR RF2,RHR RF3,RHR RF4,RHR RF5,RHR RF6,RHR MT,,RHY Pos,RHY V,RHY RF1,RHY RF2,RHY RF3,RHY RF4,RHY RF5,RHY RF6,RHY MT,,RHP Pos,RHP V,RHP RF1,RHP RF2,RHP RF3,RHP RF4,RHP RF5,RHP RF6,RHP MT,,RK Pos,RK V,RK RF1,RK RF2,RK RF3,RK RF4,RK RF5,RK RF6,RK MT,,RA1 Pos,RA1 V,RA1 RF1,RA1 RF2,RA1 RF3,RA1 RF4,RA1 RF5,RA1 RF6,RA1 MT,,RA2 Pos,RA2 V,RA2 RF1,RA2 RF2,RA2 RF3,RA2 RF4,RA2 RF5,RA2 RF6,RA2 MT,,\n""")
        self.simulationStartTime = time.time()


        self.compute_numerical_quantities()

    def reset(self, base_pos, base_vel, joint_pos, joint_vel):
        """Set the robot to the desired states. Note that the base velocities
           are expressed in the base frame.

        Args:
            base_pos (ndarray): Desired base pose.
            base_vel (ndarray): Desired base velocity.
            joint_pos (ndarray): Desired joint positions.
            joint_vel (ndarray): Desired joint velocities.
        """
        vec2list = lambda m: np.array(m.T).reshape(-1).tolist()
        pyb.resetBasePositionAndOrientation(
            self.robot_id, 
            vec2list(base_pos[0:3]), 
            vec2list(base_pos[3:7])
        )

        # Pybullet assumes the base velocity to be aligned with the world frame.
        rot = np.matrix(pyb.getMatrixFromQuaternion(
            vec2list(base_pos[3:7]))).reshape((3, 3))
        pyb.resetBaseVelocity(
            self.robot_id, 
            vec2list(rot.dot(base_vel[0:3])), 
            vec2list(rot.dot(base_vel[3:6]))
        )

        for idx, jn in enumerate(self.joint_names):
            pyb.resetJointState(
                self.robot_id, 
                self.joint_name_id_map[jn], 
                joint_pos[idx], 
                joint_vel[idx]
            )

    def get_raw_base_imu_angular_velocity(self):
        """Get polluted base IMU gyroscope angular velocity. Assume that 
        the IMU coordinate coincides with the robot's body coordinate.

        Returns:
            np.array((3,1)) IMU gyroscope angular velocity (base frame).
        """
        return (self.imu_ang_vel + self.base_imu_gyro_bias + 
            self.base_imu_gyro_thermal + 0.015*self.rng.standard_normal(3))

    def get_base_imu_angular_velocity(self):
        """Get simulated base IMU gyroscope angular velocity. Assume that 
        the IMU coordinate coincides with the robot's body coordinate.

        Returns:
            np.array((3,1)) IMU gyroscope angular velocity (base frame).
        """
        return (self.imu_ang_vel + self.base_imu_gyro_bias + 
            self.base_imu_gyro_thermal)

    def get_raw_base_imu_linear_acceleration(self):
        """Get polluted base IMU accelerometer acceleration. Assume that 
        the IMU coordinate coincides with the robot's body coordinate.

        Returns:
            np.array((3,1)) IMU accelerometer acceleration (base frame, gravity offset).
        """
        return (self.imu_lin_acc + self.base_imu_acc_bias + 
            self.base_imu_acc_thermal + 0.015*self.rng.standard_normal(3))

    def get_base_imu_linear_acceleration(self):
        """Get simulated base IMU accelerometer acceleration. Assume that 
        the IMU coordinate coincides with the robot's body coordinate.

        Returns:
            np.array((3,1)) IMU accelerometer acceleration (base frame, gravity offset).
        """
        return (self.imu_lin_acc + self.base_imu_acc_bias + 
            self.base_imu_acc_thermal)

    def get_raw_joint_positions(self):
        """Get the robot's joint positions polluted by noise.

        Returns:
            joint_positions (ndarray): Joint positions.
        """
        joint_pos = []
        for idx, jn in enumerate(self.joint_names):
            joint_pos.append(self.joint_states[idx][0])
        return (np.array(joint_pos) + 0.005*self.rng.standard_normal(self.nj))

    def get_joint_positions(self):
        """Get the robot's joint positions.

        Returns:
            joint_positions (ndarray): Joint positions.
        """
        joint_pos = []
        for idx, jn in enumerate(self.joint_names):
            joint_pos.append(self.joint_states[idx][0])
        return np.array(joint_pos)

    def get_raw_joint_velocities(self):
        """Get the robot's joint velocities polluted by noise.

        Returns:
            joint_velocities (ndarray): Joint velocities.
        """
        joint_vel = []
        for idx, jn in enumerate(self.joint_names):
            joint_vel.append(self.joint_states[idx][1])
        return (np.array(joint_vel) + 0.15*self.rng.standard_normal(self.nj))

    def get_joint_velocities(self):
        """Get the robot's joint velocities.

        Returns:
            joint_velocities (ndarray): Joint velocities.
        """
        joint_vel = []
        for idx, jn in enumerate(self.joint_names):
            joint_vel.append(self.joint_states[idx][1])
        return (np.array(joint_vel))

    def get_raw_joint_efforts(self):
        """Get the robot's joint forces (prismatic) and torques (revolute) polluted 
        by noise.

        Returns: 
            joint_efforts (ndarray): Joint forces and torques.
        """
        joint_eft = self.applied_joint_torques + self.rng.standard_normal(self.nj)
        return joint_eft

    def get_joint_efforts(self):
        """Get the robot's joint forces (prismatic) and torques (revolute).

        Returns: 
            joint_efforts (ndarray): Joint forces and torques.
        """
        return self.applied_joint_torques

    def get_raw_limb_contact_forces(self):
        """Get robot's limb contact forces with the environment.

        Returns:
            contact_forces (ndarray): limb contact forces in normal direction.
        """
        contact_forces = []
        for endeff_name in self.limb_endeff_names:
            endeff_id = self.link_name_id_map[endeff_name]
            pt = pyb.getContactPoints(self.robot_id, -1, endeff_id)
            if len(pt) == 0:
                normal_force = 0.
            else:
                normal_force = pt[0][9]
            contact_forces.append(normal_force)
        return np.array(contact_forces) + self.rng.standard_normal(self.ne)

    def get_limb_contact_forces(self):
        """Get robot's limb contact forces with the environment.

        Returns:
            contact_forces (ndarray): limb contact forces in normal direction.
        """
        contact_forces = []
        for endeff_name in self.limb_endeff_names:
            endeff_id = self.link_name_id_map[endeff_name]
            pt = pyb.getContactPoints(self.robot_id, -1, endeff_id)
            if len(pt) == 0:
                normal_force = 0.
            else:
                normal_force = pt[0][9]
            contact_forces.append(normal_force)
        return np.array(contact_forces)

    def get_base_position(self):
        """Get the robot's base position.

        Returns:
            base_pos (ndarray): base position.
        """
        return np.array(self.base_pos)

    def get_base_rotation(self):
        """Get the robot's base orientation in rotation matrix.

        Returns:
            base_orn (ndarray): rotation matrix of base orientation.
        """
        return np.array(pyb.getMatrixFromQuaternion(self.base_orn))

    def get_base_quaternion(self):
        """Get the robot's base orientation in quaternion.

        Returns:
            base_orn (ndarray): quaternion of base orientation [x,y,z,w].
        """
        return np.array(self.base_orn)

    def get_base_euler_rpy(self):
        """Get the robot's base orientation in quaternion.

        Returns:
            base_orn (ndarray): euler rpy of base orientation.
        """
        return np.array(pyb.getEulerFromQuaternion(self.base_orn))

    def get_base_linear_velocity(self):
        """Get the robot's base linear velocity in base frame.

        Returns:
            base_lin_vel (ndarray): base linear velocity.
        """
        return np.array(self.base_lin_vel)

    def get_base_angular_velocity(self):
        """Get the robot's base angular velocity in base frame.

        Returns:
            base_ang_vel (ndarray): base angular velocity.
        """
        return np.array(self.base_ang_vel)

    def get_link_pose(self, name):
        """Get the link frame pose in world frame.

        Returns:
            frame_pose (ndarray): frame pose.
        """
        state = pyb.getLinkState(self.robot_id, self.link_name_id_map[name])
        frame_pos = np.array(state[4])
        frame_orn = np.array(state[5])
        frame_pose = np.concatenate((frame_pos, frame_orn))
        return frame_pose

    def get_link_velocity(self, name):
        """Get the link frame velocity in world frame.

        Returns:
            frame_velocity (ndarray): frame velocity.
        """
        state = pyb.getLinkState(
            self.robot_id, self.link_name_id_map[name], computeLinkVelocity=1)
        frame_lin_vel = np.array(state[6])
        frame_ang_vel = np.array(state[7])
        frame_velocity = np.concatenate((frame_lin_vel, frame_ang_vel))
        return frame_velocity

    def get_limb_contact_states(self):
        """Get robot's limb contact situation with the environment.

        Returns:
            A list of 4 booleans. The i-th boolean is True if limb i is in contact
            with the environment.
        """
        contact_states = {}
        for limb_time in self.limb_names:
            contact_states[limb_time] = False

        all_contacts = pyb.getContactPoints(self.robot_id)
        for contact in all_contacts:
            if contact[2] == self.robot_id:
                continue
            if contact[3] in self.limb_endeff_ids:
                contact_states[self.endeff_name_limb_name_map[
                    self.link_id_name_map[contact[3]]]] = True

        states = []
        for i, limb_time in enumerate(self.limb_names):
            states.append(contact_states[limb_time])
        
        return states

    def apply_joint_actions(self, actions):
        """Apply the desired action to the joints.

        Args:
            actions (ndarray): Joint action to be applied.
        """
        joint_torque = self.joint_controller.convert_to_torque(
            self.get_joint_positions(), 
            self.get_joint_velocities(), 
            actions
        )

        self.applied_joint_torques = joint_torque

        zero_gains = len(joint_torque) * (0., )
        pyb.setJointMotorControlArray(
            self.robot_id, 
            self.joint_ids, 
            pyb.TORQUE_CONTROL, 
            forces=joint_torque, 
            positionGains=zero_gains, 
            velocityGains=zero_gains
        )

    def apply_joint_positions(self, names, positions):
        """Set the desired positions to the joints.

        Args:
            names: The joint names.
            positions: The desired joint positions.
        """
        
        joint_ids = np.array(
            [self.joint_name_id_map[name] for name in names]
        )
        pyb.setJointMotorControlArray(
            self.robot_id,
            joint_ids,
            pyb.POSITION_CONTROL,
            targetPositions=positions,
        )

    def apply_joint_velocities(self, names, velocities):
        """Set the desired velocities to the joints.

        Args:
            names: The joint names.
            velocities: The desired joint velocities.
        """
        joint_ids = np.array(
            [self.joint_name_id_map[name] for name in names]
        )
        pyb.setJointMotorControlArray(
            self.robot_id, 
            joint_ids, 
            pyb.VELOCITY_CONTROL,
            targetsVelocities=velocities,
        )

    def compute_numerical_quantities(self, dt=0.001):
        """Compute numerical robot quantities from simulation results."""
        self.base_pos, base_orn_w = (
            pyb.getBasePositionAndOrientation(self.robot_id))
        base_lin_vel_w, base_ang_vel_w = (
            pyb.getBaseVelocity(self.robot_id))
        self.joint_states = pyb.getJointStates(
            self.robot_id, self.joint_ids)

        # Record joint state information including position, velocity, external forces, and torque
        csv_row = "{},".format(time.time()-self.simulationStartTime)
        for i in range(self.joint_ids.size):
            csv_row += "{},{},".format(self.joint_states[i][0], self.joint_states[i][1])
            for x in range(len(self.joint_states[i][2])):
                csv_row += "{},".format(self.joint_states[i][2][x])
            for x in range(6-len(self.joint_states[i][2])):
                csv_row += ","
            csv_row += "{},,".format(self.joint_states[i][3])
        self.resultsFile.write(csv_row+"\n")


        # Compute the relative orientation relative to the robot's initial 
        # orientation
        _, self.base_orn = pyb.multiplyTransforms(
            positionA=[0,0,0], orientationA=base_orn_w, 
            positionB=[0,0,0], orientationB=self.init_orn_inv)

        self.base_lin_vel = self._transform_velocity_to_base_frame(
            base_lin_vel_w, self.base_orn)
        self.base_ang_vel = self._transform_velocity_to_base_frame(
            base_ang_vel_w, self.base_orn)

        # Compute base acceleration numerically
        if self.base_lin_vel_prev is not None and self.base_ang_vel_prev is not None:
            self.base_lin_acc = (1.0/dt) * (self.base_lin_vel-self.base_lin_vel_prev)
            self.base_ang_acc = (1.0/dt) * (self.base_ang_vel-self.base_ang_vel_prev)
        
        self.base_lin_vel_prev = self.base_lin_vel
        self.base_ang_vel_prev = self.base_ang_vel

        self.imu_lin_acc = (self.base_lin_acc - 
            self._transform_velocity_to_base_frame(self.g, self.base_orn))
        self.imu_ang_vel = self.base_ang_vel

        # Integrate IMU accelerometer/gyroscope bias terms forward
        self.base_imu_acc_bias += dt * (
            self.base_imu_acc_bias_noise/np.sqrt(dt)) * self.rng.standard_normal(3)
        self.base_imu_gyro_bias += dt * (
            self.base_imu_gyro_bias_noise/np.sqrt(dt)) * self.rng.standard_normal(3)

        # Add simulated IMU sensor thermal noise.
        self.base_imu_acc_thermal += (
            self.base_imu_acc_thermal_noise/np.sqrt(dt)) * self.rng.standard_normal(3)
        self.base_imu_gyro_thermal += (
            self.base_imu_gyro_thermal_noise/np.sqrt(dt)) * self.rng.standard_normal(3)

    def _transform_velocity_to_base_frame(self, vel, orn):
        """Transform the velocity from world frame to robot's base frame."""
        # Treat velocity as a position vector, then transform based on the
        # orientation given by dividing (or multiplying with inverse).
        # Get inverse quaternion assuming the vector is at 0,0,0 origin.
        _, orn_inv = pyb.invertTransform(
                position=[0,0,0], orientation=orn)
        # Transform the velocity at neutral orientation using a neutral
        # translation and reverse of the given orientation.
        rel_vel, _ = pyb.multiplyTransforms(
                positionA=[0,0,0], orientationA=orn_inv, 
                positionB=vel, orientationB=[0,0,0,1])
        return np.asarray(rel_vel)
