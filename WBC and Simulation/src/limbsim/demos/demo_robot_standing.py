"""
@file demo_robot_standing.py
@package limbsim
@author Jun LI (junlileeds@gmail.com)
@license License BSD-3-Clause
@Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
@date 2022-05-18

Simple demo showing how the simulation setup works and enable robot standing.
"""

import os
import sys
import inspect
import time
import numpy as np

from commutils.yaml_parser import load_yaml
from limbsim.env import BulletEnvWithGround
from limbsim.sim_robot_setting import SimRobotSetting
from limbsim.sim_robot_interface import SimRobotInterface

# absolute directory of this package
rootdir = os.path.dirname(os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe()))))

TIME_STEP = 0.002  # 500 Hz
MAX_TIME_SECS = 10  # maximum time to run the robot.

def main(argv):
    # Load configuration file
    if len(argv) == 1:
        cfg_file = argv[0]
    else:
        raise RuntimeError("Usage: python3 ./demo.py /<config file within root folder>")

    # ! Create a PyBullet simulation environment before any robots !
    env = BulletEnvWithGround(dt=TIME_STEP)  # 500 Hz

    # Create a robot instance for PyBullet.
    sim_setting = SimRobotSetting()
    sim_setting.initialize(rootdir, cfg_file)
    sim_robot = SimRobotInterface(sim_setting)

    # Add the robot to the env to update the internal structure of the robot.
    env.add_robot(sim_robot)

    # Set constant control.
    configs = load_yaml(rootdir + cfg_file)
    actions = []
    joint_des_pos = np.array(configs["control_variables"]["joint_des_pos"])
    joint_des_vel = np.array(configs["control_variables"]["joint_des_vel"])
    joint_kp = np.array(configs["control_variables"]["joint_kp"])
    joint_kd = np.array(configs["control_variables"]["joint_kd"])
    for idx, jn in enumerate(configs["control_variables"]["joint_names"]):
        actions[idx*5:idx*5+4] = [
            joint_des_pos[idx], joint_kp[idx], joint_des_vel[idx], joint_kd[idx], 0
        ]

    start_time = env.get_time_since_start()
    current_time = start_time
    while current_time - start_time < MAX_TIME_SECS:
        start_time_env = current_time
        start_time_wall = time.time()

        # Get robot's measured data
        print("-----------------------------------------------------")
        print("Current time: \n", current_time)
        print("IMU angular velocity: \n", sim_robot.get_base_imu_angular_velocity())
        print("IMU linear acceleration: \n", sim_robot.get_base_imu_linear_acceleration())
        print("Joint positions: \n", sim_robot.get_joint_positions())
        print("Joint velocities: \n", sim_robot.get_joint_velocities())
        print("Joint efforts: \n", sim_robot.get_joint_efforts())
        print("Limb contact forces: \n", sim_robot.get_limb_contact_forces())
        print("-----------------------------------------------------")

        # Apply torque to robot
        sim_robot.apply_joint_actions(actions)

        # Step the simulation environment
        env.step(sleep=False)

        current_time = env.get_time_since_start()
        # Add sleep time
        expected_duration = current_time - start_time_env
        actual_duration = time.time() - start_time_wall
        if actual_duration < expected_duration:
            time.sleep(expected_duration - actual_duration)

if __name__ == "__main__":
    main(sys.argv[1:])