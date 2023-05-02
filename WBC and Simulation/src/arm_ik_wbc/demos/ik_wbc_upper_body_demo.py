import os
import sys
import inspect
import time
import numpy as np
import math
import copy

from scipy.spatial.transform import Rotation as R

from commutils.yaml_parser import load_yaml
from limbsim.env import BulletEnvWithGround
from limbsim.sim_robot_setting import SimRobotSetting
from limbsim.sim_robot_interface import SimRobotInterface
from arm_ik_wbc.ik_wbc import IkWBC
from arm_ik_wbc.trajectory_planner import TrajectoryPlanner

# ROS
#from IkWbc.msg import ik_wbc_out

# absolute directory of this package
rootdir = os.path.dirname(os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe()))))

TIME_STEP = 0.002  # 500 Hz
MAX_TIME_SECS = 100  # maximum time to run the robot.

def main(argv):
    # Load configuration file
    if len(argv) == 1:
        cfg_file = argv[0]
    else:
        raise RuntimeError("Usage: python3 ./ik_wbc_arm_demo.py /<config file within root folder>")

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
    
    # fetch robot params
    joint_des_pos = np.array(configs["control_variables"]["joint_des_pos"])
    joint_names = configs["control_variables"]["joint_names"]
    arm_urdf_path = rootdir + configs["sim_robot_variables"]["urdf_filename"]
    arm_mesh_path = rootdir + configs["sim_robot_variables"]["mesh_foldername"]
    fixed_base = configs["sim_robot_variables"]["use_fixed_base"]
    EE_frame_name = configs["sim_robot_variables"]["limb_endeff_frame_names"]
    EE_joint_name = configs["sim_robot_variables"]["limb_endeff_joint_names"]
    base_frame_name = configs["sim_robot_variables"]["base_name"]
    
    # Initialise ik wbc
    controller = IkWBC(arm_urdf_path, arm_mesh_path, EE_frame_name, EE_frame_name, base_frame_name, joint_des_pos, fixed_base)
    
    # Initialise targect dict
    EE_target_pos = [0]*(len((controller.EE_index_list_frame))+1)
    EE_target_ori = [0]*(len((controller.EE_index_list_frame))+1)
#
    for i in range(len(controller.EE_index_list_frame)):
        EE_target_pos[i] = controller.EE_frame_pos[i].reshape(3,1)
        rot_mat = R.from_matrix(controller.EE_frame_ori[i])
        rot_euler = rot_mat.as_euler('xyz')
        EE_target_ori[i] = rot_euler.reshape(3,1)

    EE_target_pos[i+1] = np.array([0,0,0])
    EE_target_ori[i] = np.array([0,0,0])

    target_dict = {"task name":[], "target pos":[], "target ori":[]}
    target_dict["task name"] = EE_frame_name
    target_dict["target pos"] = copy.deepcopy(EE_target_pos)    
    target_dict["target ori"] = copy.deepcopy(EE_target_ori)
    
    # Initialise planner
    planner = TrajectoryPlanner(copy.deepcopy(target_dict))
    
    start_time = env.get_time_since_start()
    current_time = start_time
    
    # DRAW A SQUARE
    while current_time - start_time < MAX_TIME_SECS:
        
        #0

        target_pos = [[0.1, 0.1, 0.1], [-0.1, 0.1, 0.1]]
        target_ori = [[0, 0, 0], [0, 0, 0]]
        traj_interval = assignRelativeTarget(planner, target_dict, target_pos, target_ori)
        current_time = executeMovement(controller, planner, env, sim_robot, joint_names, current_time, target_dict, traj_interval)
        #time.sleep(5)

        #5
        print("Drawing step 6")
        interval, traj_interval = planner.generate_homing_trajectory(target_dict)
        current_time = executeMovement(controller, planner, env, sim_robot, joint_names, current_time, target_dict, traj_interval)

        
def assignWorldTarget(planner, target_dict, target_pos, target_ori):    
    #print("Assigning end effector target in world frame")
    #print("Current target_dict: {}".format(target_dict))
    #print("New target position: {}".format(target_pos))
    #print("New target orientation: {}".format(target_ori))
      
    end_target_dict = copy.deepcopy(target_dict)
    for i in range(len(target_pos)):
        end_target_dict["target pos"][i][0] = target_pos[i][0]
        end_target_dict["target pos"][i][1] = target_pos[i][1]
        end_target_dict["target pos"][i][2] = target_pos[i][2]
        
        end_target_dict["target ori"][i][0] = target_ori[i][0]
        end_target_dict["target ori"][i][1] = target_ori[i][1]
        end_target_dict["target ori"][i][2] = target_ori[i][2]
    print("")
    print("..........")
    print(end_target_dict)
    

    interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
    
    return traj_interval  

def assignRelativeTarget(planner, target_dict, target_pos, target_ori):    
    #print("Assigning end effector target in world frame")
    #print("Current target_dict: {}".format(target_dict))
    #print("New target position: {}".format(target_pos))
    #print("New target orientation: {}".format(target_ori))
      
    end_target_dict = copy.deepcopy(target_dict)
    print("/////////////////////")
    for i in range(len(target_pos)):
        print(i)
        print(target_pos[i])
        end_target_dict["target pos"][i][0] += target_pos[i][0]
        end_target_dict["target pos"][i][1] += target_pos[i][1]
        end_target_dict["target pos"][i][2] += target_pos[i][2]
        
        end_target_dict["target ori"][i][0] += target_ori[i][0]
        end_target_dict["target ori"][i][1] += target_ori[i][1]
        end_target_dict["target ori"][i][2] += target_ori[i][2]
    print("")
    print("..........")
    print(target_dict)
    print("..........")
    print(end_target_dict)
    print(end_target_dict["target pos"])
    print(end_target_dict["target ori"])
    interval, traj_interval = planner.generate_trajectory(target_dict, end_target_dict)
    time.sleep(1)
    
    return traj_interval  
        
        
def executeMovement(controller, planner, env, sim_robot, joint_names, current_time, target_dict, traj_interval):
    #print("ik_wbc_arm_demo2 > executeMovement")
    print(target_dict)
    for i in traj_interval:
            # Fetch new target dict
        target_dict = planner.run_stored_trajectory(target_dict, i)
        #time.sleep(1)
        # Fetch time
        start_time_env = current_time
        start_time_wall = time.time()
        # Run WBC
        joint_des_pos = controller.runWBC(target_dict)
        print("===================")
        print(joint_des_pos)
        """
        joint_des_pos[0:12] = 0
        for j in range(12): 
            for k in range(100):
                joint_des_pos[j] = 0.01*k
                #joint_des_pos[j+6] = 0.01*k
                print(joint_des_pos)
                time.sleep(0.02)

                 # Apply joint positions to robot
                sim_robot.apply_joint_positions(joint_names, joint_des_pos.tolist())

                # Step the simulation environment
                env.step(sleep=False)
        """
        # Apply joint positions to robot
        sim_robot.apply_joint_positions(joint_names, joint_des_pos.tolist())

        # Step the simulation environment
        env.step(sleep=False)


        current_time = env.get_time_since_start()
        # Add sleep time
        expected_duration = current_time - start_time_env
        actual_duration = time.time() - start_time_wall
        if actual_duration < expected_duration:
            time.sleep(expected_duration - actual_duration)
        #time.sleep(0.2)
        
    return current_time
                
                
        
                
#def execute_traversal(target_dict):


if __name__ == "__main__":
    main(sys.argv[1:])
