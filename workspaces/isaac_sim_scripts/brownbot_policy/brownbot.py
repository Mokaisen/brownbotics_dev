from typing import Optional, List

import numpy as np
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.core.utils.types import ArticulationAction
import time

class BrownbotPolicy(PolicyController):

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "brownbot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize robot and load RL policy.

        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot

        """

        if usd_path == None: 
            usd_path = "/isaac-sim/workspaces/isaac_sim_scene/brownbot_07.usd"
        
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        self.load_policy(
            "/isaac-sim/data_brownbot_dev/robots/brownbot/models/policy_scripted.pt",
            "/isaac-sim/data_brownbot_dev/robots/brownbot/models/params/env.yaml"        
        )
        self._action_scale = 0.20
        self._previous_action = np.zeros(7)
        self._policy_counter = 0
        self._obs_counter = 0

        self._close_gripper = False
        self._clossing_gripper = False
        self._gripper_counter = 0 

        self.target_pose_np = np.array([5.5306e-01,  1.9388e-01,  5.8111e-01]) # x, y, z

    def _compute_observation(self, cube_position:np.ndarray ):
        """
        Compute the observation vector for the policy

        Argument:

        Returns:
        np.ndarray -- The observation vector.

        """

        obs = np.zeros(45, dtype=np.float32)

        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[:14] = current_joint_pos - self.default_pos
        obs[14:28] = current_joint_vel

        #print("current_joint_pos: ", current_joint_pos)
        #print("default_pos: ", self.default_pos)
        #print("obs[:14]: ", obs[:14])

        # object position in robot root frame
        # TODO how to access this position? 
        obs[28:31] = cube_position 
        #obs[28:31] = [4.2191e-01, -2.0915e-01, 4.5190e-02]

        # Object target pose
        # TODO how to access the mdp.generated_commands
        # obs[31:38] = [0.5,0,0.4,0,0,0,0]
        obs[31:38] = self.target_pose_np.tolist() +  [8.4452e-04,
                      7.0711e-01, -2.8151e-04,  7.0711e-01]

        # previous action
        obs[38:45] = self._previous_action

        return obs

    def forward(self, dt, cube_position: np.ndarray, contact_sensors: List, replay_obs=None):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt (float) -- Timestep update in the world.

        """
        if self._policy_counter % self._decimation == 0 and self._clossing_gripper == False:
            if replay_obs is None:
                obs = self._compute_observation(cube_position)
            else:
                if self._obs_counter < len(replay_obs):
                    obs = replay_obs[self._obs_counter]
                    self._obs_counter += 1
                else:
                    return
            #print("obs: ", obs) # Debugging observation
            self.action = self._compute_action(obs)
            #print("action raw: ", self.action)
            #self.action = np.pad(self.action, (0, len(self.default_pos) - len(self.action)), mode="constant")
            #self.joint_command = self.default_pos + (self.action * self._action_scale)
            self._previous_action = self.action.copy()
        
        # print("shape self.action: ", self.action.shape)
        # print("type self.action: ", type(self.action))
        # print("type default_pose: ", type(self.default_pos))
        # print("len self.default_pos: ", len(self.default_pos))
        # Pad with zeros
        padded_action = np.pad(self.action, (0, len(self.default_pos) - len(self.action)), mode="constant")
        padded_action[6] = padded_action[6] * (-1.2)

        # Create a scaling array: scale first 6 elements, keep the rest unscaled
        scaling_array = np.ones_like(padded_action)
        scaling_array[:6] = self._action_scale  # Apply scale only to first 6

        # Apply scaling only to first 6 elements
        scaled_action = padded_action * scaling_array

        joint_command = self.default_pos + scaled_action

        action = ArticulationAction(joint_positions=joint_command)
        #action.joint_positions = action.joint_positions[:7]
        #action.joint_positions[:7] = [0.0,  -1.710e+00, 1.7, 
        #                               -1.70e+00, -1.7e+00, 0.0226e+00, 0.0] 
        #action.joint_positions[6] = 1.7
        #action.joint_positions[0] = -1.000
        
        # if action.joint_positions[6] <= 0:
        #     action.joint_positions[6] = -0.3
        # else:
        #     action.joint_positions[6] = 1.7
        
        #print("actions: ", action)
        
        # # help gripper to close 
        # if contact_sensors[0]>0.02 or contact_sensors[1]>0.02:
        #     self._close_gripper = True

        # if self._close_gripper:
        #     self._gripper_counter += 1
        #     if self._gripper_counter > 30:
        #         action.joint_positions[6] = 2.0
        #         pos_gripper = self.robot.get_joint_positions()[6]
        #         print("pose gripper: ", pos_gripper)
        #         if pos_gripper < 0.30:
        #             self._clossing_gripper = True
        #         elif pos_gripper > 0.40:
        #             self._clossing_gripper = False

        #cube distance to target position 
        distance_to_target = np.linalg.norm(self.target_pose_np - cube_position)
        #print("distance to target: ", distance_to_target)

        if distance_to_target > 0.36:
            self.robot.apply_action(action)
        #print("close gripper: ", self._close_gripper)
        
        #self._previous_action = action.joint_positions[:7].copy()

        #print("finger_joint pos: ", self.robot.get_joint_positions()[6])
        #print("actions: ", action)
        #print("previous action: ", self._previous_action)
        #print("dof names: ", self.robot.dof_names)

        self._policy_counter += 1

    def forward_2(self, dt):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt (float) -- Timestep update in the world.

        """

        start_time = time.time()

        obs = self._compute_observation()
        #print("obs: ", obs) # Debugging observation
        self.action = self._compute_action(obs)
        #print("action raw: ", self.action)
        #self.action = np.pad(self.action, (0, len(self.default_pos) - len(self.action)), mode="constant")
        #self.joint_command = self.default_pos + (self.action * self._action_scale)
            #self._previous_action = self.action.copy()
        
        # print("shape self.action: ", self.action.shape)
        # print("type self.action: ", type(self.action))
        # print("type default_pose: ", type(self.default_pos))
        # print("len self.default_pos: ", len(self.default_pos))
        # Pad with zeros
        padded_action = np.pad(self.action, (0, len(self.default_pos) - len(self.action)), mode="constant")

        joint_command = self.default_pos + (padded_action * self._action_scale)

        action = ArticulationAction(joint_positions=joint_command)
        #action.joint_positions = action.joint_positions[:7]
        #action.joint_positions[:7] = [-1.6043e+00,  1.3710e+00, 7.2097e-01, 
        #                               1.8880e+00, -1.1930e+00, -3.0226e+00,  3.0927e-01] 
        self.robot.apply_action(action)

        
        self._previous_action = action.joint_positions[:7].copy()

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        time.sleep(sleep_time)