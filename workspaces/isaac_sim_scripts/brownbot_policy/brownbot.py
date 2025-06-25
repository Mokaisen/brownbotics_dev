from typing import Optional

import numpy as np
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.core.utils.types import ArticulationAction

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
            "/isaac-sim/workspaces/brownbot_rl/logs/skrl/brownbot_lift/2025-06-13_22-21-29_ppo_torch_rewardsToOne/checkpoints/agent_72000.pt",
            "/isaac-sim/workspaces/brownbot_rl/logs/skrl/brownbot_lift/2025-06-13_22-21-29_ppo_torch_rewardsToOne/params/env.yaml",
        )
        self._action_scale = 0.2
        self._previous_action = np.zeros(7)
        self._policy_counter = 0

    def _compute_observation(self):
        """
        Compute the observation vector for the policy

        Argument:

        Returns:
        np.ndarray -- The observation vector.

        """

        obs = np.zeros(39)

        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[:14] = current_joint_pos - self.default_pos
        obs[14:28] = current_joint_vel

        # object position in robot root frame
        # TODO how to access this position? 
        obs[28:31] = [0,0,0]

        # Object target pose
        # TODO how to access the mdp.generated_commands
        obs[31:38] = [0,0,0,0,0,0,0]

        # previous action
        obs[38:45] = self._previous_action

        return obs

    def forward(self, dt):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt (float) -- Timestep update in the world.

        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation()
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        action = ArticulationAction(joint_positions=self.default_pos + (self.action * self._action_scale))
        self.robot.apply_action(action)

        self._policy_counter += 1