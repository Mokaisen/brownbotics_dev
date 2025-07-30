from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid

from isaacsim.examples.interactive.brownbot_policy.brownbot import BrownbotPolicy

import numpy as np


class BrownbotTransfer(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # TODO verify this information 
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 0.01
        # self._world_settings["rendering_dt"] = 2
        
        #load replay data
        data = np.load("/isaac-sim/workspaces/isaac_sim_scripts/brownbot_policy/data_temp/trajectory_data.npz")
        self.replay_obs = data["observations"] 

    def setup_scene(self):
        # add the robot, the object, and the ground floor 
        world = self.get_world()
        world.scene.add_default_ground_plane()
        
        self.brownbot = BrownbotPolicy(
            prim_path="/World/Brownbot",
            name="Brownbot",
            position=np.array([0,0,0.2]),
        )

        # add a cube for brownbot to pick up
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([5.2191e-01, 0.0915e-01, 4.5190e-02]), #[0.5, 0.0, 0.1]
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )

        return

    async def setup_post_load(self):
        self._physics_ready = False
        self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        self._world = self.get_world()
        self._cube = self._world.scene.get_object("fancy_cube")
        await self.get_world().play_async()

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        self._physics_ready = False
        await self._world.play_async()

    def on_physics_step(self, step_size) -> None:
        cube_position, orientation = self._cube.get_world_pose()
        #print("cube position: ", cube_position)
        #print("cube_position type: ", type(cube_position))

        if self._physics_ready:
            self.brownbot.forward(dt=step_size, cube_position=cube_position, replay_obs=None)
            #print("physics step")
        else:
            self._physics_ready = True
            self.brownbot.initialize()
            #self.brownbot.post_reset()
            #self.brownbot.robot.set_joints_default_state(self.brownbot.default_pos)

            # # Let the simulation settle before running
            # for _ in range(100):  # You can increase this number if needed
            #     self._world.step(render=False)

    def world_cleanup(self):
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")