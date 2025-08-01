from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.physics import ContactSensor

from isaacsim.examples.interactive.brownbot_policy.brownbot import BrownbotPolicy

from pxr import UsdPhysics, PhysxSchema, Gf, UsdGeom
from isaacsim.sensors.physics import _sensor

import omni
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
                scale=np.array([0.0555, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )

        # context = omni.usd.get_context()
        # self._stage = context.get_stage()
        # robot_usd_prims = self._stage.GetPrimAtPath("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/right_inner_pad")
        # UsdPhysics.CollisionAPI.Apply(robot_usd_prims)
        # UsdPhysics.MeshCollisionAPI.Apply(robot_usd_prims)
        # physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(robot_usd_prims)
        # PhysxSchema.PhysxContactReportAPI.Apply(robot_usd_prims)
        
        # world.scene.add( 
        #     ContactSensor(
        #         prim_path="/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/right_inner_pad",
        #         name="Contact_Sensor_RF",
        #         frequency=60,
        #         translation=np.array([0, 0, 0]),
        #         min_threshold=0,
        #         max_threshold=10000000,
        #         radius=-1
        #     )
        # )

        # self.contact_forces_LF = ContactSensor(
        #     prim_path="/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/left_inner_pad",
        #     name="Contact_Sensor_LF",
        #     frequency=60,
        #     translation=np.array([0, 0, 0]),
        #     min_threshold=0,
        #     max_threshold=10000000,
        #     radius=-1
        # )

        # # Add Contact Sensor
        # omni.usd.get_context().open_stage_async("/isaac-sim/workspaces/isaac_sim_scene/brownbot_08.usd")
        # omni.kit.app.get_app().next_update_async()

        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())

        # self.sensor_offsets = [Gf.Vec3d(40, 0, 0), Gf.Vec3d(40, 0, 0), Gf.Vec3d(40, 0, 0), Gf.Vec3d(40, 0, 0)]
        # self.color = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]
        # self.sensorGeoms = []

        # result, sensor = omni.kit.commands.execute(
        #     "IsaacSensorCreateContactSensor",
        #     path="/sensor",
        #     parent="/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/left_inner_pad",
        #     min_threshold=0,
        #     max_threshold=10000000,
        #     color=self.color[0],
        #     radius=0.12,
        #     sensor_period=-1,
        #     translation=self.sensor_offsets[0],
        # )

        self._cs = _sensor.acquire_contact_sensor_interface()

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
        self.brownbot._close_gripper = False
        self.brownbot._clossing_gripper = False
        self.brownbot._gripper_counter = 0
        await self._world.play_async()

    def on_physics_step(self, step_size) -> None:
        cube_position, orientation = self._cube.get_world_pose()
        #print("cube position: ", cube_position)
        #print("cube_position type: ", type(cube_position))

        #value_contact_LF = self.contact_forces_LF.get_current_frame()
        #value_contact_RF = self.contact_forces_RF.get_current_frame()
        #print("value_contact_LF: ", value_contact_LF)
        #print("value_contact_RF: ", value_contact_RF)

        reading_LF = self._cs.get_sensor_reading("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/left_inner_pad" + "/Contact_Sensor")
        reading_RF = self._cs.get_sensor_reading("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/right_inner_pad" + "/Contact_Sensor")
        contact_sensors = [reading_LF.value, reading_RF.value]
        #print("contact_sensors: ", contact_sensors)

        if self._physics_ready:
            self.brownbot.forward(dt=step_size, cube_position=cube_position, 
                                  contact_sensors=contact_sensors, replay_obs=None)
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