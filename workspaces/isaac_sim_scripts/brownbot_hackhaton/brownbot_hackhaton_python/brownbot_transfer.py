from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.physics import ContactSensor

from .brownbot import BrownbotPolicy

from pxr import UsdPhysics, PhysxSchema, Gf, UsdGeom, Sdf, Usd, Gf
from isaacsim.sensors.physics import _sensor
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import XFormPrim, RigidPrim

from scipy.spatial.transform import Rotation as R

import omni.usd
import omni
import numpy as np
import torch
from torch.nn.functional import normalize

# paste the q_corr you computed
#Q_CORR = np.array([9.99999941e-01, 7.13152944e-07, 3.42358582e-04, 1.87531109e-05], dtype=np.float64)  # [x,y,z,w]
Q_CORR = np.array([0.99034696,  0.10453527, -0.04850196,  0.07702491 ], dtype=np.float64)  # [x,y,z,w]  

class BrownbotTransfer(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # TODO verify this information 
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 0.01
        # self._world_settings["rendering_dt"] = 2
        
        #load replay data
        #data = np.load("/isaac-sim/workspaces/isaac_sim_scripts/brownbot_policy/data_temp/trajectory_data.npz")
        #self.replay_obs = data["observations"] 

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
        # Center position
        center = np.array([0.5, 0.0, 0.01])

        # Random offsets
        x_offset = np.random.uniform(-0.1, 0.1)
        y_offset = np.random.uniform(-0.15, 0.15)

        # Final position
        # position = center + np.array([x_offset, y_offset, 16.5190e-02])
        # position_bucket = center + np.array([x_offset, y_offset, 12.5190e-02])
        position = np.array([0.5, 0.0, 0.16])
        position_bucket = np.array([0.5, 0.0, 0.15])

        # world.scene.add(
        #     DynamicCuboid(
        #         prim_path="/World/random_cube",
        #         name="fancy_cube",
        #         position=np.array(position), #[0.5, 0.0, 0.1]
        #         scale=np.array([0.0515, 0.0515, 0.0505]),
        #         color=np.array([0, 0, 1.0]),
        #     )
        # )
        
        #{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd
        #/isaac-sim/workspaces/isaac_sim_scene/revel_assets/revel_assets/dhl_box/dhl_box_flat_test.usd
        add_reference_to_stage(
            usd_path="/isaac-sim/workspaces/isaac_sim_scene/dex_cube_test.usd",
            prim_path="/World/asset_01")
        #world.scene.add(XFormPrim(prim_paths_expr="/World/asset_01", name="dhl_box", positions=np.array([position]) ))
        dhl_box = world.scene.add(RigidPrim(prim_paths_expr="/World/asset_01", 
                                            name="dhl_box", 
                                            positions=np.array([position]),
                                            orientations=np.array([[0, 0, 0, 1]]), # no rotation
                                            scales=np.array([[1.2,1.2,1.2]]),
                                            # Add a placeholder mass and density
                                            masses=np.array([0.5]),
                                            densities=np.array([1000.0]),
                                            # Other physics-related properties
                                            # (These are not in your documentation but are often useful)
                                            linear_velocities=np.array([[0, 0, 0]]),
                                            angular_velocities=np.array([[0, 0, 0]]) 
                                            )
                                )
        
        
        # Define the rotation in Euler angles (roll, pitch, yaw) in degrees
        # Rotation around X is 'roll'
        euler_angles = [0, 0, 90.0] 
        # Create a rotation object from Euler angles
        rotation = R.from_euler('xyz', euler_angles, degrees=True)
        # Convert the rotation to a quaternion (x, y, z, w)
        quaternion = rotation.as_quat()

        # Assuming you have a `position` variable and the `add_reference_to_stage` call
        add_reference_to_stage(
            usd_path="/isaac-sim/workspaces/isaac_sim_scene/plastic_box_7_test_02.usd",
            prim_path="/World/asset_02")
        bucket = world.scene.add(
            RigidPrim(
                prim_paths_expr="/World/asset_02",
                name="bucket",
                positions=np.array([position_bucket]),
                orientations=np.array([quaternion]), # Use the calculated quaternion
                masses=np.array([0.5]),
                densities=np.array([1000.0]),
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

        #self._cs = _sensor.acquire_contact_sensor_interface()

        #self.compute_gripper_info()

        return

    async def setup_post_load(self):
        self._physics_ready = False
        self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        self._world = self.get_world()
        self._cube = self._world.scene.get_object("dhl_box")

        if self._cube is not None:
            _, orientations = self._cube.get_world_poses()
            self._cube_initial_orientation = orientations[0]

        # box_pos, box_quat = self.get_box_position_orientation()

        # #box dimensions
        # L, W, H, t = 0.3974, 0.2968, 0.2574, 0.02
        # wall_centers, wall_sizes, _ = self.compute_walls(
        #     torch.tensor(box_pos).unsqueeze(0), 
        #     torch.tensor(box_quat).unsqueeze(0),
        #     L, W, H, t
        # )  # (1, 4, 3), (1, 4, 3)
        # #print("Wall centers: ", wall_centers)
        # #print("Wall sizes: ", wall_sizes)

        #self.compute_gripper_info()

        pos, quat, finger_pos = self.get_end_effector_pose(
            ee_prim_path="/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/robotiq_base_link",
            ref_prim_path="/World",
            offset=(0.0, 0.0, 0.2134)
        )

        #print("EE pos:", pos)
        #print("EE quat:", quat)

        #self.print_local_axes("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/robotiq_base_link")

        await self.get_world().play_async()

    async def setup_pre_reset(self):
        self.brownbot._close_gripper = False
        self.brownbot._clossing_gripper = False
        self.brownbot._gripper_counter = 0
        return

    async def setup_post_reset(self):
        # This is a function you would override in your extension
        # Add your custom physics callback here
        self.get_world().add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        
        self._physics_ready = False
        self.brownbot._close_gripper = False
        self.brownbot._clossing_gripper = False
        self.brownbot._gripper_counter = 0
        self.brownbot._previous_action = np.zeros(7)
        self.brownbot._policy_counter = 0

        # Generate new random position for the cube
        center = np.array([0.5, 0.0, 0.01])
        x_offset = np.random.uniform(-0.1, 0.1)
        y_offset = np.random.uniform(-0.25, 0.25)
        new_position = center + np.array([x_offset, y_offset, 4.5190e-02])

        # Set the cube's new position (keep orientation unchanged)
        # if self._cube is not None:
        #     self._cube.set_world_poses(positions=np.array([new_position]), 
        #                                orientations=np.array([self._cube_initial_orientation]) )

        await self._world.play_async()

    def on_physics_step(self, step_size) -> None:
        cube_positions, orientations = self._cube.get_world_poses()
        #print("asset orientation: ", orientations)
        # Convert to torch tensor (float32)
        quat = torch.from_numpy(orientations).float()

        # If function expects shape [4], remove the batch dimension
        quat_single = quat.squeeze(0)
        ori_6d = BrownbotTransfer.quat_to_6d(quat_single).detach().cpu().numpy()
        # print("asset ori 6d: ", ori_6d )
        # print("type ori 6d: ", type(ori_6d) )

        box_pos, box_quat = self.get_box_position_orientation()

        #box dimensions
        L, W, H, t = 0.3974, 0.2968, 0.2574, 0.02
        wall_centers, wall_sizes, _ = self.compute_walls(
            torch.tensor(box_pos).unsqueeze(0), 
            torch.tensor(box_quat).unsqueeze(0),
            L, W, H, t
        )  # (1, 4, 3), (1, 4, 3)

        # Convert tensors to numpy and flatten
        wall_centers = wall_centers.detach().cpu().numpy().reshape(-1)  # (12,)
        wall_sizes   = wall_sizes.detach().cpu().numpy().reshape(-1)    # (12,)
        box_quat     = np.array(box_quat, dtype=np.float32).reshape(-1) # (4,)
        # print("wall centers: ", wall_centers)
        # print("wall sizes: ", wall_sizes)
        # print("box quat: ", box_quat)

        box_data = np.concatenate([wall_centers, wall_sizes, box_quat]).astype(np.float32)
        # print("box data: ", box_data)
        # print("type box data: ", type(box_data))

        sizes = self.get_dhl_box_size("/World/asset_01")
        # print("asset sizes: ", sizes)
        # print("type sizes: ", type(sizes))

        #print("cube position: ", cube_position)
        #print("cube_position type: ", type(cube_position))

        #value_contact_LF = self.contact_forces_LF.get_current_frame()
        #value_contact_RF = self.contact_forces_RF.get_current_frame()
        #print("value_contact_LF: ", value_contact_LF)
        #print("value_contact_RF: ", value_contact_RF)

        #reading_LF = self._cs.get_sensor_reading("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/left_inner_pad" + "/Contact_Sensor")
        #reading_RF = self._cs.get_sensor_reading("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/right_inner_pad" + "/Contact_Sensor")
        #contact_sensors = [reading_LF.value, reading_RF.value]
        contact_sensors = [0,0]
        #print("contact_sensors: ", contact_sensors)

        ee_pos, ee_quat, finger_pos = self.get_end_effector_pose(
            ee_prim_path="/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/robotiq_base_link",
            ref_prim_path="/World",
            offset=(0.0, 0.0, 0.2134)
        )

        # print("EE pos:", pos)
        # print("EE quat:", quat)
        # print("finger pos:", finger_pos)
        # print("type finger pos:", type(finger_pos), "type EE quat:", type(quat), "type EE pos:", type(pos))

        # flatten finger_pos (2x3 -> 6)
        finger_pos_flat = finger_pos.reshape(-1)
        # concatenate pos (3), quat (4), finger (6) -> (13,)
        ee_data = np.concatenate([ee_pos, ee_quat, finger_pos_flat]).astype(np.float32)

        #print("ee_data: ", ee_data)


        if self._physics_ready:
            self.brownbot.forward(dt=step_size, cube_position=cube_positions[0], 
                                  contact_sensors=contact_sensors, replay_obs=None,
                                  ori_6d=ori_6d,
                                  sizes=sizes,
                                  box_data=box_data,
                                  ee_data=ee_data,
                                  )
            #print("physics step ready")
            c=2
        else:
            self._physics_ready = True
            self.brownbot.initialize()
            #print("physics step initialization")
            #self.brownbot.post_reset()
            #self.brownbot.robot.set_joints_default_state(self.brownbot.default_pos)

            # # Let the simulation settle before running
            # for _ in range(100):  # You can increase this number if needed
            #     self._world.step(render=False)

    def world_cleanup(self):
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")

    def get_box_position_orientation(self):
        # Assuming this is your object from the scene
        bucket_object = self._world.scene.get_object("bucket")

        if bucket_object is not None:
            # 1. Get the path of the parent prim
            # This is a method on the core object wrapper
            prim_paths = bucket_object.prim_paths
            #print("Prim paths: ", prim_paths)
            
            if prim_paths:
                parent_prim_path = prim_paths[0]
                
                # 2. Get the prim object from its path
                stage = omni.usd.get_context().get_stage()
                parent_prim = stage.GetPrimAtPath(parent_prim_path)

                if parent_prim:
                    # 3. Get the Xformable API for the prim
                    xform_api = UsdGeom.Xformable(parent_prim)
                    
                    # 4. Compute the prim's transformation matrix in world space
                    # This is the most reliable way to get the global pose
                    world_matrix = xform_api.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    
                    # 5. Extract the position and orientation from the matrix
                    position = world_matrix.ExtractTranslation()
                    rotation = world_matrix.ExtractRotation().GetQuaternion()
                    
                    #print(f"Global position of '{parent_prim_path}':")
                    #print(f"Position (x, y, z): {position}")
                    #print(f"Orientation (w, x, y, z): {rotation}")

                    # Convert to numpy arrays
                    pos = np.array([position[0], position[1], position[2]], dtype=np.float32)
                    quat = np.array([
                        rotation.GetReal(), 
                        rotation.GetImaginary()[0], 
                        rotation.GetImaginary()[1], 
                        rotation.GetImaginary()[2]
                    ], dtype=np.float32)


                    return (pos, quat)

                    # # You can also get the child prims' global pose in the same way
                    # child_prim_name = "plastic_pallet"
                    # child_prim_path = Sdf.Path(parent_prim_path).AppendPath(child_prim_name)
                    
                    # child_prim = stage.GetPrimAtPath(child_prim_path)
                    # if child_prim:
                    #     child_xform_api = UsdGeom.Xformable(child_prim)
                    #     child_world_matrix = child_xform_api.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    #     child_position = child_world_matrix.ExtractTranslation()
                    #     child_rotation = child_world_matrix.ExtractRotation().GetQuaternion()

                    #     print(f"\nGlobal position of child prim '{child_prim_path}':")
                    #     print(f"Position (x, y, z): {child_position}")
                    #     print(f"Orientation (w, x, y, z): {child_rotation}")
                        
                    #     # Convert to numpy arrays
                    #     pos = np.array([child_position[0], child_position[1], child_position[2]], dtype=np.float32)
                    #     quat = np.array([
                    #         child_rotation.GetReal(), 
                    #         child_rotation.GetImaginary()[0], 
                    #         child_rotation.GetImaginary()[1], 
                    #         child_rotation.GetImaginary()[2]
                    #     ], dtype=np.float32)


                    #     return (pos, quat)
                else:
                    print(f"Error: Prim at '{parent_prim_path}' not found.")
            else:
                print("No prim paths found in the object.")

    @staticmethod
    def quat_to_rot_matrix(quat: torch.Tensor) -> torch.Tensor:
        """Convert (N, 4) quaternion to (N, 3, 3) rotation matrices."""
        q = normalize(quat, dim=-1)  # (N, 4)
        w, x, y, z = q.unbind(-1)
        B = quat.shape[0]
        R = torch.zeros(B, 3, 3, device=quat.device)

        R[:, 0, 0] = 1 - 2 * (y**2 + z**2)
        R[:, 0, 1] = 2 * (x*y - z*w)
        R[:, 0, 2] = 2 * (x*z + y*w)

        R[:, 1, 0] = 2 * (x*y + z*w)
        R[:, 1, 1] = 1 - 2 * (x**2 + z**2)
        R[:, 1, 2] = 2 * (y*z - x*w)

        R[:, 2, 0] = 2 * (x*z - y*w)
        R[:, 2, 1] = 2 * (y*z + x*w)
        R[:, 2, 2] = 1 - 2 * (x**2 + y**2)
        return R

    @staticmethod
    def compute_walls(box_pos, box_rot, L, W, H, t):
        """Return world centers & sizes of the 4 walls for each env."""
        N = box_pos.shape[0]
        R = BrownbotTransfer.quat_to_rot_matrix(box_rot)  # (N, 3, 3)

        # local_centers = torch.tensor([
        #     [0,  W/2 - t/2, H/2],   # front
        #     [0, -W/2 + t/2, H/2],   # back
        #     [-L/2 + t/2, 0, H/2],   # left
        #     [ L/2 - t/2, 0, H/2],   # right
        # ], device=box_pos.device)  # (4, 3)

        # sizes = torch.tensor([
        #     [L, t, H],
        #     [L, t, H],
        #     [t, W, H],
        #     [t, W, H],
        # ], device=box_pos.device)  # (4, 3)

        # Adapted for asset where X=W, Y=H, Z=L
        local_centers = torch.tensor([
            [ 0.0,        H/2.0,  L/2.0 - t/2.0],   # front  (+Z)
            [ 0.0,        H/2.0, -L/2.0 + t/2.0],   # back   (-Z)
            [-W/2.0 + t/2.0, H/2.0,  0.0      ],   # left   (-X)
            [ W/2.0 - t/2.0, H/2.0,  0.0      ],   # right  (+X)
        ], device=box_pos.device)  # (4,3)

        # sizes: (size_x, size_y, size_z) per wall in local coords
        sizes = torch.tensor([
            [ W, H,  t],    # front  : spans width (X), height (Y), thin in Z
            [ W, H,  t],    # back
            [ t, H,  L],    # left   : thin in X, spans height and length
            [ t, H,  L],    # right
        ], device=box_pos.device)  # (4,3)

        # Broadcast local_centers across batch
        local_centers = local_centers.unsqueeze(0).expand(N, -1, -1)  # (N, 4, 3)
        sizes = sizes.unsqueeze(0).expand(N, -1, -1)                  # (N, 4, 3)

        # Transform to world
        world_centers = torch.bmm(local_centers, R.transpose(1, 2)) + box_pos.unsqueeze(1)  # (N, 4, 3)

        return world_centers, sizes, box_rot
    
    # def compute_gripper_info(self):
    #     stage = omni.usd.get_context().get_stage()
    #     # ee_base_prim = stage.GetPrimAtPath("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/robotiq_base_link")
    #     # ee_base_xform = UsdGeom.Xformable(ee_base_prim)

    #     # # World transform of the base prim
    #     # world_matrix = ee_base_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    #     # Gripper base prim
    #     ee_base_prim = stage.GetPrimAtPath("/World/Brownbot/ur5/Robotiq_2F_140_physics_edit/robotiq_base_link")
    #     ee_base_xform = UsdGeom.Xformable(ee_base_prim)
    #     ee_world_matrix = ee_base_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        
    #     # Robot base_link prim (your chosen reference frame)
    #     base_link_prim = stage.GetPrimAtPath("/World/Brownbot/ur5/base_link")
    #     base_link_xform = UsdGeom.Xformable(base_link_prim)
    #     base_link_world_matrix = base_link_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    #     # Transform of EE relative to base_link
    #     #rel_matrix = base_link_world_matrix.GetInverse().matrix * ee_world_matrix.matrix
    #     #rel_matrix = Gf.Matrix4d(rel_matrix)  # make sure it's a USD matrix type

    #     rel_matrix = base_link_world_matrix.GetInverse() * ee_world_matrix
        
    #     # Local offset (in the gripper base frame)
    #     offset = Gf.Vec3d(0.0, 0.0, 0.2134)

    #     # Extract rotation from the relative matrix
    #     rot = rel_matrix.ExtractRotation().GetQuat()
    #     rot_np = np.array([
    #         rot.GetImaginary()[0],
    #         rot.GetImaginary()[1],
    #         rot.GetImaginary()[2],
    #         rot.GetReal()
    #     ], dtype=np.float32)

    #     rot_mat = R.from_quat(rot_np).as_matrix()

    #     # Apply offset in base_link frame
    #     pos_base = rel_matrix.ExtractTranslation()
    #     pos_with_offset = np.array([pos_base[0], pos_base[1], pos_base[2]], dtype=np.float32) + rot_mat @ np.array(offset)

    #     print("EE pos (with offset, in base_link):", pos_with_offset)
    #     print("EE quat (in base_link):", rot_np)
        
        
    #     # # Extract translation + rotation
    #     # base_pos = world_matrix.ExtractTranslation()
    #     # base_quat = world_matrix.ExtractRotation().GetQuat()

    #     # pos_base = np.array([base_pos[0], base_pos[1], base_pos[2]], dtype=np.float32)
    #     # quat_base = np.array([
    #     #     base_quat.GetImaginary()[0],
    #     #     base_quat.GetImaginary()[1],
    #     #     base_quat.GetImaginary()[2],
    #     #     base_quat.GetReal()
    #     # ], dtype=np.float32)   # (x, y, z, w)

    #     # print("Gripper base link: ", pos_base)
    #     # print("Gripper base link orientation (x,y,z,w): ", quat_base)

    #     # # Convert quat to rotation matrix
    #     # rot_base = R.from_quat(quat_base).as_matrix()

    #     # # Apply offset (local → world)
    #     # offset = np.array([0.0, 0.0, 0.2134], dtype=np.float32)
    #     # pos_ee = pos_base + rot_base @ offset
    #     # quat_ee = quat_base  # (no extra orientation offset here, but you can apply one if needed)

    #     # print("Gripper position: ", pos_ee)
    #     # print("Gripper orientation (x,y,z,w): ", quat_ee)

    @staticmethod
    def quat_multiply(q1, q2):
        # q = q1 * q2, both in [x,y,z,w]
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ], dtype=np.float32)
    
    def get_end_effector_pose(self, ee_prim_path, ref_prim_path="/World", offset=(0.0, 0.0, 0.0)):
        """
        Compute the pose of an end-effector in Isaac Sim relative to a reference prim.

        Args:
            ee_prim_path (str): Prim path of the end-effector base (e.g. robotiq_base_link).
            ref_prim_path (str): Prim path of the reference frame (e.g. /World or ur5/base_link).
            offset (tuple/list): Local offset (x,y,z) applied in ee frame.

        Returns:
            pos (np.ndarray): (3,) position of EE in reference frame.
            quat (np.ndarray): (4,) quaternion (x,y,z,w) of EE in reference frame.
        """
        stage = omni.usd.get_context().get_stage()

        # Get end-effector prim world transform
        ee_prim = stage.GetPrimAtPath(ee_prim_path)
        ee_xform = UsdGeom.Xformable(ee_prim)
        ee_world = ee_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        # Get reference prim world transform
        ref_prim = stage.GetPrimAtPath(ref_prim_path)
        ref_xform = UsdGeom.Xformable(ref_prim)
        ref_world = ref_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        # Relative transform: ref^-1 * ee
        rel_matrix = ref_world.GetInverse() * ee_world

        # Extract rotation
        rot = rel_matrix.ExtractRotation().GetQuat()
        quat = np.array([
            rot.GetImaginary()[0],
            rot.GetImaginary()[1],
            rot.GetImaginary()[2],
            rot.GetReal()
        ], dtype=np.float32)

        # Apply offset (local to ee frame)
        rot_mat = R.from_quat(quat).as_matrix()
        pos_base = rel_matrix.ExtractTranslation()
        pos = np.array([pos_base[0], pos_base[1], pos_base[2]], dtype=np.float32) + rot_mat @ np.array(offset, dtype=np.float32)

        #print("pos base:", pos_base)
        #print("raw quat: ", quat)

        #q_flip_x = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # [x, y, z, w]
        #Q_CORR = np.array([-0.99499444, -0.07040164, 0.07074066, -0.00504281], dtype=np.float32)
        #quat_corrected = BrownbotTransfer.quat_multiply(quat, q_flip_x)

        #q_flip_z = np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32)  # [x, y, z, w]
        #quat_corrected = BrownbotTransfer.quat_multiply(quat, q_flip_z)

        # # Sim basis (measured)
        # sim_axes = np.array([
        #     [-1.4391463e-06,  1.0000000e+00,  3.7505732e-05],  # X
        #     [ 9.9999976e-01,  1.4134653e-06,  6.8471715e-04],  # Y
        #     [ 6.8471709e-04,  3.7506707e-05, -9.9999976e-01],  # Z
        # ]).T  # shape (3,3), each column is axis

        # # Lab’s expected basis (canonical frame: X=[1,0,0], Y=[0,1,0], Z=[0,0,1])
        # lab_axes = np.eye(3)

        # # Correction = Lab * Sim^-1
        # R_corr = lab_axes @ np.linalg.inv(sim_axes)

        # # Quaternion [x,y,z,w]
        # q_corr = R.from_matrix(R_corr).as_quat()
        # print("Correction quaternion (x,y,z,w):", q_corr)
        # print("original quat: ", quat)

        # quat_corrected = BrownbotTransfer.quat_multiply(q_corr, quat)
        quat_corrected = BrownbotTransfer.apply_qcorr_to_sim_quat(quat)

        # ---------------------------------------------
        # Add gripper finger positions
        # ---------------------------------------------
        finger_sep = 0.14 / 2.0   # half of gripper width
        finger_offsets = np.array([
            [0.0,  finger_sep, -0.03],   # right finger center
            [0.0, -finger_sep, -0.03],   # left finger center
        ], dtype=np.float32)  # shape (2,3)

        # Transform finger offsets into world space
        # Apply offset (local to ee frame)
        #rot_mat = R.from_quat(quat_corrected).as_matrix()
        finger_pos = (rot_mat @ finger_offsets.T).T + pos  # shape (2,3)
        #print("Finger positions:", finger_pos)

    
        return pos, quat_corrected, finger_pos
    
    @staticmethod
    def print_local_axes(prim_path):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(prim)
        
        # Get local transform matrix (Gf.Matrix4d)
        local_matrix = xform.GetLocalTransformation()
        
        # Convert to numpy 3x3 rotation
        rot3x3 = np.array([
            [local_matrix[0][0], local_matrix[0][1], local_matrix[0][2]],
            [local_matrix[1][0], local_matrix[1][1], local_matrix[1][2]],
            [local_matrix[2][0], local_matrix[2][1], local_matrix[2][2]],
        ], dtype=np.float32)
        
        # Extract axes (columns of rotation matrix)
        x_axis = rot3x3[:, 0]
        y_axis = rot3x3[:, 1]
        z_axis = rot3x3[:, 2]
        
        print(f"Local axes for {prim_path}:")
        print(f"  X axis: {x_axis}")
        print(f"  Y axis: {y_axis}")
        print(f"  Z axis: {z_axis}")
        return x_axis, y_axis, z_axis

    @staticmethod
    def apply_qcorr_to_sim_quat(q_sim):
        # q_sim: numpy array [x,y,z,w]
        q_sim = q_sim.astype(np.float64)
        q_sim /= np.linalg.norm(q_sim)
        q_al = BrownbotTransfer.quat_multiply(Q_CORR, q_sim)   # left-multiply: q_al = q_corr * q_sim
        q_al /= np.linalg.norm(q_al)
        return q_al
    
    @staticmethod
    def quat_to_6d(quat: torch.Tensor) -> torch.Tensor:
        """
        Convert quaternion (x, y, z, w) into a 6D rotation representation
        (first two columns of the rotation matrix).
        
        Args:
            quat: (..., 4) tensor of quaternions
        Returns:
            (..., 6) tensor
        """
        x, y, z, w = quat.unbind(-1)

        # rotation matrix elements
        R = torch.stack([
            1 - 2 * (y*y + z*z),   2 * (x*y - z*w),     2 * (x*z + y*w),
            2 * (x*y + z*w),       1 - 2 * (x*x + z*z), 2 * (y*z - x*w),
            2 * (x*z - y*w),       2 * (y*z + x*w),     1 - 2 * (x*x + y*y)
        ], dim=-1).reshape(*quat.shape[:-1], 3, 3)

        # take first two columns (6D rep)
        r1 = normalize(R[..., 0], dim=-1)
        r2 = normalize(R[..., 1], dim=-1)

        return torch.cat([r1, r2], dim=-1)
    
    def get_dhl_box_size(self, prim_path="/World/asset_01", gripper_span=0.14):
        """Get normalized DHL box size (relative to gripper span)."""

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        # Create a BBoxCache at default time and compute LOCAL bound
        bbox_cache = UsdGeom.BBoxCache(0, ["default"])
        bbox = bbox_cache.ComputeLocalBound(prim)

        # Extract min/max corners
        min_pt = bbox.GetRange().GetMin()
        max_pt = bbox.GetRange().GetMax()

        # Raw size (width, depth, height in meters)
        raw_sizes = np.array([
            max_pt[0] - min_pt[0],
            max_pt[1] - min_pt[1],
            max_pt[2] - min_pt[2],
        ])

        # Normalize by gripper span and clamp to [0, 1]
        norm_sizes = np.clip(raw_sizes / gripper_span, 0.0, 1.0)

        #print(f"DHL box raw size (m): {raw_sizes}")
        #print(f"DHL box normalized size: {norm_sizes}")

        return norm_sizes