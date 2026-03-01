import omni
import omni.kit.app
from isaacsim.sensors.rtx import LidarRtx, get_gmo_data
from isaacsim.core.utils.stage import open_stage, is_stage_loading, add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from isaacsim.core.api import World
from omni.isaac.core.utils.xforms import reset_and_set_xform_ops
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
# TODO REVIEW: new import for ros2 publisher 
import omni.replicator.core as rep
from pxr import Gf

import omni.graph.core as og
# imports for differential controller
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController 
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.core.prims import Articulation 
from isaacsim.core.utils.types import ArticulationAction
import time
import numpy as np 
enable_extension("isaacsim.ros2.bridge")
from isaacsim.sensors.rtx import LidarRtx
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
import rclpy


class Tankbot():
    USD_PATH = "/home/harry/dev/nav_isaacsim/vehicle/turtlebot_3_flattened.usd"
    WHEEL_RADIUS = 0.033
    WHEEL_BASE = 0.287
    def __init__(self, world_path, name) -> None:
        self._world_path = None
        self._robot = None
        self._controller = None
        self._articulation_crtl = None 
        self._lidar = None
        self.spawn_and_return_model(world_path, name)
        # self.ros2_pointcloud_publisher() 
        self.init_diff_controller("diff_ctrler")
        self.cmdvel_sub_ctrl()
        
    def update(self):
        linear_vel = og.Controller.attribute(f"{self._world_path}/CmdVelSub/SubscribeCmdVel.outputs:linearVelocity").get()[0]
        angular_vel = og.Controller.attribute(f"{self._world_path}/CmdVelSub/SubscribeCmdVel.outputs:angularVelocity").get()[2]
        # print(
        #     f"-------------------------------\n Linear vel: {linear_vel}\n Angular vel: {angular_vel}"
        # )
        self.ctrl_cmd(linear_vel, angular_vel)
    
    def spawn_and_return_model(self, world_path:str, _name: str, ) -> None:
        self._world_path = world_path
        self._robot = WheeledRobot(
            prim_path = self._world_path,
            name = _name,
            wheel_dof_names=[f"wheel_left_joint", "wheel_right_joint"],
            create_robot = True,
            usd_path = self.USD_PATH
        )
        
    def init_diff_controller(self, _name:str) -> None:
        self._controller = DifferentialController(
            name = _name,
            wheel_radius = self.WHEEL_RADIUS,
            wheel_base = self.WHEEL_BASE
        )
        self._articulation_ctrl = ArticulationController()
        articulation_view = Articulation(prim_paths_expr=self._world_path, name="view")
        self._articulation_ctrl.initialize(articulation_view)
        
    def ctrl_cmd(self,lin_vel, ang_vel):
        joint_vels = self._controller.forward(command=[lin_vel,ang_vel]).get_dict()["joint_velocities"]
        self._articulation_ctrl.apply_action(
            ArticulationAction(
                joint_velocities=joint_vels
            )
        )
        
    def ros2_pointcloud_publisher(self):
        sensor = LidarRtx(
            prim_path=self._world_path+"/base_scan/RPLIDAR_S2E/RPLidar_S2E"    
        )
        sensor.initialize()
        sensor.attach_annotator("IsaacCreateRTXLidarScanBuffer")
        hydra_texture = rep.create.render_product(self._world_path+"/base_scan/RPLIDAR_S2E/RPLidar_S2E", [1,1], name="_")
        writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize(topicName="point_cloud", frameId="lidar_scan")
        writer.attach([hydra_texture])

        
    
    def cmdvel_sub_ctrl(self):
        og.Controller.edit(
            {"graph_path": f"{self._world_path}/CmdVelSub", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("Tick", "omni.graph.action.OnTick"),
                    ("SubscribeCmdVel", "isaacsim.ros2.bridge.ROS2SubscribeTwist")
                ],
                og.Controller.Keys.CONNECT: [
                    ("Tick.outputs:tick", "SubscribeCmdVel.inputs:execIn")
                ]
            }
        )
