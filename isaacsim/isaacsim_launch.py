# ------------------------ CONFIGS ------------------------ 
from omni.isaac.kit import SimulationApp

CONFIG = {
    # Isaac Sim launch configs
    # https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.simulation_app/docs/index.html#isaacsim.simulation_app.SimulationApp.DEFAULT_LAUNCHER_CONFIG 
    "headless": False, # Show GUI
    "enable_motion_bvh": True # For RTX Lidar point cloud compute
}
simulation_app = SimulationApp(launch_config = CONFIG)

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
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
from tankbot import Tankbot

# from vehicle.ros_cmd_vel_sub import CmdVelSub

# imports for differential controller
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController 
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.core.prims import Articulation 
from isaacsim.core.utils.types import ArticulationAction





import time
import numpy as np 
import omni.graph.core as og


class LaunchSim(): 
    ENV_URL = "/home/harry/dev/slam_isaacsim/world/simple_obstacles.usd"
    
    def __init__(self):
        self._world = None
        self._tankbot = None
        
        
    def start(self):
        self._world = World(stage_units_in_meters=1.0)
        add_reference_to_stage(
            usd_path = self.ENV_URL, 
            prim_path = "/World/room"
        )
        # self._world.scene.add_default_ground_plane()
        while is_stage_loading():
            simulation_app.update()
        
        self._tankbot = Tankbot("/World/tankbot", "Tankbot")
        

        self._world.reset()
        timeline = omni.timeline.get_timeline_interface()
        og.Controller.edit(
            {"graph_path": f"/World/TimePublisher", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("Tick", "omni.graph.action.OnTick"),
                    ("Ros2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"), 
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock")                    
                ],
                og.Controller.Keys.CONNECT: [
                    ("Tick.outputs:tick", "PublishClock.inputs:execIn"), 
                    ("Ros2Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp")
                ]
            }
        )
        
        i = 0
        reset_needed=False
        # timeline.play()
        timeline.stop()
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped() and not reset_needed:
                reset_needed = True
            if self._world.is_playing():
                if reset_needed:
                    self._world.reset()
                    reset_needed = False
                else:
                    self._tankbot.update()
            i+=1
        timeline.stop()
        simulation_app.close()
        

if __name__ == "__main__":
    sim = LaunchSim()
    sim.start() 
