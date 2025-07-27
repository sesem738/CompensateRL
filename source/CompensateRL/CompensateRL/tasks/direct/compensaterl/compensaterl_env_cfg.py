# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_assets import FRANKA_PANDA_CFG

@configclass
class CompensaterlEnvCfg(DirectRLEnvCfg):
    # env
    decimation = 2
    episode_length_s = 8
    action_space = 7
    observation_space = 7 + 6
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1/120,
        render_interval=decimation,
        disable_contact_processing=True,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1024, env_spacing=1.5, replicate_physics=True)

    # robot
    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # ground
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitStateCfg(
            position=(0.0, 0.0, 0.0)),
    )

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/ThorlabsTable/table_instanceable.usd"),
            init_state=AssetBaseCfg.InitStateCfg(
                position=(0.0, 0.0, 0.0),
                rot=(0.70711, 0.0, 0.0, 0.70711)),
        )
    
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0),
    )