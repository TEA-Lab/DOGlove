from __future__ import annotations

import argparse
import faulthandler
import platform
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import mink
import mujoco
import mujoco.viewer
import numpy as np
from udp_receiver import UDPReceiver
from loop_rate_limiters import RateLimiter


CANONICAL_TIPS: Tuple[str, ...] = (
    "index_tip",
    "middle_tip",
    "ring_tip",
    "little_tip",
    "thumb_tip",
)

_FK_XML = Path(__file__).parent / "DOGlove_meshes" / "DOGlove-v3.xml"
_HAND_MESHES_ROOT = Path(__file__).parent / "hand_meshes"


def _hand_scene_xml(*parts: str) -> str:
    return (_HAND_MESHES_ROOT.joinpath(*parts)).as_posix()

FK_SITE_FOR_CANONICAL_TIP: Dict[str, str] = {
    "index_tip": "index_tip_site",
    "middle_tip": "middle_tip_site",
    "ring_tip": "ring_tip_site",
    "little_tip": "pinky_tip_site",
    "thumb_tip": "thumb_tip_site",
}

FK_JOINT_NAMES: Tuple[str, ...] = (
    "thumb_bend_1",
    "thumb_bend_2",
    "thumb_split",
    "thumb_mcp",
    "index_bend_1",
    "index_bend_2",
    "index_split",
    "middle_bend_1",
    "middle_bend_2",
    "middle_split",
    "ring_bend_1",
    "ring_bend_2",
    "ring_split",
    "pinky_bend_1",
    "pinky_bend_2",
    "pinky_split",
    "thumb_bend_3",
    "index_bend_3",
    "middle_bend_3",
    "ring_bend_3",
    "pinky_bend_3",
)


@dataclass(frozen=True)
class ModelSpec:
    name: str
    ik_xml: str
    offset: Sequence[float]
    scale: float
    viewer_supports_dual: bool
    tip_site_aliases: Dict[str, Optional[str]]
    tip_target_aliases: Dict[str, Optional[str]]
    tip_position_adjustments: Dict[str, Sequence[float]]


class ViewerMode(str, Enum):
    AUTO = "auto"
    SINGLE = "single"
    DUAL = "dual"
    HEADLESS = "headless"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="FK/IK runtime for hand models with UDP glove input."
    )
    parser.add_argument(
        "--model",
        type=str,
        default="leaphand",
        choices=["inspirehand", "leaphand", "shadowhand", "allegro"],
        help="Hand model to use.",
    )
    parser.add_argument(
        "--viewer",
        type=str,
        default=ViewerMode.AUTO.value,
        choices=[mode.value for mode in ViewerMode],
        help="Viewer mode: auto, single, dual, or headless.",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Loop frequency.")
    parser.add_argument(
        "--log-env",
        action="store_true",
        help="Print runtime diagnostics (platform/version) on startup.",
    )
    return parser.parse_args(argv)


class FkIkEngine:
    def __init__(self, model_spec: ModelSpec, fps: float) -> None:
        self.model_spec = model_spec
        self.fps = fps
        self.fk_model = None
        self.fk_data = None
        self.ik_model = None
        self.configuration = None
        self.ik_data = None
        self.posture_task = None
        self.tasks = []
        self.tip_task_specs: List[Tuple[str, object]] = []
        self.tip_target_ids: Dict[str, int] = {}
        self.fk_joint_ids: List[int] = []
        self._fk_site_ids: Dict[str, int] = {}
        self._rate = RateLimiter(frequency=self.fps)

    def load_models(self) -> None:
        self.fk_model = mujoco.MjModel.from_xml_path(_FK_XML.as_posix())
        self.fk_data = mujoco.MjData(self.fk_model)
        self.ik_model = mujoco.MjModel.from_xml_path(self.model_spec.ik_xml)
        self.configuration = mink.Configuration(self.ik_model)
        self.ik_data = self.configuration.data

    def build_tasks(self) -> None:
        self.posture_task = mink.PostureTask(self.ik_model, cost=1e-2)
        self.posture_task.set_target_from_configuration(self.configuration)
        self.tip_task_specs = []
        for tip_name in CANONICAL_TIPS:
            site_name = self.model_spec.tip_site_aliases.get(tip_name)
            if site_name is None:
                continue
            task = mink.FrameTask(
                frame_name=site_name,
                frame_type="site",
                position_cost=1.0,
                orientation_cost=0.0,
                lm_damping=1.0,
            )
            self.tip_task_specs.append((tip_name, task))
        self.tasks = [self.posture_task, *[task for _, task in self.tip_task_specs]]

        for tip_name, site_name in self.model_spec.tip_site_aliases.items():
            target_name = self.model_spec.tip_target_aliases.get(tip_name)
            if site_name is None or target_name is None:
                continue
            mink.move_mocap_to_frame(
                self.ik_model, self.ik_data, target_name, site_name, "site"
            )
            target_body = self.ik_model.body(target_name)
            if target_body.mocapid[0] >= 0:
                self.tip_target_ids[tip_name] = target_body.mocapid[0]

    def build_fk_indexes(self) -> None:
        self.fk_joint_ids = []
        for joint_name in FK_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                self.fk_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            self.fk_joint_ids.append(joint_id)

        for tip_name, site_name in FK_SITE_FOR_CANONICAL_TIP.items():
            site_id = mujoco.mj_name2id(
                self.fk_model, mujoco.mjtObj.mjOBJ_SITE, site_name
            )
            self._fk_site_ids[tip_name] = site_id

    def apply_glove_joints(self, joints: Optional[Sequence[float]]) -> None:
        if joints is None:
            return
        for i, joint_id in enumerate(self.fk_joint_ids):
            if i >= len(joints):
                break
            self.fk_data.ctrl[joint_id] = joints[i]
        mujoco.mj_forward(self.fk_model, self.fk_data)

    def _compute_target_positions(self) -> Dict[str, np.ndarray]:
        """
        Application note for new hand models:

        `model_spec.scale` is a uniform FK->IK scale factor applied to all tips.
        Tune this first for each model:
        1. Set `offset=(0,0,0)` and all per-tip adjustments to zero.
        2. Move one finger through a wide range.
        3. Increase scale if IK motion amplitude is too small, decrease if too large.

        Only after scale is close should you tune global/per-tip translations.
        """
        targets: Dict[str, np.ndarray] = {}
        for tip_name, site_id in self._fk_site_ids.items():
            site_pos = self.fk_data.site_xpos[site_id]
            scaled = np.asarray(site_pos) * float(self.model_spec.scale)
            targets[tip_name] = scaled
        return targets

    def _update_ik_mocap_targets(self, target_positions: Dict[str, np.ndarray]) -> None:
        """
        Application note for alignment tuning:

        After `scale` is set, tune translation terms in this order:
        1. `model_spec.offset` for global XYZ alignment of all fingertips.
        2. `model_spec.tip_position_adjustments[tip]` for residual per-tip errors.

        `offset` should fix common drift across all fingers. Per-tip adjustments
        should stay small and only correct model-specific fingertip bias.
        """
        ox, oy, oz = (
            float(self.model_spec.offset[0]),
            float(self.model_spec.offset[1]),
            float(self.model_spec.offset[2]),
        )
        for tip_name, mocap_id in self.tip_target_ids.items():
            tip_position = target_positions.get(tip_name)
            if tip_position is None:
                continue
            adj = self.model_spec.tip_position_adjustments.get(tip_name, (0.0, 0.0, 0.0))
            self.ik_data.mocap_pos[mocap_id] = np.array(
                [
                    -tip_position[0] + ox + float(adj[0]),
                    -tip_position[1] + oy + float(adj[1]),
                    tip_position[2] + oz + float(adj[2]),
                ]
            )

    def step(self, joints: Optional[Sequence[float]]) -> None:
        self.apply_glove_joints(joints)
        mujoco.mj_step(self.fk_model, self.fk_data)
        target_positions = self._compute_target_positions()
        for tip_name, task in self.tip_task_specs:
            target_name = self.model_spec.tip_target_aliases[tip_name]
            task.set_target(
                mink.SE3.from_mocap_name(self.ik_model, self.ik_data, target_name)
            )
        self._update_ik_mocap_targets(target_positions)
        vel = mink.solve_ik(
            self.configuration, self.tasks, self._rate.dt, "quadprog", 1e-5
        )
        self.configuration.integrate_inplace(vel, self._rate.dt)
        mujoco.mj_camlight(self.ik_model, self.ik_data)

    def sleep(self) -> None:
        self._rate.sleep()


class FkIkApp:
    def __init__(self, model_spec: ModelSpec, viewer_mode: ViewerMode, fps: float):
        self.model_spec = model_spec
        self.viewer_mode = viewer_mode
        self.engine = FkIkEngine(model_spec=model_spec, fps=fps)
        self.receiver = UDPReceiver()

    def _effective_viewer_mode(self) -> ViewerMode:
        if self.viewer_mode != ViewerMode.AUTO:
            return self.viewer_mode
        if platform.system().lower() == "darwin":
            return ViewerMode.SINGLE
        if not self.model_spec.viewer_supports_dual:
            return ViewerMode.SINGLE
        return ViewerMode.DUAL

    def _run_headless(self) -> None:
        while True:
            joints = self.receiver.get_most_recent_joints()
            self.engine.step(joints)
            self.engine.sleep()

    def _run_single_viewer(self) -> None:
        with mujoco.viewer.launch_passive(
            model=self.engine.ik_model,
            data=self.engine.ik_data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            mujoco.mjv_defaultFreeCamera(self.engine.ik_model, viewer.cam)
            while viewer.is_running():
                joints = self.receiver.get_most_recent_joints()
                self.engine.step(joints)
                viewer.sync()
                self.engine.sleep()

    def _run_dual_viewer(self) -> None:
        with mujoco.viewer.launch_passive(
            model=self.engine.fk_model, data=self.engine.fk_data
        ) as fk_viewer, mujoco.viewer.launch_passive(
            model=self.engine.ik_model,
            data=self.engine.ik_data,
            show_left_ui=False,
            show_right_ui=False,
        ) as ik_viewer:
            mujoco.mjv_defaultFreeCamera(self.engine.fk_model, fk_viewer.cam)
            mujoco.mjv_defaultFreeCamera(self.engine.ik_model, ik_viewer.cam)
            while fk_viewer.is_running() and ik_viewer.is_running():
                joints = self.receiver.get_most_recent_joints()
                self.engine.step(joints)
                fk_viewer.sync()
                ik_viewer.sync()
                self.engine.sleep()

    def run(self) -> None:
        self.engine.load_models()
        self.engine.build_tasks()
        self.engine.build_fk_indexes()
        self.receiver.start()

        try:
            mode = self._effective_viewer_mode()
            if mode == ViewerMode.HEADLESS:
                self._run_headless()
            elif mode == ViewerMode.SINGLE:
                self._run_single_viewer()
            else:
                self._run_dual_viewer()
        finally:
            self.receiver.stop()


def get_model_specs() -> Dict[str, ModelSpec]:
    return {
        "inspirehand": ModelSpec(
            name="inspirehand",
            ik_xml=_hand_scene_xml("inspire_hand", "scene.xml"),
            offset=(-0.055, 0.15, -0.29),
            scale=1.5,
            viewer_supports_dual=True,
            tip_site_aliases={
                "index_tip": "index_tip",
                "middle_tip": "middle_tip",
                "ring_tip": "ring_tip",
                "little_tip": "little_tip",
                "thumb_tip": "thumb_tip",
            },
            tip_target_aliases={
                "index_tip": "index_tip_target",
                "middle_tip": "middle_tip_target",
                "ring_tip": "ring_tip_target",
                "little_tip": "little_tip_target",
                "thumb_tip": "thumb_tip_target",
            },
            tip_position_adjustments={},
        ),
        "leaphand": ModelSpec(
            name="leaphand",
            ik_xml=_hand_scene_xml("leap_hand", "scene.xml"),
            offset=(-0.055, 0.15, -0.24),
            scale=1.5,
            viewer_supports_dual=True,
            tip_site_aliases={
                "index_tip": "index_tip",
                "middle_tip": "middle_tip",
                "ring_tip": "ring_tip",
                "little_tip": None,
                "thumb_tip": "thumb_tip",
            },
            tip_target_aliases={
                "index_tip": "index_tip_target",
                "middle_tip": "middle_tip_target",
                "ring_tip": "ring_tip_target",
                "little_tip": None,
                "thumb_tip": "thumb_tip_target",
            },
            tip_position_adjustments={},
        ),
        "shadowhand": ModelSpec(
            name="shadowhand",
            ik_xml=_hand_scene_xml("shadow_hand", "scene_right.xml"),
            offset=(-0.05, 0.1, -0.01),
            scale=1.0,
            viewer_supports_dual=True,
            tip_site_aliases={
                "index_tip": "index_tip",
                "middle_tip": "middle_tip",
                "ring_tip": "ring_tip",
                "little_tip": "little_tip",
                "thumb_tip": "thumb_tip",
            },
            tip_target_aliases={
                "index_tip": "index_tip_target",
                "middle_tip": "middle_tip_target",
                "ring_tip": "ring_tip_target",
                "little_tip": "little_tip_target",
                "thumb_tip": "thumb_tip_target",
            },
            tip_position_adjustments={},
        ),
        "allegro": ModelSpec(
            name="allegro",
            ik_xml=_hand_scene_xml("wonik_allegro", "scene_right.xml"),
            offset=(-0.055, 0.15, -0.24),
            scale=1.5,
            viewer_supports_dual=True,
            tip_site_aliases={
                "index_tip": "index_tip",
                "middle_tip": "middle_tip",
                "ring_tip": "ring_tip",
                "little_tip": None,
                "thumb_tip": "thumb_tip",
            },
            tip_target_aliases={
                "index_tip": "index_tip_target",
                "middle_tip": "middle_tip_target",
                "ring_tip": "ring_tip_target",
                "little_tip": None,
                "thumb_tip": "thumb_tip_target",
            },
            tip_position_adjustments={},
        ),
    }


def run_main(argv: Optional[Sequence[str]] = None) -> None:
    faulthandler.enable()
    args = parse_args(argv)
    specs = get_model_specs()
    spec = specs[args.model]
    if args.log_env:
        print(f"Platform: {platform.platform()}")
        print(f"MuJoCo version: {mujoco.mj_versionString()}")
    app = FkIkApp(model_spec=spec, viewer_mode=ViewerMode(args.viewer), fps=args.fps)
    app.run()


if __name__ == "__main__":
    run_main()
