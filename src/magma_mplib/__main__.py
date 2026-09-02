# Main script for running the mplib planner based on OMPL
# Author : Loan BERNAT (l.bernat@sileane.com)

# ruff: noqa: E402
import logging
import os
from pathlib import Path
from typing import List, Optional

import numpy as np
from mplib import set_global_seed
from pydantic import BaseModel, Field
from mplib.pymp import Pose
from mplib.planner import Planner
import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse

## INIT
class PlannerOptions(BaseModel):
    robot: str
    joint_vel_limit: float
    joint_acc_limit: float

class InitPlannerRequest(BaseModel):
    options: PlannerOptions
    link_names: List[str]
    joint_names: List[str]
    base_pose: List[float]
    control_timesteps: float
    move_group: str = "panda_hand_tcp"

## PLAN
class PlanRequest(BaseModel):
    pose: List[float] = Field(..., min_length=7, max_length=7)
    robot_qpos: List[float] = Field(..., min_length=7, max_length=7)
    base_pose: Optional[List[float]] = Field(None, min_length=7, max_length=7)
    robot_name: Optional[str] = None
    env_id: Optional[int] = None
    waypoint_index: Optional[int] = None
    planner_attempt: int = Field(0, ge=0)

class PlanResponse(BaseModel):
    status: str
    position: Optional[List[List[float]]] = None

BASE_DIR = Path(__file__).parent
logger = logging.getLogger("uvicorn.error")
MAX_ACTIONS_PER_WAYPOINT = 200
BASE_PLANNER_SEED = 0

class MPLIBServer:
    planner : Optional[Planner]

    def __init__(self):
        self.planner = None
        self.debug_logging = os.getenv("MAGMA_PLANNER_DEBUG", "0").lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

    def initPlanner(self, req : InitPlannerRequest):

        urdf_path = str((BASE_DIR / "assets" / "panda" / f"{req.options.robot}.urdf").resolve())
        srdf_path = str((BASE_DIR / "assets" / "panda" / f"{req.options.robot}.srdf").resolve())

        self.planner = Planner(
            urdf=urdf_path,
            srdf=srdf_path,
            user_link_names=req.link_names,
            user_joint_names=req.joint_names,
            move_group=req.move_group,
            joint_vel_limits=np.ones(7) * req.options.joint_vel_limit,
            joint_acc_limits=np.ones(7) * req.options.joint_acc_limit
        )
        self.control_timestep = req.control_timesteps
        pose = Pose(
            p=np.asarray(req.base_pose[0:3]),
            q=np.asarray(req.base_pose[3:])
        )
        self.planner.set_base_pose(pose)
        
        return JSONResponse("OK")

    def plan(self, req : PlanRequest):
        if self.planner is None:
            logger.error("Planner not initialized. Please call /init before /plan.")
            return PlanResponse(status="Failure")
        try:
            if req.base_pose is not None:
                base_pose = Pose(
                    p=np.asarray(req.base_pose[0:3]),
                    q=np.asarray(req.base_pose[3:])
                )
                self.planner.set_base_pose(base_pose)

            pose = Pose(
                p=np.asarray(req.pose[0:3]),
                q=np.asarray(req.pose[3:])
            )
            arm_qpos = np.asarray(req.robot_qpos)
            physical_qpos = self.planner.robot.get_qpos().copy()
            physical_qpos[:len(arm_qpos)] = arm_qpos
            planner_seed = BASE_PLANNER_SEED + req.planner_attempt
            set_global_seed(planner_seed)

            request_context = (
                f"robot={req.robot_name!r} env={req.env_id} "
                f"waypoint={req.waypoint_index} attempt={req.planner_attempt} "
                f"seed={planner_seed}"
            )
            if self.debug_logging:
                self.planner.pinocchio_model.compute_forward_kinematics(
                    physical_qpos
                )
                tcp_link_index = self.planner.link_name_2_idx[
                    self.planner.move_group
                ]
                tcp_pose_in_base = self.planner.pinocchio_model.get_link_pose(
                    tcp_link_index
                )
                planner_base_pose = self.planner.robot.get_base_pose()
                current_tcp_pose = planner_base_pose * tcp_pose_in_base
                base_values = np.concatenate(
                    [planner_base_pose.p, planner_base_pose.q]
                ).round(4).tolist()
                tcp_values = np.concatenate(
                    [current_tcp_pose.p, current_tcp_pose.q]
                ).round(4).tolist()
                target_values = np.asarray(req.pose).round(4).tolist()
                arm_values = arm_qpos.round(4).tolist()
                request_context += (
                    f" base={base_values}"
                    f" current_tcp={tcp_values}"
                    f" target={target_values}"
                    f" arm_qpos={arm_values}"
                )

            result = None
            screw_result = None
            screw_status = "Skipped"
            screw_action_count = None

            if req.planner_attempt == 0:
                screw_result = self.planner.plan_screw(
                    pose,
                    physical_qpos,
                    time_step=self.control_timestep,
                )
                screw_status = screw_result["status"]
                screw_action_count = (
                    len(screw_result["position"])
                    if screw_status == "Success"
                    else None
                )
                screw_is_too_long = (
                    screw_action_count is not None
                    and screw_action_count > MAX_ACTIONS_PER_WAYPOINT
                )

                if screw_status == "Success" and not screw_is_too_long:
                    result = screw_result
                    if self.debug_logging:
                        logger.info(
                            "Motion plan succeeded with screw planning: %s",
                            request_context,
                        )
                elif screw_is_too_long:
                    logger.warning(
                        (
                            "Screw trajectory has %d actions, above the %d-action "
                            "limit; trying RRT: %s"
                        ),
                        screw_action_count,
                        MAX_ACTIONS_PER_WAYPOINT,
                        request_context,
                    )
                elif self.debug_logging:
                    logger.warning(
                        (
                            "Screw planning failed with status %r; "
                            "falling back to RRT: %s"
                        ),
                        screw_status,
                        request_context,
                    )
            elif self.debug_logging:
                logger.info("Planner retry uses RRT directly: %s", request_context)

            if result is None:
                rrt_result = self.planner.plan_pose(
                    pose,
                    physical_qpos,
                    time_step=self.control_timestep,
                    wrt_world=True,
                )

                if rrt_result["status"] == "Success":
                    result = rrt_result
                    rrt_action_count = len(rrt_result["position"])
                    if rrt_action_count > MAX_ACTIONS_PER_WAYPOINT:
                        logger.warning(
                            (
                                "RRT trajectory has %d actions, above the %d-action "
                                "limit: %s"
                            ),
                            rrt_action_count,
                            MAX_ACTIONS_PER_WAYPOINT,
                            request_context,
                        )
                    if self.debug_logging:
                        logger.info(
                            "Motion plan succeeded with RRT after screw status %r: %s",
                            screw_status,
                            request_context,
                        )
                elif screw_status == "Success" and screw_result is not None:
                    result = screw_result
                    logger.warning(
                        (
                            "RRT planning failed with status %r after a %d-action "
                            "screw trajectory; keeping the screw trajectory: %s"
                        ),
                        rrt_result["status"],
                        screw_action_count,
                        request_context,
                    )
                else:
                    result = rrt_result
                    logger.error(
                        "Motion planning failed with screw status %r and RRT status %r: %s",
                        screw_status,
                        result["status"],
                        request_context,
                    )

            if result["status"] != "Success":
                return PlanResponse(status="Failure")

            return PlanResponse(
                status="Success",
                position=np.asarray(result["position"]).tolist()
            )
        except Exception as e:
            logger.exception("Unexpected motion planning error: %s", e)
            return PlanResponse(status="Failure")

    def run(self, host: str = "MAGMA_mplib", port: int = 8000) -> None:
        self.app = FastAPI()
        self.app.post("/init")(self.initPlanner)
        self.app.post("/plan")(self.plan)
        uvicorn.run(self.app, host=host, port=port)

if __name__ == "__main__":
    server = MPLIBServer()
    server.run(host="0.0.0.0", port=8000)
