# Main script for running the mplib planner based on OMPL
# Author : Loan BERNAT (l.bernat@sileane.com)

# ruff: noqa: E402
from typing import List, Optional
import logging
from pydantic import BaseModel, Field

import numpy as np
from mplib.pymp import Pose
from mplib.planner import Planner
from pathlib import Path
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

class PlanResponse(BaseModel):
    status: str
    position: Optional[List[List[float]]] = None

BASE_DIR = Path(__file__).parent
logger = logging.getLogger("uvicorn.error")

class MPLIBServer:
    planner : Optional[Planner]

    def __init__(self):
        self.planner = None

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
            print("ERROR : Planner not initialized. Please call /init before.")
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

            result = self.planner.plan_screw(
                pose,
                physical_qpos,
                time_step=self.control_timestep
            )

            screw_status = result["status"]
            if screw_status == "Success":
                logger.info("Motion plan succeeded with screw planning")
            else:
                logger.warning(
                    "Screw planning failed with status %r; falling back to RRT",
                    screw_status,
                )
                result = self.planner.plan_pose(
                    pose,
                    physical_qpos,
                    time_step=self.control_timestep,
                    wrt_world=True,
                )

                if result["status"] == "Success":
                    logger.info(
                        "Motion plan succeeded with RRT after screw status %r",
                        screw_status,
                    )
                else:
                    logger.error(
                        "Motion planning failed with screw status %r and RRT status %r",
                        screw_status,
                        result["status"],
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
