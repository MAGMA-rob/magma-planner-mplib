"""Request-isolated v2 protocol. This module does not import MPLib."""

from __future__ import annotations

from typing import Any, Literal
from pydantic import BaseModel, ConfigDict, Field, model_validator
import math


class Contract(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class PoseSpec(Contract):
    position: list[float] = Field(min_length=3, max_length=3)
    quaternion: list[float] = Field(min_length=4, max_length=4)

    @model_validator(mode="after")
    def normalize_quaternion(self) -> PoseSpec:
        norm = math.sqrt(sum(value * value for value in self.quaternion))
        if norm < 1e-12:
            raise ValueError("Quaternion cannot be zero; order is wxyz")
        self.quaternion = [value / norm for value in self.quaternion]
        return self


class RobotModelSpec(Contract):
    asset: str = Field(pattern=r"^[A-Za-z0-9_]+$")
    link_names: list[str] = Field(min_length=1)
    joint_names: list[str] = Field(min_length=1)
    move_group: str = "panda_hand_tcp"
    velocity_limits: list[float] = Field(min_length=1)
    acceleration_limits: list[float] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_model(self) -> RobotModelSpec:
        if len(set(self.joint_names)) != len(self.joint_names) or len(set(self.link_names)) != len(
            self.link_names
        ):
            raise ValueError("Model joint/link names must be unique")
        if self.move_group not in self.link_names:
            raise ValueError("The move group must be a model link")
        if len(self.velocity_limits) != len(self.acceleration_limits) or any(
            value <= 0 for value in self.velocity_limits + self.acceleration_limits
        ):
            raise ValueError("Velocity/acceleration limits must have matching positive values")
        return self


class GeometrySpec(Contract):
    kind: Literal["box", "sphere", "capsule", "convex_mesh"]
    dimensions: list[float] = Field(default_factory=list)
    vertices: list[list[float]] = Field(default_factory=list)
    faces: list[list[int]] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_geometry(self) -> GeometrySpec:
        count = {"box": 3, "sphere": 1, "capsule": 2}.get(self.kind)
        if count is not None:
            if len(self.dimensions) != count or any(value <= 0 for value in self.dimensions):
                raise ValueError("Invalid positive geometry dimensions")
            if self.vertices or self.faces:
                raise ValueError("Primitive shapes cannot contain mesh data")
        elif len(self.vertices) < 4 or not self.faces:
            raise ValueError("A convex mesh needs vertices and triangle faces")
        if any(len(vertex) != 3 for vertex in self.vertices) or any(
            len(face) != 3 or any(index < 0 or index >= len(self.vertices) for index in face)
            for face in self.faces
        ):
            raise ValueError("Mesh vertices and indexed triangles must be valid")
        return self


class CollisionBody(Contract):
    name: str = Field(min_length=1)
    geometry: GeometrySpec
    pose: PoseSpec
    attached_link: str | None = None
    touch_links: list[str] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_attachment(self) -> CollisionBody:
        if self.touch_links and self.attached_link is None:
            raise ValueError("Touch links require an attached collision body")
        return self


class PlanningScene(Contract):
    mode: Literal["historical", "scene"] = "historical"
    objects: list[CollisionBody] = Field(default_factory=list)
    allowed_contacts: list[tuple[str, str]] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_scene(self) -> PlanningScene:
        names = [body.name for body in self.objects]
        if len(names) != len(set(names)):
            raise ValueError("Scene object names must be unique")
        if self.mode == "historical" and (self.objects or self.allowed_contacts):
            raise ValueError("Scene objects require explicit scene mode")
        return self


class PlanRequestV2(Contract):
    model: RobotModelSpec
    base_pose: PoseSpec
    target_pose: PoseSpec
    joint_positions: list[float]
    time_step: float = Field(gt=0)
    seed: int = Field(ge=0, le=4294967295)
    attempt: int = Field(default=0, ge=0)
    planning_time: float = Field(default=2.0, gt=0, le=60)
    timeout: float = Field(default=30.0, gt=0, le=300)
    context: dict[str, Any] = Field(default_factory=dict)
    scene: PlanningScene = Field(default_factory=PlanningScene)

    @model_validator(mode="after")
    def validate_joint_positions(self) -> PlanRequestV2:
        if len(self.joint_positions) != len(self.model.joint_names):
            raise ValueError("Provide all physical joint positions in model joint order")
        return self


class PlanResponseV2(Contract):
    status: Literal[
        "success", "no_solution", "invalid_request", "timeout", "unsupported_scene", "service_error"
    ]
    joint_names: list[str] = Field(default_factory=list)
    positions: list[list[float]] = Field(default_factory=list)
    time_step: float
    context: dict[str, Any] = Field(default_factory=dict)
    diagnostics: dict[str, Any] = Field(default_factory=dict)
