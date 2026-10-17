"""Worker-local MPLib cache; every job rebuilds the complete dynamic world."""

from __future__ import annotations

from collections import OrderedDict
from hashlib import sha256
from pathlib import Path
import traceback
from typing import Any
import xml.etree.ElementTree as ET

from .contracts import GeometrySpec, PlanRequestV2, PlanResponseV2, PoseSpec

_MODELS: OrderedDict[str, Any] = OrderedDict()
_ASSETS = Path(__file__).parent / "assets" / "panda"


def _pose(spec: PoseSpec) -> Any:
    from mplib import Pose

    return Pose(spec.position, spec.quaternion)


def _geometry(spec: GeometrySpec) -> Any:
    import numpy as np
    from mplib.collision_detection import fcl

    if spec.kind == "box":
        return fcl.Box(*spec.dimensions)
    if spec.kind == "sphere":
        return fcl.Sphere(spec.dimensions[0])
    if spec.kind == "capsule":
        return fcl.Capsule(*spec.dimensions)
    return fcl.Convex(np.asarray(spec.vertices), np.asarray(spec.faces, dtype=np.int32))


def plan_request(payload: dict[str, Any]) -> dict[str, Any]:
    """Process entrypoint. No live planner object crosses process boundaries."""
    import numpy as np
    from mplib import Planner, PlanningWorld, set_global_seed
    from mplib.collision_detection import fcl

    request = PlanRequestV2.model_validate(payload)
    diagnostics: dict[str, Any] = {}
    planner = None
    try:
        set_global_seed(request.seed)
        urdf = _ASSETS / f"{request.model.asset}.urdf"
        srdf = _ASSETS / f"{request.model.asset}.srdf"
        if not urdf.is_file() or not srdf.is_file():
            return PlanResponseV2(
                status="invalid_request",
                time_step=request.time_step,
                context=request.context,
                diagnostics={"reason": "Unknown configured robot asset"},
            ).model_dump()
        assets = sha256(request.model.model_dump_json().encode() + urdf.read_bytes() + srdf.read_bytes())
        for mesh in sorted({element.attrib["filename"] for element in ET.parse(urdf).iter("mesh")}):
            mesh_path = (urdf.parent / mesh).resolve()
            if not mesh_path.is_relative_to(_ASSETS.resolve()) or not mesh_path.is_file():
                raise ValueError(f"Robot mesh is not a configured asset: {mesh}")
            assets.update(mesh.encode())
            assets.update(mesh_path.read_bytes())
        fingerprint = assets.hexdigest()
        planner = _MODELS.pop(fingerprint, None)
        if planner is None:
            planner = Planner(
                str(urdf),
                srdf=str(srdf),
                move_group=request.model.move_group,
                user_link_names=request.model.link_names,
                user_joint_names=request.model.joint_names,
                joint_vel_limits=request.model.velocity_limits,
                joint_acc_limits=request.model.acceleration_limits,
            )
        _MODELS[fingerprint] = planner
        while len(_MODELS) > 8:
            _MODELS.popitem(last=False)
        # Replacing the entire world clears objects, attachments and allowed contacts.
        planner.planning_world = PlanningWorld([planner.robot])
        from mplib.planning import ompl

        planner.planner = ompl.OMPLPlanner(world=planner.planning_world)
        planner.set_base_pose(_pose(request.base_pose))
        planner.robot.set_qpos(np.asarray(request.joint_positions), True)
        known_names = set(request.model.link_names) | {body.name for body in request.scene.objects}
        if any(body.name in request.model.link_names for body in request.scene.objects):
            raise ValueError("Scene body names must not collide with robot link names")
        if any(
            first not in known_names or second not in known_names or first == second
            for first, second in request.scene.allowed_contacts
        ):
            raise ValueError("Allowed contacts must reference two distinct known collision bodies")
        for body in request.scene.objects:
            if body.attached_link is None:
                planner.planning_world.add_object(
                    body.name, fcl.CollisionObject(_geometry(body.geometry), _pose(body.pose))
                )
            else:
                if body.attached_link not in planner.link_name_2_idx:
                    raise ValueError(f"Unknown attached link {body.attached_link}")
                planner.planning_world.add_object(
                    body.name, fcl.CollisionObject(_geometry(body.geometry), _pose(body.pose))
                )
                planner.planning_world.attach_object(
                    body.name,
                    planner.robot.get_name(),
                    planner.link_name_2_idx[body.attached_link],
                    list(body.touch_links),
                )
                for link in body.touch_links:
                    if link not in planner.link_name_2_idx:
                        raise ValueError(f"Unknown touch link {link}")
                    planner.planning_world.get_allowed_collision_matrix().set_entry(body.name, link, True)
        for first, second in request.scene.allowed_contacts:
            planner.planning_world.get_allowed_collision_matrix().set_entry(first, second, True)
        diagnostics["model_fingerprint"] = fingerprint
        initial = np.asarray(request.joint_positions, dtype=float)
        target = _pose(request.target_pose)
        result = None
        screw = None
        if request.attempt == 0:
            screw = planner.plan_screw(target, initial.copy(), time_step=request.time_step, wrt_world=True)
            diagnostics["screw_status"] = screw["status"]
            if screw["status"] == "Success" and len(screw["position"]) <= 200:
                result = screw
        if result is None:
            result = planner.plan_pose(
                target,
                initial.copy(),
                time_step=request.time_step,
                planning_time=request.planning_time,
                wrt_world=True,
            )
            diagnostics["rrt_status"] = result["status"]
            if result["status"] != "Success" and screw is not None and screw["status"] == "Success":
                result = screw
                diagnostics["fallback"] = "long_screw_trajectory"
        if result["status"] != "Success":
            return PlanResponseV2(
                status="no_solution",
                time_step=request.time_step,
                context=request.context,
                diagnostics=diagnostics,
            ).model_dump()
        joint_names = [request.model.joint_names[index] for index in planner.move_group_joint_indices]
        positions = np.asarray(result["position"], dtype=float)
        if positions.ndim != 2 or positions.shape[1] != len(joint_names) or not np.isfinite(positions).all():
            raise ValueError("MPLib returned an invalid trajectory")
        return PlanResponseV2(
            status="success",
            joint_names=joint_names,
            positions=positions.tolist(),
            time_step=request.time_step,
            context=request.context,
            diagnostics=diagnostics,
        ).model_dump()
    except (ValueError, KeyError, AssertionError) as error:
        return PlanResponseV2(
            status="invalid_request",
            time_step=request.time_step,
            context=request.context,
            diagnostics={**diagnostics, "reason": str(error)},
        ).model_dump()
    except Exception as error:
        return PlanResponseV2(
            status="service_error",
            time_step=request.time_step,
            context=request.context,
            diagnostics={**diagnostics, "reason": str(error), "traceback": traceback.format_exc()},
        ).model_dump()
    finally:
        if planner is not None:
            # Attachments also affect the articulated model, not only PlanningWorld.
            # A job with attachments discards the model rather than trusting an incomplete cleanup.
            if any(body.attached_link is not None for body in request.scene.objects):
                for key, cached in list(_MODELS.items()):
                    if cached is planner:
                        del _MODELS[key]
