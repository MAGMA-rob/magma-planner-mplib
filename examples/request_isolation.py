"""Executable MPLib 0.2.1 probe: base poses, full joints, geometry and isolation.

Run in the planner environment: python examples/request_isolation.py
Add --endpoint http://127.0.0.1:8000 to exercise HTTP process workers as well.
"""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import argparse
import json
from pathlib import Path
from time import perf_counter
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

import numpy as np
from mplib import Planner, Pose

from magma_mplib.planning import plan_request
import magma_mplib.planning as planning


def build_request(asset: str = "panda_v2") -> dict:
    assets = Path(planning.__file__).parent / "assets" / "panda"
    root = ET.parse(assets / f"{asset}.urdf").getroot()
    links = [item.attrib["name"] for item in root.findall("link")]
    joints = [item.attrib["name"] for item in root.findall("joint") if item.attrib["type"] != "fixed"]
    planner = Planner(
        str(assets / f"{asset}.urdf"),
        srdf=str(assets / f"{asset}.srdf"),
        move_group="panda_hand_tcp",
        user_link_names=links,
        user_joint_names=joints,
    )
    initial = np.asarray([0, 0.3, 0, -1.8, 0, 2.1, 0.78, 0.04, 0.04])
    goal = initial.copy()
    goal[0] += 0.1
    pinocchio = planner.robot.get_pinocchio_model()
    pinocchio.compute_forward_kinematics(goal)
    pose = pinocchio.get_link_pose(planner.link_name_2_idx["panda_hand_tcp"])
    return {
        "model": {
            "asset": asset,
            "link_names": links,
            "joint_names": joints,
            "velocity_limits": [0.9] * 7,
            "acceleration_limits": [0.9] * 7,
        },
        "base_pose": {"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]},
        "target_pose": {"position": pose.p.tolist(), "quaternion": pose.q.tolist()},
        "joint_positions": initial.tolist(),
        "time_step": 0.05,
        "seed": 123,
        "context": {"probe": asset},
        "scene": {"mode": "historical"},
    }


def http_plan(endpoint: str, request: dict, route: str = "/v2/plan") -> dict:
    encoded = json.dumps(request).encode()
    call = Request(endpoint.rstrip("/") + route, data=encoded, headers={"Content-Type": "application/json"})
    try:
        response = urlopen(call, timeout=35)
    except HTTPError as error:
        response = error
    with response:
        return json.load(response)


def main() -> None:
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--endpoint")
    args = parser.parse_args()
    started = perf_counter()
    initial = build_request()
    base = Pose([1.2, -0.4, 0.1], [np.cos(0.4), 0, 0, np.sin(0.4)])
    moved = deepcopy(initial)
    target = base * Pose(initial["target_pose"]["position"], initial["target_pose"]["quaternion"])
    moved["base_pose"] = {"position": base.p.tolist(), "quaternion": base.q.tolist()}
    moved["target_pose"] = {"position": target.p.tolist(), "quaternion": target.q.tolist()}
    moved["context"] = {"probe": "translated-rotated"}
    shapes = [
        {"kind": "box", "dimensions": [0.1, 0.1, 0.1]},
        {"kind": "sphere", "dimensions": [0.1]},
        {"kind": "capsule", "dimensions": [0.05, 0.1]},
        {
            "kind": "convex_mesh",
            "vertices": [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
            "faces": [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        },
    ]
    scene = deepcopy(initial)
    scene["scene"] = {
        "mode": "scene",
        "objects": [
            {
                "name": f"body-{index}",
                "geometry": shape,
                "pose": {"position": [3 + index, 3, 3], "quaternion": [1, 0, 0, 0]},
            }
            for index, shape in enumerate(shapes)
        ],
    }
    attached = deepcopy(initial)
    attached["scene"] = {
        "mode": "scene",
        "objects": [
            {
                "name": "attached",
                "geometry": shapes[1],
                "pose": {"position": [3, 3, 3], "quaternion": [1, 0, 0, 0]},
                "attached_link": "panda_hand_tcp",
                "touch_links": [],
            }
        ],
    }
    requests = [initial, moved, scene, attached, build_request("panda_v3"), initial]
    for request in requests:
        result = plan_request(request)
        print(request["context"], result["status"], len(result["positions"]), result["diagnostics"])
        assert result["status"] == "success", result
        assert result["context"] == request["context"]
        assert result["joint_names"] == initial["model"]["joint_names"][:7]
    retry = deepcopy(initial)
    retry["attempt"] = 1
    retry["seed"] = 321
    first_retry = plan_request(retry)
    plan_request(requests[-2])
    second_retry = plan_request(retry)
    assert first_retry["status"] == second_retry["status"] == "success"
    assert (
        first_retry["positions"] == second_retry["positions"]
    ), "RRT changed despite an identical request seed"
    blocked = deepcopy(initial)
    blocked["planning_time"] = 0.1
    blocked["scene"] = {
        "mode": "scene",
        "objects": [
            {
                "name": "blocker",
                "geometry": {"kind": "box", "dimensions": [4, 4, 4]},
                "pose": {"position": [0, 0, 0.5], "quaternion": [1, 0, 0, 0]},
            }
        ],
    }
    assert plan_request(blocked)["status"] == "no_solution"
    blocked["scene"]["allowed_contacts"] = [["blocker", name] for name in initial["model"]["link_names"]]
    assert plan_request(blocked)["status"] == "success"
    assert plan_request(initial)["status"] == "success"
    if args.endpoint:
        with ThreadPoolExecutor(max_workers=4) as executor:
            outputs = list(executor.map(lambda request: http_plan(args.endpoint, request), requests))
        assert all(result["status"] == "success" for result in outputs), outputs
        assert [result["context"] for result in outputs] == [request["context"] for request in requests]
        unsupported = deepcopy(initial)
        unsupported["scene"] = {
            "mode": "scene",
            "objects": [{"name": "bad", "geometry": {"kind": "heightmap"}, "pose": initial["base_pose"]}],
        }
        assert http_plan(args.endpoint, unsupported)["status"] == "unsupported_scene"
        invalid = deepcopy(initial)
        invalid["joint_positions"] = initial["joint_positions"][:7]
        assert http_plan(args.endpoint, invalid)["status"] == "invalid_request"
        timeout = deepcopy(initial)
        timeout["timeout"] = 0.000001
        assert http_plan(args.endpoint, timeout)["status"] == "timeout"
        assert http_plan(args.endpoint, initial)["status"] == "success"
    print(f"Request isolation checks passed in {perf_counter() - started:.3f}s")


if __name__ == "__main__":
    main()
