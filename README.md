# MAGMA Planner – MPLIB Backend
![Docker](https://img.shields.io/badge/Docker-Supported-blue)
![Ubuntu version](https://img.shields.io/badge/Ubuntu-24.04-blue)

This repository provides a Python package exposing a FastAPI/Uvicorn server implementing a motion planner backend for **MAGMA-GEN** and **MAGMA-BENCH**.

The default implementation relies on [**MPLIB**](https://motion-planning-lib.readthedocs.io/latest/), but any planner can be used as long as it exposes the same API contract (`/init` and `/plan`).

---

# Overview

The planner is deployed as a standalone HTTP server.

It is designed to:
- Receive initialization parameters (`/init`)
- Compute motion plans (`/plan`)
- Return trajectories in a format compatible with MAGMA

It can be launched in two ways:
1. Directly as a Python package
2. Inside a Docker container

---

# Installation and Usage

## Option 1 — Run as a Python Package

### 1. Clone the repository

```bash
git clone git@github.com:MAGMA-s/magma-planner-mplib.git
cd magma-planner-mplib
```

### 2. Install dependencies

```bash
pip install .
```

### 3. Launch the server

```bash
python3 -m magma_mplib
```

By default, the server runs on:

```
http://0.0.0.0:8000
```

You can test it with:

```bash
curl http://localhost:8000
```

---

## Option 2 — Run with Docker (Recommended for Deployment)

### 1. Build and Launch the planner

```bash
bash scripts/launch_planner.bash
```

This script:
- Starts the container
- Publishes port `8000`
- Runs the server inside Docker

You should see something similar to:

```
0.0.0.0:8000->8000/tcp
```

You can test it with:

```bash
curl http://localhost:8000
```

---

# Integration with MAGMA-GEN and MAGMA-BENCH

To use this planner inside MAGMA:

1. Open your `config.yaml`
2. Set the planner server address:

```yaml
magma_planner_address: "http://localhost:8000"
```

This tells MAGMA where to send:
- `/init` requests at startup
- `/plan` requests during execution

The planner must be running before launching MAGMA-GEN or MAGMA-BENCH.

---

# Planner API Specification

Any planner backend must expose the following HTTP endpoints:

- `POST /init`
- `POST /plan`

The request and response payloads must match exactly the following schemas.

---

# `/init` Endpoint

## Request Schema

```python
class PlannerOptions(BaseModel):
    robot: str
    joint_vel_limit: float
    joint_acc_limit: float

class InitPlannerRequest(BaseModel):
    options: PlannerOptions
    link_names: List[str]
    joint_names: List[str]
    base_pose: List[float]          # [x, y, z, qx, qy, qz, qw]
    control_timesteps: float
    move_group: str = "panda_hand_tcp"
```

## Description

- Loads URDF/SRDF
- Sets velocity and acceleration limits
- Sets base pose
- Configures planning group
- Stores control timestep

Must return HTTP 200 on success.

---

# `/plan` Endpoint

## Request Schema

```python
class PlanRequest(BaseModel):
    pose: List[float]               # [x, y, z, qx, qy, qz, qw]
    robot_qpos: List[float]         # Seven arm joints; excludes the gripper
    base_pose: Optional[List[float]] = None
```

## Response Schema

```python
class PlanResponse(BaseModel):
    status: str                     # "Success" or "Failure"
    position: Optional[List[List[float]]] = None
```

## Description

- Computes trajectory from current joint configuration
- Optionally updates base pose
- Returns a time-indexed list of joint configurations
- Must return `"Failure"` if planning fails

The returned trajectory must include:
- The seven planned arm joints

The MAGMA action converter owns the normalized gripper command and appends it
to the planner trajectory when constructing ManiSkill controller actions. The
MPLIB backend completes the seven received arm joints with its valid physical
finger positions because MPLIB 0.2.1 requires the full robot configuration in
the RRT fallback.

---

# Creating Your Own Planner Backend

You are not required to use MPLIB.

Any planner implementation is valid as long as:

1. It exposes:
   - `POST /init`
   - `POST /plan`
2. It respects the exact request and response schemas
3. It returns trajectories compatible with MAGMA

Minimal example structure:

```python
from fastapi import FastAPI
import uvicorn

app = FastAPI()

@app.post("/init")
def init_planner(req: InitPlannerRequest):
    # Initialize your planner here
    return "OK"

@app.post("/plan")
def plan(req: PlanRequest):
    # Compute trajectory here
    return PlanResponse(status="Success", position=trajectory)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

Your planner can use:
- OMPL
- MoveIt
- Custom optimization
- Learned planners
- Any internal architecture

As long as the API contract is respected, MAGMA will remain compatible.


---

# Support

Contact:  
l.bernat@sileane.com

---

# Authors

Loan BERNAT  
l.bernat@sileane.com

---

# License

BSD 2-Clause
