# Request-isolated planning v2

Run `magma-planner serve --workers 2` in the dedicated MPLib 0.2.1 environment
or planner Docker image. One worker is the default. Each worker is a process,
executes one calculation at a time and keeps its own bounded model cache. Do not
install the planner's MPLib dependencies into the ManiSkill environment.

`POST /v2/plan` needs no `/init`. Its full request consists of:

```json
{
  "model": {
    "asset": "panda_v2",
    "link_names": ["... all model links in physical order ..."],
    "joint_names": ["... all active joints, including fingers ..."],
    "move_group": "panda_hand_tcp",
    "velocity_limits": [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9],
    "acceleration_limits": [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]
  },
  "base_pose": {"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]},
  "target_pose": {"position": [0.4, 0, 0.3], "quaternion": [0, 1, 0, 0]},
  "joint_positions": [0, 0, 0, -1.8, 0, 2.1, 0.78, 0.04, 0.04],
  "time_step": 0.05,
  "seed": 123,
  "attempt": 0,
  "planning_time": 2,
  "timeout": 30,
  "context": {"execution_id": "...", "epoch": 0, "batch_id": 1, "robot": "arm1", "physical_version": 1},
  "scene": {"mode": "historical"}
}
```

The placeholder model names in this sketch must be replaced by the actual
ordered model links/joints. The response includes the joint order of its
trajectory; a consumer must map by names, never assume seven arm values followed
by a normalized gripper command. `joint_positions` contains complete physical
joint positions. Normalized gripper commands stay in the backend controller.
Velocity/acceleration limits describe the move-group joints, not the fingers.

## Frames and changing poses

All poses use the environment's local world frame: base, target, obstacles and
attached objects. Quaternions are explicitly `wxyz` and normalized on validation.
There is no implicit base transform on targets. ManiSkill vectorization offsets
are handled at its boundary and must not be added a second time.

Each request sets the real base pose and all joint positions. A scenario
`move_to`, translated/rotated base, displaced object or restored checkpoint needs
only a new request. Model identity does not change with base pose. Navigation
planning for the base is outside this service.

## Cache and isolation

The model fingerprint includes the descriptor, URDF/SRDF bytes and referenced
mesh bytes. Assets must be configured below the server's packaged asset root;
clients cannot supply arbitrary filesystem paths. Up to eight models are cached
per worker. Every job creates a new planning world/OMPL planner and supplies the
entire dynamic scene. Objects absent from the new request do not survive. Models
used with attachments are discarded afterwards, since attachments also mutate
articulated-model state. The service exposes only `/v2/plan`; `/init` and `/plan`
are removed and return HTTP 404. `/health` advertises `protocol_versions=["2"]`.

Each request explicitly seeds MPLib/OMPL. Attempt zero tries screw, then RRT when
screw fails or exceeds the historical 200-action preference. Retries use RRT.
The backend derives seeds from execution seed, batch, attempt, robot and phase;
arrival order is irrelevant. Coordinated trajectories of moving robots sharing
one world are not implemented.

## Opt-in scene collisions

Historical mode adds no scene obstacles and remains the default. Scene mode
requires the complete collision state, supplied by physical collision geometry
or an explicit scenario provider. It never guesses geometry from object names.

Each `scene.objects` entry has a unique `name`, `pose` and `geometry`:

- `box`: dimensions `[x, y, z]`.
- `sphere`: dimensions `[radius]`.
- `capsule`: dimensions `[radius, length]`.
- `convex_mesh`: `vertices` of three floats and triangular indexed `faces`.

An attached object has `attached_link` and explicit `touch_links`; its pose is
still world-local. The service computes its link-relative attachment from the
current robot state. It is not simultaneously added as an independent obstacle.
`allowed_contacts` is a list of exact name pairs. Unknown geometry or invalid
meshes produce diagnostics rather than silently removing collision bodies.

The backend serializes scene-mode batch motions and obtains a fresh scene at
movement boundaries. Providers update attachments/allowed contacts at the
appropriate tool phase, including other stationary robots. Merely enabling a
provider does not create geometry for the scenario: completeness is its contract.

## Results and deadlines

Response status is one of `success`, `no_solution`, `invalid_request`, `timeout`,
`unsupported_scene`, `service_error`. Success includes `joint_names`, `positions`,
`time_step`, the unchanged `context` and strategy diagnostics. An empty successful
trajectory means the current configuration already satisfies the goal.
Malformed planning requests return a structured v2 response with HTTP 422.

The HTTP service bounds response waiting with `timeout`; a calculation already
running may continue privately in its worker after the caller times out. It
cannot mutate a different request's world. `planning_time` bounds RRT work. The
injected core client has its own bounded HTTP timeout and runs outside the control
thread. Canceling physical work invalidates the pending response. The backend
checks full context and physical version before emitting any trajectory and
never emits the prefix of a failed segment.

Worker recycling after a native-library hang and distributed model assets are
future operational improvements. Service response deadlines do not promise
preemptive interruption of a native MPLib call.
