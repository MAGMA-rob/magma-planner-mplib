# Planner service

Read `docs/planner-v2.md` before changing the planning API.
MPLib 0.2.1 runs separately from ManiSkill; do not merge their dependencies.

- A v2 request supplies its complete dynamic state. Never reuse poses, obstacles,
  attached bodies or allowed contacts from a previous request.
- Mutable MPLib models and global random seeds belong to one worker process.
- Expose only planner v2. Do not reintroduce `/init`, `/plan` or shared mutable robot state.
- Use named joint ordering and physical finger positions, never normalized gripper commands.
- Keep code typed and readable; avoid single-use forwarding helpers.
- Do not create test files without an explicit request. Validate through existing core
  integration tests and isolated planner checks.
