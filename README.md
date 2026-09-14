# MAGMA Planner — MPLib

Motion planner HTTP server for MAGMA-GEN and MAGMA-BENCH, powered by MPLib.

## Run with Docker (recommended)

Install Git and Docker, then run these commands from a terminal:

```bash
git clone https://github.com/MAGMA-rob/magma-planner-mplib.git
cd magma-planner-mplib
bash scripts/launch_planner.bash -p 8000
```

The script builds the image locally, installs the planner dependencies inside it,
and starts the server on port 8000. No Python environment is needed on the host.
Keep this terminal running while using MAGMA. To stop the container from another
terminal, run `docker stop magma_mplib`.

### Why Docker?

This planner requires **MPLib 0.2.1**, whereas **ManiSkill 3.0.1 requires MPLib
0.1.1 on Linux**. These versions cannot be installed together in the same Python
environment. Docker isolates the planner dependencies from the MAGMA simulation
environment. The two processes communicate over HTTP.

## Run without Docker

Docker is optional: use a **separate Python environment** for the planner.
Do not install it in the environment containing MAGMA-GEN or ManiSkill.

After cloning the repository, run from its root:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
magma-planner serve --host 127.0.0.1 --port 8000
```

Python 3.10 matches the Docker image. The editable installation lets you modify
the planner code; restart the server after changes. Launch MAGMA-GEN or
MAGMA-BENCH from their own environment in another terminal.

## Connect to MAGMA

For MAGMA running on the same host, set this in its configuration:

```yaml
magma_planner_address: "http://localhost:8000"
```

Start the planner before launching generation or evaluation. If MAGMA runs on
another machine or inside a container, use the planner's address reachable from
that environment instead of `localhost`.

## Documentation and support

See the [official documentation](https://magma-rob.github.io/docs/intro) for the
planner API and integration details. Report bugs through
[GitHub Issues](https://github.com/MAGMA-rob/magma-planner-mplib/issues).

## License

[BSD 2-Clause](LICENSE).
