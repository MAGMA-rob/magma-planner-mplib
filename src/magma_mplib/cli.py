"""Launch the MPLib service after parsing command-line arguments."""

import argparse


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Start the MPLib planner service.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--workers", type=int, default=1, help="Number of process-isolated v2 planning workers"
    )
    args = parser.parse_args(argv)
    import uvicorn
    from .service import create_app

    uvicorn.run(create_app(workers=args.workers), host=args.host, port=args.port)
    return 0
