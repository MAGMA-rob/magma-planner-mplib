"""Launch the MPLib service after parsing command-line arguments."""

import argparse


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Start the MPLib planner service.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args(argv)
    from .__main__ import MPLIBServer

    MPLIBServer().run(host=args.host, port=args.port)
    return 0
