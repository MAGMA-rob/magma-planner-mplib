"""Asynchronous HTTP dispatch to process-isolated planning workers."""

import asyncio
from concurrent.futures import ProcessPoolExecutor
from contextlib import asynccontextmanager
import multiprocessing
import math
from typing import Any

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.exception_handlers import request_validation_exception_handler
from fastapi.responses import JSONResponse
from .contracts import PlanRequestV2, PlanResponseV2
from .planning import plan_request


def create_app(*, workers: int = 1) -> FastAPI:
    if workers < 1:
        raise ValueError("At least one planning worker is required")

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        pool = ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context("spawn"))
        app.state.planning_pool = pool
        try:
            yield
        finally:
            pool.shutdown(wait=True, cancel_futures=True)

    app = FastAPI(lifespan=lifespan)

    @app.exception_handler(RequestValidationError)
    async def invalid_request(request: Request, error: RequestValidationError):
        if request.url.path != "/v2/plan":
            return await request_validation_exception_handler(request, error)
        body = error.body if isinstance(error.body, dict) else {}
        unsupported = any("geometry" in item["loc"] and "kind" in item["loc"] for item in error.errors())
        time_step = body.get("time_step")
        if not isinstance(time_step, (int, float)) or not math.isfinite(time_step) or time_step <= 0:
            time_step = 0.0
        result = PlanResponseV2(
            status="unsupported_scene" if unsupported else "invalid_request",
            time_step=time_step,
            context=body.get("context") if isinstance(body.get("context"), dict) else {},
            diagnostics={"reason": str(error)},
        )
        return JSONResponse(status_code=422, content=result.model_dump())

    @app.get("/health")
    async def health() -> dict[str, Any]:
        return {"status": "ready", "protocol_versions": ["2"], "workers": workers}

    @app.post("/v2/plan", response_model=PlanResponseV2)
    async def plan(request: PlanRequestV2) -> PlanResponseV2:
        future = asyncio.get_running_loop().run_in_executor(
            app.state.planning_pool, plan_request, request.model_dump()
        )
        try:
            result = await asyncio.wait_for(asyncio.shield(future), timeout=request.timeout)
            return PlanResponseV2.model_validate(result)
        except asyncio.TimeoutError:
            return PlanResponseV2(
                status="timeout",
                time_step=request.time_step,
                context=request.context,
                diagnostics={"reason": "Planning deadline exceeded"},
            )
        except Exception as error:
            return PlanResponseV2(
                status="service_error",
                time_step=request.time_step,
                context=request.context,
                diagnostics={"reason": str(error)},
            )

    return app
