"""FastAPI transport for the RM75 web HMI.

REST carries bounded commands and snapshots; WebSocket carries timestamped
telemetry plus the browser-to-server control lease heartbeat.  No endpoint
accepts arbitrary ROS topic, service, launch file, or shell names.
"""
import asyncio
from contextlib import asynccontextmanager
import json
import logging
from pathlib import Path
import time
import uuid
from typing import Any, Dict, List, Optional

import numpy as np
from fastapi import FastAPI, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .services import WebRuntime
from .session import SessionError


LOG = logging.getLogger("rm75_sampling_gui.web.api")


class SessionRequest(BaseModel):
    client: str = Field(default="browser", max_length=120)


class JointRequest(BaseModel):
    target: List[float] = Field(min_length=7, max_length=7)


class CommandRequest(BaseModel):
    payload: Dict[str, Any] = Field(default_factory=dict)


class ControlPathRequest(BaseModel):
    path: str


class ToolPickConfiguration(BaseModel):
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    lift_height: float = Field(gt=0.0)


class CartesianRequest(BaseModel):
    name: Optional[str] = None
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    planner: str = "ompl"
    speed: int = 5


async def _thread(callback: Any, *args: Any) -> Any:
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, lambda: callback(*args))


def _session(value: Optional[str]) -> str:
    if not value:
        raise HTTPException(401, "需要有效的X-RM75-Control-Session")
    return value


def _image_jpeg(runtime: WebRuntime) -> bytes:
    if runtime.simulation:
        return runtime.simulation_jpeg()
    message = runtime.ros.monitor.image
    if message is None:
        raise RuntimeError("尚未收到RGB图像")
    encoding = str(message.encoding).lower()
    channels = 1 if encoding in ("mono8", "8uc1") else 4 if "rgba" in encoding else 3
    array = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.height) * int(message.step)
    if array.size < expected:
        raise RuntimeError("RGB图像缓冲区不完整")
    rows = array[:expected].reshape(int(message.height), int(message.step))
    pixels = rows[:, :int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels)
    import cv2
    if encoding.startswith("rgb"):
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    elif encoding.startswith("rgba"):
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    elif encoding.startswith("bgra"):
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    ok, encoded = cv2.imencode(".jpg", pixels,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        raise RuntimeError("RGB图像JPEG编码失败")
    return encoded.tobytes()


def create_app(runtime: WebRuntime, frontend_dir: Optional[str] = None) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        runtime.connect()
        stopping = asyncio.Event()

        async def heartbeat_loop() -> None:
            while not stopping.is_set():
                try:
                    runtime.publish_guarded_heartbeat()
                    runtime.sample_experiment()
                except Exception:
                    LOG.exception("控制会话心跳或实验采样失败")
                await asyncio.sleep(0.1)

        task = asyncio.create_task(heartbeat_loop())
        try:
            yield
        finally:
            stopping.set()
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            await _thread(runtime.close)

    app = FastAPI(
        title="RM75 Sampling Web Control Platform",
        version="1.0.0",
        docs_url="/api/docs",
        openapi_url="/api/openapi.json",
        lifespan=lifespan,
    )

    @app.exception_handler(SessionError)
    async def session_error(_request: Request, error: SessionError) -> JSONResponse:
        return JSONResponse(status_code=409, content={"detail": str(error)})

    @app.exception_handler(ValueError)
    async def value_error(_request: Request, error: ValueError) -> JSONResponse:
        return JSONResponse(status_code=422, content={"detail": str(error)})

    @app.exception_handler(RuntimeError)
    async def runtime_error(_request: Request, error: RuntimeError) -> JSONResponse:
        LOG.warning("请求被安全边界拒绝：%s", error)
        return JSONResponse(status_code=409, content={"detail": str(error)})

    @app.get("/api/v1/health")
    async def health() -> Dict[str, Any]:
        snap = await _thread(runtime.snapshot)
        return {"ok": True, "offline": runtime.offline,
                "simulation": runtime.simulation,
                "ros_master": snap.get("health", {}).get("master", False),
                "timestamp": time.time()}

    @app.get("/api/v1/state")
    async def state() -> Dict[str, Any]:
        return await _thread(runtime.snapshot)

    @app.get("/api/v1/model")
    async def model() -> Dict[str, Any]:
        return runtime.model_descriptor()

    @app.get("/api/v1/model/mesh/{index}")
    async def mesh(index: int) -> FileResponse:
        try:
            path = runtime.mesh_path(index)
        except IndexError as error:
            raise HTTPException(404, str(error)) from error
        return FileResponse(path, media_type="model/stl")

    @app.post("/api/v1/joints/preview")
    async def preview(request: JointRequest) -> Dict[str, Any]:
        return runtime.preview(request.target)

    @app.post("/api/v1/kinematics/fk")
    async def forward_kinematics(request: JointRequest) -> Dict[str, Any]:
        return await _thread(runtime.forward_kinematics, request.target)

    @app.post("/api/v1/cartesian/preview")
    async def cartesian_preview(request: CartesianRequest) -> Dict[str, Any]:
        payload = request.model_dump() if hasattr(request, "model_dump") else request.dict()
        return await _thread(runtime.inverse_kinematics, payload)

    @app.post("/api/v1/cartesian/execute")
    async def cartesian_execute(request: CartesianRequest,
                                x_rm75_control_session: Optional[str] = Header(default=None)) -> Dict[str, Any]:
        payload = request.model_dump() if hasattr(request, "model_dump") else request.dict()
        return await _thread(runtime.execute_cartesian,
                             _session(x_rm75_control_session), payload)

    @app.get("/api/v1/poses")
    async def poses() -> Dict[str, Any]:
        return {"poses": await _thread(runtime.poses)}

    @app.put("/api/v1/poses/{index}")
    async def save_pose(index: int, request: CartesianRequest,
                        x_rm75_control_session: Optional[str] = Header(default=None)) -> Dict[str, Any]:
        runtime.require_session(_session(x_rm75_control_session))
        payload = request.model_dump() if hasattr(request, "model_dump") else request.dict()
        return await _thread(runtime.save_pose, index, payload)

    @app.get("/api/v1/settings")
    async def settings() -> Dict[str, Any]:
        return runtime.settings()

    @app.put("/api/v1/settings")
    async def save_settings(request: Request) -> Dict[str, Any]:
        return runtime.save_settings(await request.json())

    @app.post("/api/v1/control/session")
    async def acquire(request: SessionRequest, http_request: Request) -> Dict[str, Any]:
        if runtime.offline:
            raise HTTPException(403, "OFFLINE模式禁止取得控制权")
        client = "{} ({})".format(request.client,
                                  http_request.client.host if http_request.client else "local")
        return runtime.sessions.acquire(client)

    @app.delete("/api/v1/control/session")
    async def release(x_rm75_control_session: Optional[str] = Header(default=None)) -> Dict[str, Any]:
        return {"released": runtime.sessions.release(
            _session(x_rm75_control_session))}

    @app.put("/api/v1/control/path")
    async def control_path(request: ControlPathRequest,
                           x_rm75_control_session: Optional[str] = Header(default=None)) -> Dict[str, Any]:
        return await _thread(runtime.set_control_path,
                             _session(x_rm75_control_session), request.path)

    @app.post("/api/v1/joints/execute")
    async def execute_joints(request: JointRequest,
                             x_rm75_control_session: Optional[str] = Header(default=None)) -> Any:
        return await _thread(runtime.execute_joints,
                             _session(x_rm75_control_session), request.target)

    @app.get("/api/v1/tools")
    async def tools() -> Dict[str, Any]:
        return {"tools": runtime.tool_configurations()}

    @app.put("/api/v1/tools/{slot_id}")
    async def save_tool(slot_id: int, request: ToolPickConfiguration,
                        x_rm75_control_session: Optional[str] = Header(default=None)) -> Any:
        payload = request.model_dump() if hasattr(request, "model_dump") else request.dict()
        return await _thread(runtime.save_tool_configuration, slot_id,
                             payload, _session(x_rm75_control_session))

    @app.post("/api/v1/tools/{slot_id}/dry-run")
    async def dry_run_tool(slot_id: int,
                           x_rm75_control_session: Optional[str] = Header(default=None)) -> Any:
        return await _thread(runtime.dry_run_configured_tool, slot_id,
                             _session(x_rm75_control_session))

    @app.post("/api/v1/tools/{slot_id}/execute")
    async def execute_tool(slot_id: int,
                           x_rm75_control_session: Optional[str] = Header(default=None)) -> Any:
        return await _thread(runtime.execute_configured_tool, slot_id,
                             _session(x_rm75_control_session))

    @app.post("/api/v1/system/{name}/{action}")
    async def lifecycle(name: str, action: str,
                        x_rm75_control_session: Optional[str] = Header(default=None)) -> Any:
        return await _thread(runtime.lifecycle, name, action,
                             _session(x_rm75_control_session))

    @app.post("/api/v1/commands/{name}")
    async def command(name: str, request: CommandRequest,
                      x_rm75_control_session: Optional[str] = Header(default=None)) -> Any:
        return await _thread(runtime.command, name,
                             _session(x_rm75_control_session), request.payload)

    @app.get("/api/v1/parameters")
    async def parameters() -> Dict[str, Any]:
        return runtime.parameters()

    @app.get("/api/v1/experiments")
    async def experiments() -> List[Dict[str, Any]]:
        return runtime.experiments()

    @app.get("/api/v1/vision/frame.jpg")
    async def frame() -> Response:
        try:
            image = await _thread(_image_jpeg, runtime)
        except Exception as error:
            raise HTTPException(503, str(error)) from error
        return Response(image, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.websocket("/api/v1/ws")
    async def websocket(websocket: WebSocket) -> None:
        await websocket.accept()
        connection_id = uuid.uuid4().hex
        bound_session: Optional[str] = None
        send_lock = asyncio.Lock()

        async def send(message: Dict[str, Any]) -> None:
            async with send_lock:
                await websocket.send_json(message)

        async def sender() -> None:
            while True:
                snapshot = await _thread(runtime.snapshot)
                await send({"type": "state", "data": snapshot})
                await asyncio.sleep(0.1)

        async def receiver() -> None:
            nonlocal bound_session
            while True:
                message = await websocket.receive_json()
                kind = message.get("type")
                if kind == "heartbeat":
                    session_id = str(message.get("session_id", ""))
                    if bound_session is None:
                        runtime.sessions.bind_websocket(session_id, connection_id)
                        bound_session = session_id
                    runtime.sessions.heartbeat(session_id, connection_id)
                    await send({"type": "heartbeat_ack", "timestamp": time.time()})
                elif kind == "ping":
                    await send({"type": "pong", "timestamp": time.time()})
                else:
                    await send({"type": "error", "message": "不支持的WebSocket消息"})

        send_task = asyncio.create_task(sender())
        receive_task = asyncio.create_task(receiver())
        try:
            done, pending = await asyncio.wait(
                (send_task, receive_task), return_when=asyncio.FIRST_EXCEPTION)
            for task in done:
                task.result()
            for task in pending:
                task.cancel()
        except (WebSocketDisconnect, SessionError):
            pass
        finally:
            send_task.cancel()
            receive_task.cancel()
            if bound_session:
                runtime.sessions.release(bound_session, connection_id)

    build = Path(frontend_dir).resolve() if frontend_dir else None
    if build and build.is_dir():
        assets = build / "assets"
        if assets.is_dir():
            app.mount("/assets", StaticFiles(directory=str(assets)), name="assets")

        @app.get("/{path:path}")
        async def frontend(path: str) -> FileResponse:
            candidate = (build / path).resolve()
            if path and candidate.is_file() and build in candidate.parents:
                headers = ({"Cache-Control": "no-store, max-age=0"}
                           if candidate.name == "index.html" else None)
                return FileResponse(str(candidate), headers=headers)
            # index.html contains the current content-hashed JS/CSS filenames.
            # Caching it can pin the browser to an obsolete UI after a restart.
            return FileResponse(
                str(build / "index.html"),
                headers={"Cache-Control": "no-store, max-age=0"},
            )

    return app
