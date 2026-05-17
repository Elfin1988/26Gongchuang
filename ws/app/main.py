from __future__ import annotations

import asyncio
import errno
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import serial
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .protocol import (
    COMMANDS,
    FLAG_NEED_ACK,
    Frame,
    MODULES,
    RESULT_CODES,
    SequenceGenerator,
    build_payload,
)
from .serial_manager import SerialManager, SerialSettings

BASE_DIR = Path(__file__).resolve().parent


class ConnectionRequest(BaseModel):
    port: str
    baudrate: int = 115200
    data_bits: int = Field(default=8, alias="dataBits")
    parity: str = "N"
    stop_bits: float = Field(default=1, alias="stopBits")


class CommandRequest(BaseModel):
    module: int
    cmd: int
    need_ack: bool = Field(default=True, alias="needAck")
    payload: Dict[str, Any] = Field(default_factory=dict)


class RawSendRequest(BaseModel):
    mode: str = "hex"
    data: str


class CommandPreviewRequest(BaseModel):
    module: int
    cmd: int
    need_ack: bool = Field(default=True, alias="needAck")
    payload: Dict[str, Any] = Field(default_factory=dict)


class ConnectionManager:
    def __init__(self) -> None:
        self._clients: Set[WebSocket] = set()
        self._queue: "asyncio.Queue[Dict[str, Any]]" = asyncio.Queue()
        self._broadcast_task: Optional[asyncio.Task] = None

    async def start(self) -> None:
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())

    async def stop(self) -> None:
        if self._broadcast_task:
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self._clients.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self._clients.discard(websocket)

    def publish(self, event: Dict[str, Any]) -> None:
        self._queue.put_nowait(event)

    async def _broadcast_loop(self) -> None:
        while True:
            event = await self._queue.get()
            stale: List[WebSocket] = []
            for client in list(self._clients):
                try:
                    await client.send_json(event)
                except Exception:
                    stale.append(client)
            for client in stale:
                self.disconnect(client)


ws_manager = ConnectionManager()
serial_manager = SerialManager(ws_manager.publish)
seq_gen = SequenceGenerator()


def friendly_serial_error_message(exc: Exception, port: Optional[str] = None) -> str:
    raw_message = str(exc).strip() or exc.__class__.__name__
    port_label = port or "当前串口"
    lower_message = raw_message.lower()
    winerror = getattr(exc, "winerror", None)
    err_no = getattr(exc, "errno", None)

    if winerror == 121 or "timeout" in lower_message or "信号灯超时时间已到" in raw_message:
        return (
            f"无法打开 {port_label}。设备没有正常响应，请检查下位机是否已连接、是否已上电，"
            f"以及串口号是否已经变化。\n原始错误: {raw_message}"
        )

    if err_no in (errno.EACCES, errno.EBUSY, 13) or "access is denied" in lower_message or "permission" in lower_message:
        return (
            f"无法打开 {port_label}。这个串口可能正被其他软件占用，或者当前用户没有访问权限。\n"
            f"原始错误: {raw_message}"
        )

    if err_no in (errno.ENOENT, 2) or "file not found" in lower_message or "cannot find the file" in lower_message:
        return (
            f"找不到 {port_label}。请先刷新串口列表，确认设备仍然在线，或检查插拔后串口号是否改变。\n"
            f"原始错误: {raw_message}"
        )

    return f"串口操作失败，请检查连接、供电和端口占用情况。\n原始错误: {raw_message}"


@asynccontextmanager
async def lifespan(_: FastAPI):
    await ws_manager.start()
    try:
        yield
    finally:
        await serial_manager.close()
        await ws_manager.stop()


app = FastAPI(title="26Gongchuang Web Serial Station", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")


@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return (BASE_DIR / "templates" / "index.html").read_text(encoding="utf-8")


@app.get("/api/meta")
async def get_meta() -> dict:
    commands = []
    for (module, cmd), spec in COMMANDS.items():
        commands.append(
            {
                "module": module,
                "module_name": MODULES.get(module, f"0x{module:02X}"),
                "cmd": cmd,
                "name": spec["name"],
                "request_fields": spec.get("request_fields", []),
            }
        )
    return {
        "modules": MODULES,
        "results": RESULT_CODES,
        "commands": commands,
        "serialDefaults": {
            "baudrate": 115200,
            "dataBits": 8,
            "parity": "N",
            "stopBits": 1,
        },
    }


@app.get("/api/ports")
async def list_ports() -> dict:
    return {"ports": serial_manager.list_ports()}


@app.get("/api/status")
async def status() -> dict:
    settings = serial_manager.settings
    return {
        "connected": serial_manager.is_open,
        "settings": settings.__dict__ if settings else None,
    }


@app.post("/api/connect")
async def connect_port(request: ConnectionRequest) -> dict:
    parity_map = {
        "N": serial.PARITY_NONE,
        "E": serial.PARITY_EVEN,
        "O": serial.PARITY_ODD,
    }
    bytesize_map = {
        5: serial.FIVEBITS,
        6: serial.SIXBITS,
        7: serial.SEVENBITS,
        8: serial.EIGHTBITS,
    }
    stopbits_map = {
        1: serial.STOPBITS_ONE,
        1.5: serial.STOPBITS_ONE_POINT_FIVE,
        2: serial.STOPBITS_TWO,
    }

    try:
        settings = SerialSettings(
            port=request.port,
            baudrate=request.baudrate,
            bytesize=bytesize_map[request.data_bits],
            parity=parity_map[request.parity],
            stopbits=stopbits_map[request.stop_bits],
        )
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=f"Unsupported serial setting: {exc}") from exc

    try:
        await serial_manager.open(settings)
    except Exception as exc:
        raise HTTPException(
            status_code=400,
            detail=friendly_serial_error_message(exc, request.port),
        ) from exc

    return {"ok": True}


@app.post("/api/disconnect")
async def disconnect_port() -> dict:
    await serial_manager.close()
    return {"ok": True}


@app.post("/api/command")
async def send_command(request: CommandRequest) -> dict:
    try:
        payload = build_payload(request.module, request.cmd, request.payload)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    flags = FLAG_NEED_ACK if request.need_ack else 0
    frame = Frame(
        flags=flags,
        seq=seq_gen.next(),
        module=request.module,
        cmd=request.cmd,
        payload=payload,
        crc=0,
    )
    try:
        await serial_manager.send_frame(frame)
    except RuntimeError as exc:
        raise HTTPException(
            status_code=400,
            detail="串口还没有打开，请先连接下位机并打开串口后再发送命令。",
        ) from exc
    return {"ok": True, "frame": frame.to_dict()}


@app.post("/api/command-preview")
async def preview_command(request: CommandPreviewRequest) -> dict:
    try:
        payload = build_payload(request.module, request.cmd, request.payload)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    flags = FLAG_NEED_ACK if request.need_ack else 0
    frame = Frame(
        flags=flags,
        seq=seq_gen.peek(),
        module=request.module,
        cmd=request.cmd,
        payload=payload,
        crc=0,
    )
    return {"ok": True, "frame": frame.to_dict()}


@app.post("/api/raw-send")
async def raw_send(request: RawSendRequest) -> dict:
    if request.mode == "hex":
        cleaned = "".join(request.data.split())
        try:
            raw = bytes.fromhex(cleaned)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=f"Invalid hex input: {exc}") from exc
    elif request.mode == "text":
        raw = request.data.encode("utf-8")
    else:
        raise HTTPException(status_code=400, detail="mode must be 'hex' or 'text'")

    try:
        await serial_manager.send_raw(raw, display_mode=request.mode)
    except RuntimeError as exc:
        raise HTTPException(
            status_code=400,
            detail="串口还没有打开，请先连接下位机并打开串口后再发送原始数据。",
        ) from exc
    return {"ok": True}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await ws_manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
