from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Dict, List, Optional

import serial
from serial import SerialException
from serial.tools import list_ports

from .protocol import Frame, FrameParser, decode_payload


def utc_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def unix_timestamp_ms() -> int:
    return int(datetime.now().timestamp() * 1000)


@dataclass
class SerialSettings:
    port: str
    baudrate: int = 115200
    bytesize: int = serial.EIGHTBITS
    parity: str = serial.PARITY_NONE
    stopbits: float = serial.STOPBITS_ONE


class SerialManager:
    def __init__(self, event_callback: Callable[[dict], None]) -> None:
        self._event_callback = event_callback
        self._serial: Optional[serial.Serial] = None
        self._reader_task: Optional[asyncio.Task] = None
        self._parser = FrameParser()
        self._lock = asyncio.Lock()
        self._settings: Optional[SerialSettings] = None

    @staticmethod
    def list_ports() -> List[Dict]:
        ports: List[Dict] = []
        for port in list_ports.comports():
            ports.append(
                {
                    "device": port.device,
                    "name": port.name,
                    "description": port.description,
                    "hwid": port.hwid,
                    "vid": port.vid,
                    "pid": port.pid,
                    "serial_number": port.serial_number,
                    "manufacturer": port.manufacturer,
                    "product": port.product,
                    "interface": port.interface,
                }
            )
        return ports

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def settings(self) -> Optional[SerialSettings]:
        return self._settings

    async def open(self, settings: SerialSettings) -> None:
        async with self._lock:
            await self.close()
            self._serial = serial.Serial(
                port=settings.port,
                baudrate=settings.baudrate,
                bytesize=settings.bytesize,
                parity=settings.parity,
                stopbits=settings.stopbits,
                timeout=0.05,
            )
            self._settings = settings
            self._reader_task = asyncio.create_task(self._reader_loop())
            self._emit(
                {
                    "type": "connection",
                    "timestamp": utc_timestamp(),
                    "connected": True,
                    "settings": settings.__dict__,
                }
            )

    async def close(self) -> None:
        current_task = asyncio.current_task()
        reader_task = self._reader_task
        self._reader_task = None
        if reader_task and reader_task is not current_task:
            reader_task.cancel()
            try:
                await reader_task
            except asyncio.CancelledError:
                pass
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._emit(
                {
                    "type": "connection",
                    "timestamp": utc_timestamp(),
                    "connected": False,
                    "settings": self._settings.__dict__ if self._settings else None,
                }
            )
        self._serial = None
        self._settings = None

    async def send_raw(self, data: bytes, display_mode: str = "hex") -> None:
        async with self._lock:
            if not self.is_open or not self._serial:
                raise RuntimeError("Serial port is not open")
            self._serial.write(data)
        self._emit(
            {
                "type": "raw_tx",
                "timestamp": utc_timestamp(),
                "display_mode": display_mode,
                "hex": data.hex(" ").upper(),
                "text": data.decode("utf-8", errors="replace"),
            }
        )

    async def send_frame(self, frame: Frame) -> None:
        raw = frame.to_bytes()
        await self.send_raw(raw, display_mode="protocol")
        frame_dict = frame.to_dict()
        frame_dict["decoded"] = decode_payload(frame)
        self._emit(
            {
                "type": "protocol_tx",
                "timestamp": utc_timestamp(),
                "frame": frame_dict,
            }
        )

    async def _reader_loop(self) -> None:
        assert self._serial is not None
        loop = asyncio.get_running_loop()
        try:
            while True:
                chunk = await loop.run_in_executor(None, self._serial.read, 256)
                if not chunk:
                    await asyncio.sleep(0.01)
                    continue

                self._emit(
                    {
                        "type": "raw_rx",
                        "timestamp": utc_timestamp(),
                        "hex": chunk.hex(" ").upper(),
                        "text": chunk.decode("utf-8", errors="replace"),
                    }
                )

                frames = self._parser.feed(chunk)
                for frame in frames:
                    frame_dict = frame.to_dict()
                    frame_dict["decoded"] = decode_payload(frame)
                    self._emit(
                        {
                            "type": "protocol_rx",
                            "timestamp": utc_timestamp(),
                            "frame": frame_dict,
                        }
                    )
                    if frame.is_response:
                        self._emit(
                            {
                                "type": "response_rx",
                                "timestamp": utc_timestamp(),
                                "frame": frame_dict,
                            }
                        )
                    elif frame.is_event:
                        event_payload = {
                            "module": frame.module,
                            "module_name": frame.module_name,
                            "cmd": frame.cmd,
                            "command_name": frame.command_name,
                            "seq": frame.seq,
                            "payload_hex": frame.payload.hex(" ").upper(),
                            "decoded": frame_dict.get("decoded", {}),
                        }
                        self._emit(
                            {
                                "type": "event_rx",
                                "timestamp": utc_timestamp(),
                                "event": event_payload,
                            }
                        )

                        decoded = frame_dict.get("decoded", {})
                        if frame.module == 0x10 and frame.cmd == 0x81 and not decoded.get("decode_error"):
                            self._emit(
                                {
                                    "type": "chassis_rpm_event",
                                    "timestamp": utc_timestamp(),
                                    "sample": {
                                        "t": utc_timestamp(),
                                        "unix_ms": unix_timestamp_ms(),
                                        "fl": decoded.get("fl_rpm", 0),
                                        "fr": decoded.get("fr_rpm", 0),
                                        "rl": decoded.get("rl_rpm", 0),
                                        "rr": decoded.get("rr_rpm", 0),
                                    },
                                }
                            )
                        elif frame.module == 0x10 and frame.cmd == 0x82 and not decoded.get("decode_error"):
                            self._emit(
                                {
                                    "type": "chassis_closed_loop_event",
                                    "timestamp": utc_timestamp(),
                                    "sample": {
                                        "t": utc_timestamp(),
                                        "unix_ms": unix_timestamp_ms(),
                                        "mode": decoded.get("control_mode", 0),
                                        "status_flags": decoded.get("status_flags", 0),
                                        "fl_target_rpm": decoded.get("fl_target_rpm", 0),
                                        "fr_target_rpm": decoded.get("fr_target_rpm", 0),
                                        "rl_target_rpm": decoded.get("rl_target_rpm", 0),
                                        "rr_target_rpm": decoded.get("rr_target_rpm", 0),
                                        "fl_actual_rpm": decoded.get("fl_actual_rpm", 0),
                                        "fr_actual_rpm": decoded.get("fr_actual_rpm", 0),
                                        "rl_actual_rpm": decoded.get("rl_actual_rpm", 0),
                                        "rr_actual_rpm": decoded.get("rr_actual_rpm", 0),
                                        "fl_pwm": decoded.get("fl_pwm", 0),
                                        "fr_pwm": decoded.get("fr_pwm", 0),
                                        "rl_pwm": decoded.get("rl_pwm", 0),
                                        "rr_pwm": decoded.get("rr_pwm", 0),
                                    },
                                }
                            )
        except asyncio.CancelledError:
            raise
        except SerialException as exc:
            self._emit(
                {
                    "type": "error",
                    "timestamp": utc_timestamp(),
                    "message": f"Serial error: {exc}",
                }
            )
            await self._handle_reader_failure()
        except Exception as exc:  # pragma: no cover
            self._emit(
                {
                    "type": "error",
                    "timestamp": utc_timestamp(),
                    "message": f"Unexpected reader error: {exc}",
                }
            )
            await self._handle_reader_failure()

    def _emit(self, event: dict) -> None:
        self._event_callback(event)

    async def _handle_reader_failure(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._reader_task = None
        self._emit(
            {
                "type": "connection",
                "timestamp": utc_timestamp(),
                "connected": False,
                "settings": self._settings.__dict__ if self._settings else None,
            }
        )
        self._settings = None
