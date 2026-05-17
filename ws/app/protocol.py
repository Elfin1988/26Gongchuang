from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

SOF1 = 0xAA
SOF2 = 0x55

FLAG_NEED_ACK = 0x01
FLAG_IS_RESPONSE = 0x02
FLAG_IS_ERROR = 0x04
FLAG_IS_EVENT = 0x08


MODULES: Dict[int, str] = {
    0x00: "SYSTEM",
    0x10: "CHASSIS",
    0x20: "STEPPER",
    0x30: "SERVO",
    0x40: "VACUUM",
    0x50: "FRICTION",
    0x60: "CONVEYOR",
}


def get_module_name(module: int) -> str:
    return MODULES.get(module, f"0x{module:02X}")

RESULT_CODES: Dict[int, str] = {
    0x00: "OK",
    0x01: "BAD_LEN",
    0x02: "BAD_PARAM",
    0x03: "BUSY",
    0x04: "TIMEOUT",
    0x05: "UNSUPPORTED",
    0x06: "CRC_ERROR",
    0x07: "ESTOP_ACTIVE",
    0x08: "DEVICE_FAULT",
}


def get_result_name(code: int) -> str:
    return RESULT_CODES.get(code, f"0x{code:02X}")


RESULT_DESCRIPTIONS: Dict[int, str] = {
    0x00: "下位机已接收并成功执行该命令。",
    0x01: "载荷长度不符合这条命令的协议要求。",
    0x02: "命令参数超出允许范围，或参数组合不合法。",
    0x03: "目标模块当前正忙，暂时无法执行这条命令。",
    0x04: "下位机执行该命令超时。",
    0x05: "下位机当前固件不支持这条命令。",
    0x06: "下位机检测到校验错误，通常说明通信数据有损坏。",
    0x07: "急停仍处于激活状态，当前命令被拒绝执行。",
    0x08: "目标设备或模块出现故障，命令无法正常完成。",
}


def get_result_description(code: int) -> str:
    return RESULT_DESCRIPTIONS.get(code, "这是一个未在当前协议文档中登记的结果码。")


COMMANDS: Dict[Tuple[int, int], Dict[str, Any]] = {
    (0x00, 0x01): {
        "name": "PING",
        "request_format": "",
    },
    (0x00, 0x03): {
        "name": "ESTOP",
        "request_format": "",
    },
    (0x00, 0x04): {
        "name": "CLEAR_ESTOP",
        "request_format": "",
    },
    (0x10, 0x01): {
        "name": "SET_VELOCITY",
        "request_format": "<hhhH",
        "request_fields": ["vx_mm_s", "vy_mm_s", "wz_dps_x10", "timeout_ms"],
    },
    (0x10, 0x02): {
        "name": "STOP",
        "request_format": "",
    },
    (0x10, 0x03): {
        "name": "SET_CONTROL_MODE",
        "request_format": "<B",
        "request_fields": ["mode"],
    },
    (0x10, 0x04): {
        "name": "SET_WHEEL_TARGET",
        "request_format": "<hhhhH",
        "request_fields": ["fl_target_rpm", "fr_target_rpm", "rl_target_rpm", "rr_target_rpm", "timeout_ms"],
    },
    (0x10, 0x05): {
        "name": "SET_PID",
        "request_format": "<BHHH",
        "request_fields": ["wheel_mask", "kp_x1000", "ki_x1000", "kd_x1000"],
    },
    (0x10, 0x06): {
        "name": "SET_WHEEL_TRIM",
        "request_format": "<Bhhhh",
        "request_fields": ["trim_mode", "fl_value", "fr_value", "rl_value", "rr_value"],
    },
    (0x10, 0x81): {
        "name": "EVENT_RPM_REPORT",
        "event_format": "<HHHH",
        "event_fields": ["fl_rpm", "fr_rpm", "rl_rpm", "rr_rpm"],
    },
    (0x10, 0x82): {
        "name": "EVENT_CLOSED_LOOP_REPORT",
        "event_format": "<hhhhhhhhBBBBBB",
        "event_fields": [
            "fl_target_rpm",
            "fr_target_rpm",
            "rl_target_rpm",
            "rr_target_rpm",
            "fl_actual_rpm",
            "fr_actual_rpm",
            "rl_actual_rpm",
            "rr_actual_rpm",
            "fl_pwm",
            "fr_pwm",
            "rl_pwm",
            "rr_pwm",
            "control_mode",
            "status_flags",
        ],
    },
    (0x20, 0x01): {
        "name": "JOG",
        "request_format": "<BBHH",
        "request_fields": ["axis_mask", "direction", "speed_sps", "accel_sps2"],
    },
    (0x20, 0x02): {
        "name": "STOP",
        "request_format": "<B",
        "request_fields": ["axis_mask"],
    },
    (0x30, 0x01): {
        "name": "SET_ANGLE",
        "request_format": "<BhH",
        "request_fields": ["servo_id", "angle_deg_x10", "duration_ms"],
    },
    (0x40, 0x01): {
        "name": "SET_OUTPUT",
        "request_format": "<BBH",
        "request_fields": ["pump_on", "valve_on", "timeout_ms"],
    },
    (0x40, 0x02): {
        "name": "STOP",
        "request_format": "",
    },
    (0x50, 0x01): {
        "name": "SET_OUTPUT",
        "request_format": "<BBB",
        "request_fields": ["motor_mask", "direction", "pwm_command_percent"],
    },
    (0x50, 0x02): {
        "name": "STOP",
        "request_format": "",
    },
    (0x60, 0x01): {
        "name": "SET_OUTPUT",
        "request_format": "<BBH",
        "request_fields": ["direction", "pwm_command_percent", "timeout_ms"],
    },
    (0x60, 0x02): {
        "name": "STOP",
        "request_format": "",
    },
}


def get_command_name(module: int, cmd: int) -> str:
    spec = COMMANDS.get((module, cmd))
    return spec["name"] if spec else f"0x{cmd:02X}"


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


@dataclass
class Frame:
    flags: int
    seq: int
    module: int
    cmd: int
    payload: bytes
    crc: int

    @property
    def need_ack(self) -> bool:
        return bool(self.flags & FLAG_NEED_ACK)

    @property
    def is_response(self) -> bool:
        return bool(self.flags & FLAG_IS_RESPONSE)

    @property
    def is_error(self) -> bool:
        return bool(self.flags & FLAG_IS_ERROR)

    @property
    def is_event(self) -> bool:
        return bool(self.flags & FLAG_IS_EVENT)

    @property
    def module_name(self) -> str:
        return get_module_name(self.module)

    @property
    def command_name(self) -> str:
        return get_command_name(self.module, self.cmd)

    @property
    def result_code(self) -> Optional[int]:
        if self.is_response and self.payload:
            return self.payload[0]
        return None

    @property
    def result_name(self) -> Optional[str]:
        code = self.result_code
        if code is None:
            return None
        return get_result_name(code)

    def to_bytes(self) -> bytes:
        header = bytes(
            [
                SOF1,
                SOF2,
                self.flags & 0xFF,
                self.seq & 0xFF,
                self.module & 0xFF,
                self.cmd & 0xFF,
                len(self.payload) & 0xFF,
            ]
        )
        crc_input = header[2:] + self.payload
        crc = crc16_modbus(crc_input)
        return header + self.payload + struct.pack("<H", crc)

    def to_dict(self) -> dict:
        return {
            "flags": self.flags,
            "seq": self.seq,
            "module": self.module,
            "module_name": self.module_name,
            "cmd": self.cmd,
            "command_name": self.command_name,
            "payload_hex": self.payload.hex(" ").upper(),
            "payload_len": len(self.payload),
            "need_ack": self.need_ack,
            "is_response": self.is_response,
            "is_error": self.is_error,
            "is_event": self.is_event,
            "result_code": self.result_code,
            "result_name": self.result_name,
            "frame_hex": self.to_bytes().hex(" ").upper(),
        }


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[Frame]:
        self._buffer.extend(chunk)
        frames: List[Frame] = []

        while True:
            if len(self._buffer) < 9:
                break

            sof_index = self._buffer.find(bytes([SOF1, SOF2]))
            if sof_index < 0:
                self._buffer.clear()
                break
            if sof_index > 0:
                del self._buffer[:sof_index]
            if len(self._buffer) < 9:
                break

            payload_len = self._buffer[6]
            frame_len = 2 + 5 + payload_len + 2
            if len(self._buffer) < frame_len:
                break

            raw_frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]

            flags = raw_frame[2]
            seq = raw_frame[3]
            module = raw_frame[4]
            cmd = raw_frame[5]
            payload = raw_frame[7 : 7 + payload_len]
            recv_crc = struct.unpack("<H", raw_frame[-2:])[0]
            calc_crc = crc16_modbus(raw_frame[2:-2])
            if recv_crc != calc_crc:
                continue

            frames.append(
                Frame(
                    flags=flags,
                    seq=seq,
                    module=module,
                    cmd=cmd,
                    payload=payload,
                    crc=recv_crc,
                )
            )

        return frames


class SequenceGenerator:
    def __init__(self) -> None:
        self._seq = 0

    def peek(self) -> int:
        return self._seq

    def next(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq


def build_payload(module: int, cmd: int, values: dict) -> bytes:
    spec = COMMANDS.get((module, cmd))
    if not spec:
        raise ValueError(f"Unsupported command: module=0x{module:02X}, cmd=0x{cmd:02X}")

    fmt = spec.get("request_format", "")
    if not fmt:
        return b""

    field_names = spec.get("request_fields", [])
    try:
        packed_values = [values[name] for name in field_names]
    except KeyError as exc:
        raise ValueError(f"Missing field: {exc.args[0]}") from exc
    return struct.pack(fmt, *packed_values)


def decode_payload(frame: Frame) -> dict:
    spec = COMMANDS.get((frame.module, frame.cmd))
    decoded = {}
    if not spec:
        return decoded

    if frame.is_event:
        fmt = spec.get("event_format", "")
        fields = spec.get("event_fields", [])
        if fmt and fields and frame.payload:
            try:
                values = struct.unpack(fmt, frame.payload)
                for name, value in zip(fields, values):
                    decoded[name] = value
            except struct.error:
                decoded["decode_error"] = "payload_length_mismatch"
                return decoded

        if frame.module == 0x10 and frame.cmd == 0x81:
            decoded["summary"] = (
                f"底盘四轮转速上报: FL {decoded.get('fl_rpm', 0)} rpm, "
                f"FR {decoded.get('fr_rpm', 0)} rpm, "
                f"RL {decoded.get('rl_rpm', 0)} rpm, "
                f"RR {decoded.get('rr_rpm', 0)} rpm"
            )
            decoded["event_name"] = "CHASSIS_RPM_REPORT"
        elif frame.module == 0x10 and frame.cmd == 0x82:
            mode_name = "closed_loop" if decoded.get("control_mode", 0) == 1 else "open_loop"
            decoded["summary"] = (
                f"closed-loop report: mode={mode_name}, "
                f"FL {decoded.get('fl_actual_rpm', 0)}/{decoded.get('fl_target_rpm', 0)} rpm, "
                f"FR {decoded.get('fr_actual_rpm', 0)}/{decoded.get('fr_target_rpm', 0)} rpm, "
                f"RL {decoded.get('rl_actual_rpm', 0)}/{decoded.get('rl_target_rpm', 0)} rpm, "
                f"RR {decoded.get('rr_actual_rpm', 0)}/{decoded.get('rr_target_rpm', 0)} rpm"
            )
            decoded["event_name"] = "CHASSIS_CLOSED_LOOP_REPORT"
        return decoded

    if frame.is_response:
        if len(frame.payload) >= 3:
            result, request_module, request_cmd = struct.unpack("<BBB", frame.payload[:3])
            decoded["result"] = result
            decoded["result_name"] = get_result_name(result)
            decoded["result_description"] = get_result_description(result)
            decoded["request_module"] = request_module
            decoded["request_module_name"] = get_module_name(request_module)
            decoded["request_cmd"] = request_cmd
            decoded["request_cmd_name"] = get_command_name(request_module, request_cmd)
            decoded["summary"] = (
                f"下位机返回了统一格式应答，结果为 {decoded['result_name']}，"
                f"对应的原始请求是 {decoded['request_module_name']} / {decoded['request_cmd_name']}。"
            )
            decoded["response_format"] = "unified_ack_v1"
            if len(frame.payload) > 3:
                decoded["extra_payload_hex"] = frame.payload[3:].hex(" ").upper()
        elif frame.payload:
            decoded["result"] = frame.payload[0]
            decoded["result_name"] = get_result_name(frame.payload[0])
            decoded["result_description"] = get_result_description(frame.payload[0])
            decoded["summary"] = (
                f"下位机返回了旧格式应答，当前载荷里只有结果码 {decoded['result_name']}。"
                f"这类应答没有把 request_module / request_cmd 放进载荷，"
                f"因此需要结合帧头中的 MODULE / CMD 来判断它是在回复哪条命令。"
            )
            decoded["response_format"] = "legacy_result_only"
            decoded["request_module"] = frame.module
            decoded["request_module_name"] = get_module_name(frame.module)
            decoded["request_cmd"] = frame.cmd
            decoded["request_cmd_name"] = get_command_name(frame.module, frame.cmd)
            if len(frame.payload) > 1:
                decoded["extra_payload_hex"] = frame.payload[1:].hex(" ").upper()
        return decoded

    fmt = spec.get("request_format", "")
    fields = spec.get("request_fields", [])
    if fmt and fields and frame.payload:
        try:
            values = struct.unpack(fmt, frame.payload)
            for name, value in zip(fields, values):
                decoded[name] = value
        except struct.error:
            decoded["decode_error"] = "payload_length_mismatch"

    if frame.module == 0x10 and frame.cmd == 0x01:
        decoded["summary"] = (
            f"底盘: 前进系数约 {decoded.get('vx_mm_s', 0) / 500:.2f}, "
            f"横移系数约 {decoded.get('vy_mm_s', 0) / 500:.2f}, "
            f"旋转系数约 {decoded.get('wz_dps_x10', 0) / 900:.2f}, "
            f"运行时长 {decoded.get('timeout_ms', 0)} ms"
        )
    elif frame.module == 0x10 and frame.cmd == 0x03:
        decoded["summary"] = (
            "chassis control mode: "
            f"{'closed_loop' if decoded.get('mode', 0) == 1 else 'open_loop'}"
        )
    elif frame.module == 0x10 and frame.cmd == 0x04:
        decoded["summary"] = (
            "wheel targets: "
            f"FL {decoded.get('fl_target_rpm', 0)} rpm, "
            f"FR {decoded.get('fr_target_rpm', 0)} rpm, "
            f"RL {decoded.get('rl_target_rpm', 0)} rpm, "
            f"RR {decoded.get('rr_target_rpm', 0)} rpm, "
            f"timeout {decoded.get('timeout_ms', 0)} ms"
        )
    elif frame.module == 0x10 and frame.cmd == 0x05:
        decoded["summary"] = (
            "wheel PI params: "
            f"mask 0x{decoded.get('wheel_mask', 0):02X}, "
            f"kp={decoded.get('kp_x1000', 0) / 1000:.3f}, "
            f"ki={decoded.get('ki_x1000', 0) / 1000:.3f}, "
            f"kd={decoded.get('kd_x1000', 0) / 1000:.3f}"
        )
    elif frame.module == 0x10 and frame.cmd == 0x06:
        trim_mode = decoded.get("trim_mode", 0)
        if trim_mode == 1:
            decoded["summary"] = (
                "wheel trim offset: "
                f"FL {decoded.get('fl_value', 0)}, "
                f"FR {decoded.get('fr_value', 0)}, "
                f"RL {decoded.get('rl_value', 0)}, "
                f"RR {decoded.get('rr_value', 0)}"
            )
        else:
            decoded["summary"] = (
                "wheel trim scale: "
                f"FL {decoded.get('fl_value', 100) / 100:.2f}, "
                f"FR {decoded.get('fr_value', 100) / 100:.2f}, "
                f"RL {decoded.get('rl_value', 100) / 100:.2f}, "
                f"RR {decoded.get('rr_value', 100) / 100:.2f}"
            )
    elif frame.module == 0x20 and frame.cmd == 0x01:
        decoded["summary"] = (
            f"步进: 轴掩码 {decoded.get('axis_mask', 0)}, 方向 {decoded.get('direction', 0)}, "
            f"速度 {decoded.get('speed_sps', 0)} steps/s, 加速度 {decoded.get('accel_sps2', 0)} steps/s²"
        )
    elif frame.module == 0x20 and frame.cmd == 0x02:
        decoded["summary"] = f"步进: 停止轴掩码 {decoded.get('axis_mask', 0)}"
    elif frame.module == 0x30 and frame.cmd == 0x01:
        decoded["summary"] = (
            f"舵机: 编号 {decoded.get('servo_id', 0)}, "
            f"角度 {decoded.get('angle_deg_x10', 0) / 10:.1f}°, "
            f"运动时长 {decoded.get('duration_ms', 0)} ms"
        )
    elif frame.module == 0x40 and frame.cmd == 0x01:
        decoded["summary"] = (
            f"抽气: 气泵 {'开' if decoded.get('pump_on', 0) else '关'}, "
            f"电磁阀 {'开' if decoded.get('valve_on', 0) else '关'}, "
            f"超时 {decoded.get('timeout_ms', 0)} ms"
        )
    elif frame.module == 0x50 and frame.cmd == 0x01:
        decoded["summary"] = (
            f"摩擦带: 方向 {decoded.get('direction', 0)}, "
            f"速度系数约 {decoded.get('pwm_command_percent', 0) / 100:.2f}"
        )
    elif frame.module == 0x50 and frame.cmd == 0x02:
        decoded["summary"] = "摩擦带: 停止"
    elif frame.module == 0x60 and frame.cmd == 0x01:
        decoded["summary"] = (
            f"运送电机: 方向 {decoded.get('direction', 0)}, "
            f"速度系数约 {decoded.get('pwm_command_percent', 0) / 100:.2f}, "
            f"超时 {decoded.get('timeout_ms', 0)} ms"
        )
    elif frame.module == 0x60 and frame.cmd == 0x02:
        decoded["summary"] = "运送电机: 停止"
    elif frame.module == 0x00 and frame.cmd == 0x01:
        decoded["summary"] = "系统: PING"
    elif frame.module == 0x00 and frame.cmd == 0x03:
        decoded["summary"] = "系统: 急停"
    elif frame.module == 0x00 and frame.cmd == 0x04:
        decoded["summary"] = "系统: 解除急停"

    return decoded
