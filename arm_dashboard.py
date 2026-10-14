#!/usr/bin/env python3
"""Local IM6014 3-joint arm dashboard server.

Binds exclusively to 127.0.0.1. Starts disconnected; no USB port opened on startup.
Serves offline dashboard UI and proxies authenticated actions to ArmManualSession.
"""

from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
from pathlib import Path
import re
import socket
import sys
import threading
import time
from typing import Any


try:
    from arm_manual import ArmManualSession
except ImportError:
    ArmManualSession = None

from im6014_port import (ADAPTER_PROFILES, DEFAULT_ADAPTER_PROFILE,
                         default_port, enumerate_ports)

ROOT = Path(__file__).resolve().parent
ARM_WEB_DIR = ROOT / "arm_web"
ARM_ASSETS_DIR = ROOT / "arm_assets"
MAX_BODY_BYTES = 16384
MAX_CONCURRENT_REQUESTS = 16
HTTP_READ_TIMEOUT_S = 2.0
CONTENT_LENGTH_RE = re.compile(r"[0-9]+\Z")
MESH_PATH_RE = re.compile(r"meshes/[A-Za-z0-9._-]+\.(?:bin|stl)\Z")
WEB_STATIC_FILES = frozenset({
    "index.html",
    "style.css",
    "dashboard.js",
    "arm_scene.js",
    "input_reducer.js",
    "presentation.js",
    "presets.js",
    "vendor/three.module.js",
    "vendor/LICENSE.three",
})
_cached_ports: list[dict[str, Any]] = []
_cached_ports_time: float = 0.0
_cached_ports_error: str | None = None
TOP_LEVEL_FIELDS = frozenset({
    "physical_enabled", "default_port", "default_ports", "adapter_profile", "adapter_profiles",
    "ports", "port_error", "state", "reason", "connected", "demo", "profile_id", "profiles", "calibrated", "armed", "stop_confirmed",
    "feedback_age_s", "settings", "joints", "latest", "latest_feedback", "events",
})
SETTING_FIELDS = frozenset({
    "kp", "kd", "max_rpm", "effort_limit_nm", "max_effort_nm", "lower_deg", "upper_deg",
    "direction", "known_angle_deg", "source", "units",
})
JOINT_FIELDS = frozenset({
    "name", "id", "source", "configured", "calibrated", "position_rad", "velocity_rad_s", "torque_nm", "mode", "timeout",
    "voltage", "driver_temp", "winding_temp", "errors", "warnings", "state", "status",
    "feedback_age_s", "age_s", "health", "stop_confirmed", "stop_result", "stop_error", "protection_fault", "settings", "latest",
})
LATEST_FIELDS = JOINT_FIELDS | frozenset({"time_s", "sequence", "measured_at_s"})
EVENT_FIELDS = frozenset({"time_s", "kind", "severity", "action", "status"})
PORT_FIELDS = frozenset({"device", "path", "label", "description", "kind"})
SECRET_ASSIGNMENT_RE = re.compile(r"(?i)\b(session|ticket|owner|csrf|authorization|access[_ -]?token)\b\s*[:=]\s*[^\s,;]+")


def _is_safe_child_path(child_path: Path, parent_dir: Path) -> bool:
    """Verifies that child_path is resolved strictly inside parent_dir."""
    try:
        child_resolved = child_path.resolve()
        parent_resolved = parent_dir.resolve()
        child_resolved.relative_to(parent_resolved)
        return child_resolved.is_file()
    except (ValueError, RuntimeError):
        return False


def _check_finite_number(val: Any) -> bool:
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        return False
    try:
        return math.isfinite(float(val))
    except (OverflowError, TypeError, ValueError):
        return False


def _finite_json(value: Any) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, (int, float)):
        return _check_finite_number(value)
    if isinstance(value, list):
        return all(_finite_json(item) for item in value)
    if isinstance(value, dict):
        return all(isinstance(key, str) and _finite_json(item) for key, item in value.items())
    return False


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"Invalid JSON numeric constant: {value}")


def _safe_text(value: Any, limit: int = 256) -> str | None:
    if not isinstance(value, str):
        return None
    text = value[:limit]
    return SECRET_ASSIGNMENT_RE.sub(r"\1=[redacted]", text)


def _safe_scalar(value: Any) -> Any:
    if isinstance(value, str):
        return _safe_text(value)
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value if _check_finite_number(value) else None
    return None


def _project_fields(data: Any, fields: frozenset[str]) -> dict[str, Any] | None:
    if not isinstance(data, dict):
        return None
    projected: dict[str, Any] = {}
    for key in fields:
        if key not in data:
            continue
        value = data[key]
        if key == "settings":
            nested = _project_fields(value, SETTING_FIELDS)
            if nested is not None:
                projected[key] = nested
        elif key == "joints":
            if isinstance(value, list):
                projected[key] = [joint for item in value[:8] if (joint := _project_fields(item, JOINT_FIELDS)) is not None]
        elif key == "profiles":
            if isinstance(value, list):
                profiles_list = []
                for item in value[:8]:
                    if isinstance(item, dict):
                        p_id = _safe_text(item.get("id"), 64)
                        joints_val = item.get("joints")
                        if p_id is not None and isinstance(joints_val, list):
                            safe_joints = [j for j in (_safe_text(name, 16) for name in joints_val[:8]) if j is not None]
                            profiles_list.append({"id": p_id, "joints": safe_joints})
                projected[key] = profiles_list
        elif key == "adapter_profiles":
            if isinstance(value, list):
                profiles_list = []
                for item in value[:8]:
                    if isinstance(item, dict):
                        profile_id = _safe_text(item.get("id"), 32)
                        label = _safe_text(item.get("label"), 64)
                        if profile_id and label:
                            profiles_list.append({"id": profile_id, "label": label})
                projected[key] = profiles_list
        elif key == "default_ports":
            if isinstance(value, dict):
                defaults = {}
                for profile_id in (profile.id for profile in ADAPTER_PROFILES):
                    path = _safe_text(value.get(profile_id), 256)
                    defaults[profile_id] = path or ""
                projected[key] = defaults
        elif key in ("latest", "latest_feedback"):
            if isinstance(value, list):
                projected[key] = [item for entry in value[:16] if (item := _project_fields(entry, LATEST_FIELDS)) is not None]
            else:
                nested = _project_fields(value, LATEST_FIELDS)
                if nested is not None:
                    projected[key] = nested
        elif key == "events":
            if isinstance(value, list):
                events: list[Any] = []
                for entry in value[-100:]:
                    if isinstance(entry, str):
                        safe = _safe_text(entry, 256)
                        if safe is not None:
                            events.append(safe)
                    elif isinstance(entry, dict):
                        event = _project_fields(entry, EVENT_FIELDS)
                        if event is not None:
                            events.append(event)
                projected[key] = events
        elif key == "ports":
            if isinstance(value, list):
                ports: list[dict[str, Any]] = []
                for item in value[:32]:
                    if not isinstance(item, dict):
                        continue
                    port: dict[str, Any] = {}
                    for field in PORT_FIELDS:
                        field_value = item.get(field)
                        if field == "kind" and field_value in (
                                "ft232h-candidate", "legacy-ft4232h", "unknown", "unsupported-channel"):
                            port[field] = field_value
                        elif field in ("device", "path", "label", "description"):
                            safe = _safe_text(field_value, 256 if field != "description" else 160)
                            if safe is not None:
                                port[field] = safe
                    ports.append(port)
                projected[key] = ports
        elif key in ("errors", "warnings"):
            if type(value) is int and 0 <= value <= (0xFFFFFFFF if key == "errors" else 7):
                projected[key] = value
            elif isinstance(value, list):
                projected[key] = [text for item in value[:32] if (text := _safe_text(item, 160)) is not None]
        else:
            safe = _safe_scalar(value)
            if safe is not None or value is None:
                projected[key] = safe
    return projected


def _declared_arm_assets() -> frozenset[str]:
    declared = {"model.json", "arm.urdf"}
    try:
        model = json.loads((ARM_ASSETS_DIR / "model.json").read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
    except (OSError, ValueError, UnicodeDecodeError):
        return frozenset(declared)

    bodies = model.get("bodies", {}) if isinstance(model, dict) else {}
    if isinstance(bodies, dict):
        for materials in bodies.values():
            if isinstance(materials, dict):
                for info in materials.values():
                    path = info.get("file") if isinstance(info, dict) else None
                    if isinstance(path, str) and MESH_PATH_RE.fullmatch(path):
                        declared.add(path)
    portable = model.get("portable_meshes", {}) if isinstance(model, dict) else {}
    if isinstance(portable, dict):
        for name, info in portable.items():
            path = info.get("file", name) if isinstance(info, dict) else name
            if isinstance(path, str) and MESH_PATH_RE.fullmatch(path):
                declared.add(path)
    return frozenset(declared)


ARM_ASSET_FILES = _declared_arm_assets()


class ArmDashboardHandler(BaseHTTPRequestHandler):
    """Secure HTTP handler for arm workbench dashboard."""

    server: ArmDashboardServer

    def setup(self) -> None:
        super().setup()
        if self.connection is not None:
            try:
                self.connection.settimeout(HTTP_READ_TIMEOUT_S)
            except (AttributeError, OSError):
                pass

    def log_message(self, format: str, *args: Any) -> None:

        pass

    def _send_json(self, status: int, data: dict[str, Any]) -> None:
        body = json.dumps(data, allow_nan=False, ensure_ascii=False).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            if self.close_connection:
                self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, socket.timeout, OSError):
            self.close_connection = True

    def _send_error_json(self, status: int, reason: str) -> None:
        self._send_json(status, {"error": True, "reason": _safe_text(str(reason), 200) or "Request failed"})

    def _validate_host(self) -> bool:
        host_values = self.headers.get_all("Host", [])
        if len(host_values) != 1:
            self.close_connection = True
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Exactly one Host header is required")
            return False
        host = host_values[0].strip()
        expected_host = f"127.0.0.1:{self.server.bound_port}"
        if not host:
            self.close_connection = True
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Missing Host header")
            return False
        if host != expected_host:
            self.close_connection = True
            self._send_error_json(HTTPStatus.FORBIDDEN, "Invalid Host header")
            return False
        return True

    def _validate_origin(self) -> bool:
        origins = self.headers.get_all("Origin", [])
        if len(origins) != 1 or not origins[0]:
            self.close_connection = True
            self._send_error_json(HTTPStatus.FORBIDDEN, "Exactly one Origin header is required on POST")
            return False
        origin = origins[0]
        expected = f"http://127.0.0.1:{self.server.bound_port}"
        if origin != expected:
            self.close_connection = True
            self._send_error_json(HTTPStatus.FORBIDDEN, "Mismatched Origin header")
            return False
        return True

    def _read_json_body(self) -> dict[str, Any] | None:
        content_types = self.headers.get_all("Content-Type", [])
        if len(content_types) != 1:
            self.close_connection = True
            self._send_error_json(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "Exactly one Content-Type header is required")
            return None
        content_type = content_types[0]
        media_type = content_type.partition(";")[0].strip().lower()
        if media_type != "application/json":
            self.close_connection = True
            self._send_error_json(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "Content-Type must be application/json")
            return None

        if self.headers.get_all("Transfer-Encoding", []):
            self.close_connection = True
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Transfer-Encoding is not supported")
            return None

        content_lengths = self.headers.get_all("Content-Length", [])
        if len(content_lengths) != 1:
            self.close_connection = True
            self._send_error_json(HTTPStatus.LENGTH_REQUIRED, "Exactly one Content-Length header is required")
            return None

        content_length_text = content_lengths[0].strip()
        if len(content_length_text) > len(str(MAX_BODY_BYTES)) or not CONTENT_LENGTH_RE.fullmatch(content_length_text):
            self.close_connection = True
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid Content-Length")
            return None
        content_length = int(content_length_text)

        if content_length > MAX_BODY_BYTES:
            self.close_connection = True
            self._send_error_json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "Request body exceeds 16KB limit")
            return None

        try:
            raw_body = self.rfile.read(content_length)
            if len(raw_body) != content_length:
                self.close_connection = True
                self._send_error_json(HTTPStatus.BAD_REQUEST, "Truncated request body")
                return None
            data = json.loads(raw_body.decode("utf-8"), parse_constant=_reject_json_constant)
            if not _finite_json(data):
                raise ValueError("Non-finite or out-of-range JSON number")
            if not isinstance(data, dict):
                self._send_error_json(HTTPStatus.BAD_REQUEST, "JSON body must be an object")
                return None
            return data
        except socket.timeout:
            self.close_connection = True
            self._send_error_json(HTTPStatus.REQUEST_TIMEOUT, "Request body read timed out")
            return None
        except (ValueError, UnicodeDecodeError, RecursionError):
            self.close_connection = True
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Malformed or non-finite JSON")
            return None

    def _get_serial_ports(self, force: bool = False) -> list[dict[str, Any]]:
        global _cached_ports, _cached_ports_time, _cached_ports_error
        now = time.monotonic()
        if not force and now - _cached_ports_time < 3.0:
            return [dict(port) for port in _cached_ports]

        ports_list: list[dict[str, Any]] = []
        ports_error = None
        try:
            for record in enumerate_ports():
                ports_list.append({
                    "device": record.device,
                    "path": record.path,
                    "label": record.label,
                    "description": record.description,
                    "kind": record.kind,
                })
        except Exception as error:
            ports_error = f"Port enumeration failed ({type(error).__name__})"
        _cached_ports = ports_list
        _cached_ports_time = now
        _cached_ports_error = ports_error
        return [dict(port) for port in ports_list]

    @staticmethod
    def _default_ports(ports):
        return {profile.id: default_port(ports, profile.id) for profile in ADAPTER_PROFILES}

    def do_GET(self) -> None:
        if not self._validate_host():
            return

        path = self.path.split("?", 1)[0]

        if path == "/api/state":
            self._handle_get_state()
            return
        if path == "/api/ports":
            ports = self._get_serial_ports(force=True)
            defaults = self._default_ports(ports)
            self._send_json(HTTPStatus.OK, {
                "ports": ports,
                "default": defaults[DEFAULT_ADAPTER_PROFILE],
                "default_ports": defaults,
                "adapter_profiles": [{"id": profile.id, "label": profile.label}
                                     for profile in ADAPTER_PROFILES],
                "error": _cached_ports_error,
            })
            return

        self._handle_static_file(path)

    def _handle_get_state(self) -> None:
        """Observational state endpoint. Emits strictly allowlisted telemetry."""
        try:
            snapshot = self.server.session.snapshot()
        except Exception:
            self._send_error_json(HTTPStatus.SERVICE_UNAVAILABLE, "Backend state is unavailable")
            return
        observational = _project_fields(snapshot, TOP_LEVEL_FIELDS) if isinstance(snapshot, dict) else None
        if observational is None:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, "Backend state is unavailable")
            return
        observational["physical_enabled"] = getattr(self.server.session, "physical_opt_in", False) is True
        ports = self._get_serial_ports()
        observational["ports"] = ports
        observational["port_error"] = _cached_ports_error
        defaults = self._default_ports(ports)
        adapter_profile = observational.get("adapter_profile", DEFAULT_ADAPTER_PROFILE)
        if type(adapter_profile) is not str or adapter_profile not in defaults:
            adapter_profile = DEFAULT_ADAPTER_PROFILE
        observational["adapter_profile"] = adapter_profile
        observational["adapter_profiles"] = [
            {"id": profile.id, "label": profile.label} for profile in ADAPTER_PROFILES]
        observational["default_ports"] = defaults
        observational["default_port"] = defaults[adapter_profile]
        self._send_json(HTTPStatus.OK, observational)

    def _handle_static_file(self, raw_path: str) -> None:
        clean_path = raw_path.lstrip("/") or "index.html"
        if "\0" in clean_path or "\\" in clean_path or clean_path.startswith("/"):
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return
        if clean_path in WEB_STATIC_FILES:
            target = ARM_WEB_DIR / clean_path
            base_dir = ARM_WEB_DIR
        elif clean_path.startswith("arm_assets/") and clean_path.removeprefix("arm_assets/") in ARM_ASSET_FILES:
            target = ARM_ASSETS_DIR / clean_path.removeprefix("arm_assets/")
            base_dir = ARM_ASSETS_DIR
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return

        if not _is_safe_child_path(target, base_dir):
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return

        mime_type, _ = mimetypes.guess_type(str(target))
        if target.suffix in (".bin", ".stl"):
            mime_type = "application/octet-stream"
        elif target.suffix in (".js", ".mjs"):
            mime_type = "text/javascript"
        elif mime_type is None:
            mime_type = "application/octet-stream"

        try:
            content = target.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mime_type)
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except OSError:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "Error reading file")

    def do_POST(self) -> None:
        if not self._validate_host():
            return
        if not self._validate_origin():
            return

        path = self.path.split("?", 1)[0]
        data = self._read_json_body()
        if data is None:
            return

        if path == "/api/session":
            self._handle_post_session(data)
        elif path == "/api/session-status":
            self._handle_post_session_status(data)
        elif path == "/api/action":
            self._handle_post_action(data)
        elif path == "/api/receipt":
            self._handle_post_receipt(data)
        elif path == "/api/ticket":
            self._handle_post_ticket(data)
        elif path == "/api/input":
            self._handle_post_input(data)
        elif path == "/api/stop":
            self._handle_post_stop(data)
        elif path == "/api/draft":
            self._handle_post_draft(data)
        elif path == "/api/retire":
            self._handle_post_retire(data)
        else:
            self._send_error_json(HTTPStatus.NOT_FOUND, f"Unknown API endpoint: {path}")

    def _handle_post_draft(self, data):
        try:
            session_val = data.get('session')
            values_val = data.get('values')
            joint_val = data.get('joint', 'J1')
            draft = self.server.session.edit_parameters(session_val, values_val, joint=joint_val)
        except (ValueError, TypeError) as error:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(error))
            return
        self._send_json(HTTPStatus.OK, {"draft": draft, "applied": False})

    def _handle_post_session(self, _data: dict[str, Any]) -> None:
        try:
            session_id = self.server.session.create_session()
        except RuntimeError as error:
            self._send_error_json(HTTPStatus.SERVICE_UNAVAILABLE, str(error))
            return
        self._send_json(HTTPStatus.OK, {"session": session_id})

    def _handle_post_session_status(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        if not isinstance(session, str):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid session")
            return
        try:
            controls = self.server.session.controls_motor(session)
            sequence = self.server.session.input_sequence(session)
            self._send_json(HTTPStatus.OK, {"controls_motor": controls, "last_sequence": sequence})
        except ValueError as error:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(error))

    def _handle_post_action(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        action = data.get("action")
        values = data.get("values", {})
        ticket = data.get("ticket")
        if (not isinstance(session, str) or not isinstance(action, str) or not isinstance(values, dict)
            or not isinstance(ticket, str) or not ticket or len(ticket) > 128):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid action arguments")
            return
        try:
            op_id = self.server.session.request(session, action, values, ticket=ticket)
            self._send_json(HTTPStatus.OK, {"operation": op_id})
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))

    def _handle_post_receipt(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        operation = data.get("operation")
        if not isinstance(session, str) or not isinstance(operation, str):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid receipt query arguments")
            return
        try:
            receipt_data = self.server.session.receipt(session, operation)
            self._send_json(HTTPStatus.OK, receipt_data)
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))

    def _handle_post_ticket(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        if not isinstance(session, str):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid ticket request arguments")
            return
        try:
            ticket = self.server.session.issue_ticket(session)
            sequence = self.server.session.input_sequence(session) if hasattr(self.server.session, "input_sequence") else -1
            controls = self.server.session.controls_motor(session) if hasattr(self.server.session, "controls_motor") else False
            self._send_json(HTTPStatus.OK, {"ticket": ticket, "last_sequence": sequence, "controls_motor": controls})
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))

    def _handle_post_input(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        ticket = data.get("ticket")
        seq = data.get("seq")
        deadman = data.get("deadman")
        axes = data.get("axes")
        device = data.get("device")

        if not isinstance(session, str) or not isinstance(ticket, str):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid session or ticket")
            return
        if not isinstance(deadman, bool):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "motion-intent flag must be an exact boolean")
            return
        if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
            self._send_error_json(HTTPStatus.BAD_REQUEST, "seq must be a nonnegative integer")
            return
        if not isinstance(axes, list) or len(axes) != 3 or not all(_check_finite_number(a) and -1.0 <= a <= 1.0 for a in axes):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "axes must be a finite 3-vector in range [-1.0, 1.0]")
            return
        if not isinstance(device, str) or len(device) == 0 or len(device) > 64:
            self._send_error_json(HTTPStatus.BAD_REQUEST, "device must be a bounded non-empty string")
            return

        try:
            self.server.session.submit(session, ticket, seq, deadman, axes, device)
            self._send_json(HTTPStatus.OK, {"ok": True})
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))

    def _handle_post_stop(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        reason = data.get("reason", "api_stop")
        if not isinstance(session, str) or not isinstance(reason, str):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid stop request arguments")
            return
        try:

            self.server.session.stop(session, reason)
            self._send_json(HTTPStatus.OK, {"ok": True})
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))

    def _handle_post_retire(self, data: dict[str, Any]) -> None:
        session = data.get("session")
        if not isinstance(session, str):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Invalid retire request arguments")
            return
        try:
            retired = self.server.session.retire_session(session)
            self._send_json(HTTPStatus.OK, {"ok": True, "retired": retired})
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))


class ArmDashboardServer(ThreadingHTTPServer):
    """Threading HTTP server with bound port tracking and concurrency semaphore."""

    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        session: Any,
        request_handler_class: type[BaseHTTPRequestHandler] = ArmDashboardHandler,
    ) -> None:
        if server_address[0] != "127.0.0.1":
            raise ValueError("Arm dashboard must bind to 127.0.0.1")
        super().__init__(server_address, request_handler_class)
        self.session = session
        self.bound_port = self.server_address[1]
        self.semaphore = threading.BoundedSemaphore(MAX_CONCURRENT_REQUESTS)

    def process_request(self, request: socket.socket, client_address: Any) -> None:
        if not self.semaphore.acquire(blocking=False):
            self._reject_busy(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.semaphore.release()
            raise

    def process_request_thread(self, request: socket.socket, client_address: Any) -> None:
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.semaphore.release()

    @staticmethod
    def _reject_busy(request: socket.socket) -> None:
        body = b"Server busy; concurrency limit reached\n"
        response = (
            b"HTTP/1.1 503 Service Unavailable\r\n"
            b"Connection: close\r\n"
            b"Content-Type: text/plain; charset=utf-8\r\n"
            + f"Content-Length: {len(body)}\r\n\r\n".encode("ascii")
            + body
        )
        try:
            request.settimeout(0.25)
            request.sendall(response)
        except OSError:
            pass
        finally:
            request.close()


def run_dashboard(
    port: int = 8768,
    physical_opt_in: bool = False,
    session: Any = None,
) -> None:
    if session is None:
        if ArmManualSession is None:
            raise RuntimeError("arm_manual backend is not available in environment")
        session = ArmManualSession(physical_opt_in=physical_opt_in)

    server_address = ("127.0.0.1", port)
    server = ArmDashboardServer(server_address, session)
    mode_str = "PHYSICAL J1" if physical_opt_in else "DEMO (Virtual 3-joint)"
    print(f"IM6014 Arm Dashboard running in {mode_str} mode at http://127.0.0.1:{server.bound_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        server.server_close()
        session.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="IM6014 3-joint Arm Workbench Dashboard")
    parser.add_argument("--port", type=int, default=8768, help="Port to bind (default: 8768)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", dest="physical", action="store_false", help="Run in virtual demo mode (default)")
    mode.add_argument("--physical", dest="physical", action="store_true", help="Opt into physical mode after confirming the adapter path and wiring")
    parser.set_defaults(physical=False)
    args = parser.parse_args()

    run_dashboard(port=args.port, physical_opt_in=args.physical)


if __name__ == "__main__":
    main()
