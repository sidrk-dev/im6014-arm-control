#!/usr/bin/env python3
"""HTTP server invariant and routing tests for IM6014 Arm Dashboard.

Tests the actual HTTP server in arm_dashboard.py using a mock backend
conforming to the shared ArmManualSession public API.
"""

from __future__ import annotations

import http.client
import json
import math
import socket
import threading
import time
from typing import Any
import unittest

from arm_dashboard import ArmDashboardHandler, ArmDashboardServer, MAX_BODY_BYTES


class MockArmManualSession:
    """Mock conforming to the shared ArmManualSession public contract."""

    def __init__(self, physical_opt_in: bool = False) -> None:
        self.physical_opt_in = physical_opt_in
        self.sessions: set[str] = set()
        self.operations: dict[str, dict[str, Any]] = {}
        self.tickets: set[str] = set()
        self.ticket_counter = 0
        self.submitted_inputs: list[dict[str, Any]] = []
        self.stopped = False
        self.stop_reason: str | None = None
        self.closed = False

    def create_session(self) -> str:
        sid = f"mock-session-{len(self.sessions) + 1}"
        self.sessions.add(sid)
        return sid

    def controls_motor(self, session: str) -> bool:
        if session not in self.sessions:
            raise ValueError("Unknown session")
        return False

    def input_sequence(self, session: str) -> int:
        if session not in self.sessions:
            raise ValueError("Unknown session")
        return -1

    def retire_session(self, session: str) -> bool:
        if session in self.sessions:
            self.sessions.remove(session)
            return True
        return False

    def request(self, session: str, action: str, values: dict[str, Any], *, ticket: str | None = None) -> str:
        if session not in self.sessions:
            raise ValueError("Unauthorized session")
        if not isinstance(ticket, str) or ticket not in self.tickets:
            raise ValueError("Missing or invalid action ticket")
        self.tickets.remove(ticket)
        op_id = f"op-{len(self.operations) + 1}"
        self.operations[op_id] = {
            "status": "complete",
            "reason": None,
            "action": action,
            "values": values,
            "ticket_used": True,
        }
        return op_id

    def receipt(self, session: str, op: str) -> dict[str, Any]:
        if session not in self.sessions:
            raise ValueError("Unauthorized session")
        return self.operations.get(op, {"status": "rejected", "reason": "Operation unknown"})

    def snapshot(self) -> dict[str, Any]:
        return {
            "state": "CONNECTED",
            "reason": None,
            "connected": True,
            "demo": not self.physical_opt_in,
            "calibrated": True,
            "armed": False,
            "stop_confirmed": self.stopped,
            "feedback_age_s": 0.012,
            "settings": {"kp": 3.0, "kd": 0.2, "ticket": "secret_nested_ticket", "credential": "private"},
            "joints": [
                {
                    "name": "J1 base",
                    "source": "actual_encoder" if self.physical_opt_in else "virtual_pose",
                    "position_rad": 0.0,
                    "velocity_rad_s": 0.0,
                    "mode": 0,
                    "timeout": False,
                    "voltage": 24.1,
                    "driver_temp": 32.5,
                    "winding_temp": 34.0,
                    "errors": [],
                    "warnings": [],
                    "torque_nm": 0.0,
                    "session": "secret_joint_session",
                    "owner": "secret_joint_owner",
                },
                {"name": "J2 shoulder", "source": "virtual_pose", "position_rad": 0.0, "velocity_rad_s": 0.0},
                {"name": "J3 elbow", "source": "virtual_pose", "position_rad": 0.0, "velocity_rad_s": 0.0},
            ],
            "latest": {
                "position_rad": math.nan,
                "velocity_rad_s": 0.0,
                "session": "secret_latest_session",
                "nested": {"ticket": "secret_deep_ticket"},
            },
            "events": ["Session initialized", {"kind": "health", "status": "ok", "ticket": "secret_event_ticket", "session": "secret_event_session"}],
            # Internal fields that MUST NOT leak in observational snapshot
            "session": "secret_internal_session",
            "ticket": "secret_ticket_id",
            "owner": "secret_owner_authority",
        }

    def issue_ticket(self, session: str) -> str:
        if session not in self.sessions:
            raise ValueError("Unauthorized session")
        self.ticket_counter += 1
        tid = f"ticket-{self.ticket_counter}"
        self.tickets.add(tid)
        return tid

    def submit(
        self,
        session: str,
        ticket: str,
        seq: int,
        deadman: bool,
        axes: list[float],
        device: str,
    ) -> None:
        if session not in self.sessions:
            raise ValueError("Unauthorized session")
        if ticket not in self.tickets:
            raise ValueError("Invalid or expired ticket")
        self.tickets.remove(ticket)
        self.submitted_inputs.append({
            "session": session,
            "ticket": ticket,
            "seq": seq,
            "deadman": deadman,
            "axes": axes,
            "device": device,
        })

    def stop(self, session: str, reason: str) -> None:
        self.stopped = True
        self.stop_reason = reason

    def close(self) -> bool:
        self.closed = True
        return True


class ArmHttpServerTests(unittest.TestCase):
    """Test HTTP invariants, security boundaries, and routes."""

    server: ArmDashboardServer
    server_thread: threading.Thread
    port: int
    session: MockArmManualSession

    @classmethod
    def setUpClass(cls) -> None:
        cls.session = MockArmManualSession(physical_opt_in=False)
        # Bind to port 0 to let OS select an available port on 127.0.0.1
        cls.server = ArmDashboardServer(("127.0.0.1", 0), cls.session, ArmDashboardHandler)
        cls.port = cls.server.bound_port
        cls.server_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.server_thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.server_thread.join(timeout=2.0)

    def _request(
        self,
        method: str,
        path: str,
        body: bytes | None = None,
        headers: dict[str, str] | None = None,
        omit_origin: bool = False,
    ) -> tuple[int, dict[str, str], bytes]:
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3.0)
        req_headers = {
            "Host": f"127.0.0.1:{self.port}",
        }
        if method == "POST":
            req_headers["Origin"] = f"http://127.0.0.1:{self.port}"
            req_headers["Content-Type"] = "application/json"
        if headers:
            req_headers.update(headers)
        if omit_origin:
            req_headers.pop("Origin", None)

        conn.request(method, path, body=body, headers=req_headers)
        resp = conn.getresponse()
        resp_body = resp.read()
        resp_headers = {k: v for k, v in resp.getheaders()}
        conn.close()
        return resp.status, resp_headers, resp_body

    def test_host_header_validation(self) -> None:
        # Valid host with bound port -> 200 OK
        status, _, _ = self._request("GET", "/api/state", headers={"Host": f"127.0.0.1:{self.port}"})
        self.assertEqual(status, 200)

        # Host without bound port -> 403 Forbidden
        status, _, _ = self._request("GET", "/api/state", headers={"Host": "127.0.0.1"})
        self.assertEqual(status, 403)

        # External Host -> 403 Forbidden
        status, _, _ = self._request("GET", "/api/state", headers={"Host": f"attacker.com:{self.port}"})
        self.assertEqual(status, 403)

    def test_origin_header_validation_on_post(self) -> None:
        # Valid Origin -> 200 OK
        status, _, body = self._request(
            "POST",
            "/api/session",
            body=b"{}",
            headers={"Origin": f"http://127.0.0.1:{self.port}"},
        )
        self.assertEqual(status, 200)

        # The alternate loopback spelling is deliberately rejected on POST.
        status, _, _ = self._request(
            "POST",
            "/api/session",
            body=b"{}",
            headers={"Host": f"localhost:{self.port}"},
        )
        self.assertEqual(status, 403)

        # Missing Origin -> 403 Forbidden
        status, _, _ = self._request(
            "POST",
            "/api/session",
            body=b"{}",
            omit_origin=True,
        )
        self.assertEqual(status, 403)

        # Mismatched / external Origin -> 403 Forbidden
        status, _, _ = self._request(
            "POST",
            "/api/session",
            body=b"{}",
            headers={"Origin": "http://evil.com"},
        )
        self.assertEqual(status, 403)

    def test_body_size_cap_and_content_length(self) -> None:
        # Body larger than 16 KB -> 413 Payload Too Large
        oversized = b"x" * (MAX_BODY_BYTES + 1)
        status, _, _ = self._request("POST", "/api/session", body=oversized)
        self.assertEqual(status, 413)

        # Missing Content-Type -> 415 Unsupported Media Type
        status, _, _ = self._request(
            "POST",
            "/api/session",
            body=b"{}",
            headers={"Content-Type": "text/plain"},
        )
        self.assertEqual(status, 415)

        # A JSON-looking prefix is not an application/json media type.
        status, _, _ = self._request(
            "POST", "/api/session", body=b"{}", headers={"Content-Type": "application/jsonp"}
        )
        self.assertEqual(status, 415)

        # Valid parameters remain accepted and exercise the JSON parser path.
        status, _, _ = self._request(
            "POST", "/api/session", body=b"{}", headers={"Content-Type": "application/json; charset=utf-8"}
        )
        self.assertEqual(status, 200)

    def test_non_finite_json_rejection(self) -> None:
        # JSON containing NaN or Infinity must be rejected
        raw_nan = b'{"axes": [NaN, 0.0, 0.0]}'
        status, _, _ = self._request("POST", "/api/input", body=raw_nan)
        self.assertEqual(status, 400)

        raw_inf = b'{"axes": [Infinity, 0.0, 0.0]}'
        status, _, _ = self._request("POST", "/api/input", body=raw_inf)
        self.assertEqual(status, 400)

        # JSON's numeric grammar accepts exponents that overflow a float.
        raw_overflow = b'{"nested": {"value": 1e9999}}'
        status, _, _ = self._request("POST", "/api/session", body=raw_overflow)
        self.assertEqual(status, 400)

    def test_state_snapshot_observational_invariants(self) -> None:
        # /api/state GET returns snapshot and strictly strips any authority/secrets
        status, _, body = self._request("GET", "/api/state")
        self.assertEqual(status, 200)
        data = json.loads(body.decode("utf-8"))

        self.assertEqual(data.get("state"), "CONNECTED")
        self.assertEqual(data.get("connected"), True)
        self.assertIn("joints", data)
        self.assertEqual(len(data["joints"]), 3)
        self.assertEqual(data["settings"], {"kp": 3.0, "kd": 0.2})
        self.assertNotIn("position_rad", data["latest"])
        self.assertEqual(data["latest"]["velocity_rad_s"], 0.0)
        self.assertEqual(data["events"][1], {"kind": "health", "status": "ok"})
        self.assertNotIn("session", data["joints"][0])

        # Invariant: Must NOT leak internal session or ticket or owner capability
        self.assertNotIn("session", data)
        self.assertNotIn("ticket", data)
        self.assertNotIn("owner", data)
        self.assertNotIn("csrf", data)
        self.assertNotIn("secret_", json.dumps(data))

    def test_observer_resume_does_not_mint_a_ticket(self) -> None:
        sid = self.session.create_session()
        before = self.session.ticket_counter
        for _ in range(20):
            status, _, body = self._request("POST", "/api/session-status", body=json.dumps({"session": sid}).encode())
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(body), {"controls_motor": False, "last_sequence": -1})
        self.assertEqual(self.session.ticket_counter, before)
        status, _, _ = self._request("POST", "/api/session-status", body=b'{"session":"unknown"}')
        self.assertEqual(status, 400)

    def test_session_capacity_reports_a_clear_error(self) -> None:
        from unittest.mock import patch
        with patch.object(self.session, 'create_session', side_effect=RuntimeError('Maximum sessions reached')):
            status, _, body = self._request("POST", "/api/session", body=b'{}')
        self.assertEqual(status, 503)
        self.assertEqual(json.loads(body)['reason'], 'Maximum sessions reached')

    def test_full_api_workflow(self) -> None:
        # 1. Acquire session
        status, _, body = self._request("POST", "/api/session", body=b"{}")
        self.assertEqual(status, 200)
        sess = json.loads(body.decode("utf-8")).get("session")
        self.assertTrue(sess and sess.startswith("mock-session"))

        # 2. An action cannot be submitted without its single-use ticket.
        action_without_ticket = json.dumps({"session": sess, "action": "check", "values": {}}).encode("utf-8")
        status, _, _ = self._request("POST", "/api/action", body=action_without_ticket)
        self.assertEqual(status, 400)
        self.assertEqual(self.session.operations, {})

        # 3. Ticketed action request
        ticket_payload = json.dumps({"session": sess}).encode("utf-8")
        status, _, body = self._request("POST", "/api/ticket", body=ticket_payload)
        self.assertEqual(status, 200)
        action_ticket = json.loads(body.decode("utf-8")).get("ticket")
        self.assertTrue(action_ticket and action_ticket.startswith("ticket-"))
        action_payload = json.dumps({
            "session": sess,
            "action": "check",
            "values": {},
            "ticket": action_ticket,
        }).encode("utf-8")
        status, _, body = self._request("POST", "/api/action", body=action_payload)
        self.assertEqual(status, 200)
        op_id = json.loads(body.decode("utf-8")).get("operation")
        self.assertTrue(op_id and op_id.startswith("op-"))
        status, _, _ = self._request("POST", "/api/action", body=action_payload)
        self.assertEqual(status, 400, "an action ticket is single-use")
        self.assertEqual(len(self.session.operations), 1)

        # 4. Receipt query
        receipt_payload = json.dumps({"session": sess, "operation": op_id}).encode("utf-8")
        status, _, body = self._request("POST", "/api/receipt", body=receipt_payload)
        self.assertEqual(status, 200)
        receipt_res = json.loads(body.decode("utf-8"))
        self.assertEqual(receipt_res.get("status"), "complete")

        # 5. Issue ticket for input
        ticket_payload = json.dumps({"session": sess}).encode("utf-8")
        status, _, body = self._request("POST", "/api/ticket", body=ticket_payload)
        self.assertEqual(status, 200)
        tid = json.loads(body.decode("utf-8")).get("ticket")
        self.assertTrue(tid and tid.startswith("ticket-"))

        # 6. Submit input (valid)
        input_payload = json.dumps({
            "session": sess,
            "ticket": tid,
            "seq": 0,
            "deadman": True,
            "axes": [0.5, -0.2, 0.0],
            "device": "keyboard",
        }).encode("utf-8")
        status, _, body = self._request("POST", "/api/input", body=input_payload)
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body.decode("utf-8")), {"ok": True})

        # 7. Submit input (invalid deadman not bool)
        bad_deadman = json.dumps({
            "session": sess,
            "ticket": tid,
            "seq": 1,
            "deadman": 1,  # int, not bool
            "axes": [0.5, 0.0, 0.0],
            "device": "keyboard",
        }).encode("utf-8")
        status, _, _ = self._request("POST", "/api/input", body=bad_deadman)
        self.assertEqual(status, 400)

        # 8. Submit input (invalid axes out of [-1, 1])
        bad_axes = json.dumps({
            "session": sess,
            "ticket": tid,
            "seq": 2,
            "deadman": True,
            "axes": [2.5, 0.0, 0.0],  # Out of range
            "device": "keyboard",
        }).encode("utf-8")
        status, _, _ = self._request("POST", "/api/input", body=bad_axes)
        self.assertEqual(status, 400)

        # 9. Priority Stop
        stop_payload = json.dumps({"session": sess, "reason": "emergency_test"}).encode("utf-8")
        status, _, body = self._request("POST", "/api/stop", body=stop_payload)
        self.assertEqual(status, 200)
        self.assertTrue(self.session.stopped)
        self.assertEqual(self.session.stop_reason, "emergency_test")

        # 10. Retire session
        retire_payload = json.dumps({"session": sess}).encode("utf-8")
        status, _, body = self._request("POST", "/api/retire", body=retire_payload)
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body.decode("utf-8")).get("retired"), True)

    def test_static_files_and_path_traversal_protection(self) -> None:
        # Index HTML
        status, headers, body = self._request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"IM6014 Arm Dashboard", body)

        # Vendor Three.js
        status, _, body = self._request("GET", "/vendor/three.module.js")
        self.assertEqual(status, 200)
        self.assertTrue(len(body) > 1000)

        # Assets model.json
        status, _, body = self._request("GET", "/arm_assets/model.json")
        self.assertEqual(status, 200)
        self.assertIn(b"revolute_2", body)

        # Directory traversal attempt -> must be rejected (403 or 404)
        status, _, _ = self._request("GET", "/../arm_station.py")
        self.assertIn(status, (403, 404))

        status, _, _ = self._request("GET", "/arm_assets/../../etc/passwd")
        self.assertIn(status, (403, 404))
        status, _, _ = self._request("GET", "/%2e%2e/arm_station.py")
        self.assertEqual(status, 404)
        status, _, _ = self._request("GET", "/motor_session.py")
        self.assertEqual(status, 404)
        status, _, _ = self._request("GET", "/arm_assets/meshes/Part_1.stl")
        self.assertEqual(status, 200)

    def test_get_ports_endpoint(self) -> None:
        status, headers, body = self._request("GET", "/api/ports")
        self.assertEqual(status, 200)
        self.assertIn("application/json", headers.get("Content-Type", ""))
        data = json.loads(body.decode("utf-8"))
        self.assertIn("ports", data)
        self.assertIn("default", data)
        self.assertIsInstance(data["ports"], list)
        self.assertTrue(all("device" in p and "label" in p for p in data["ports"]))

    def test_concurrency_semaphore_quick_reject(self) -> None:
        acquired = []
        deadline = time.monotonic() + 0.5
        while len(acquired) < 16 and time.monotonic() < deadline:
            if self.server.semaphore.acquire(blocking=False):
                acquired.append(True)
            else:
                time.sleep(0.001)
        try:
            self.assertEqual(len(acquired), 16)
            status, _, _ = self._request("GET", "/api/state")
            self.assertEqual(status, 503)
        finally:
            for _ in acquired:
                self.server.semaphore.release()


if __name__ == "__main__":
    unittest.main()
