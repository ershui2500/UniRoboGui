#!/usr/bin/env python3
"""Exercise real worker processes in Mock mode; never initialize real DDS."""
import concurrent.futures
import http.client
import importlib.util
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import threading
import time
from unittest.mock import patch

from test_customer_config_http import read_websocket_text_message


project = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("pc_session", project / "scripts/pc_session.py")
pc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pc)


def main(binary):
    session = pc.Session(binary, project / "web", mock=True)
    server = pc.Server(("127.0.0.1", 0), pc.Handler)
    server.session = session
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    port = server.server_port

    def request(path, payload=None, method=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=15)
        try:
            conn.request(method or ("POST" if payload is not None else "GET"), path,
                         None if payload is None else json.dumps(payload),
                         headers or {"Content-Type": "application/json", "X-Unirobo-PC-Session": session.id})
            response = conn.getresponse()
            body = response.read()
            return response.status, json.loads(body) if body.startswith(b"{") else body
        finally:
            conn.close()

    connect = {"product_id": "g1", "interface": "lo"}
    def check_partial_telemetry(product):
        def partial_health(port, path, *args, **kwargs):
            code, kind, raw = original_upstream(port, path, *args, **kwargs)
            if path == "/api/health":
                data = json.loads(raw)
                data["status"] = "degraded"
                for key, source in data["sources"].items():
                    source.update(received=True, age_ms=1 if key == "low_state" else 60000,
                                  status="online" if key == "low_state" else "offline")
                raw = json.dumps(data).encode()
            return code, kind, raw
        with patch.object(pc, "upstream", side_effect=partial_health):
            assert session.state()["state"] == "online", (product, "auxiliary data loss must not disconnect fresh LowState")
        health = {"dds_initialized": False, "server_time_ms": 100000,
                  "sources": {"low_state": {"received": True, "age_ms": 1, "status": "online"}}}
        with patch.object(pc, "upstream", return_value=(200, "application/json", json.dumps(health).encode())):
            assert session.state()["state"] != "online", (product, "DDS must be initialized")
        for source in ({"received": True, "age_ms": 60000, "status": "offline"},
                       {"received": True, "age_ms": 1500, "status": "delayed"},
                       {"received": False, "age_ms": None, "status": "offline"}):
            health = {"dds_initialized": True, "server_time_ms": 100000,
                      "sources": {"low_state": source}}
            with patch.object(pc, "upstream", return_value=(200, "application/json", json.dumps(health).encode())):
                assert session.state()["state"] != "online", (product, source)
    try:
        state = request("/api/pc/session")[1]
        assert state["state"] == "disconnected" and not state["dds_initialized"]
        assert not state["physical_identity_verified"] and state["mock"]
        snapshot = request("/api/snapshot")[1]
        assert snapshot["sources"]["low_state"]["received"] is False
        assert not snapshot["voice"]["initialized"] and not snapshot["control"]["initialized"]
        assert request("/")[0] == 200
        assert request("/app.js")[0] == 200
        assert request("/assets/unitree/g1_description/g1_29dof.urdf")[0] == 200
        assert request("/assets/unitree/r1_description/R1.urdf")[0] == 200
        for path in ("/../CMakeLists.txt", "/%2e%2e/CMakeLists.txt", "/%00", "/scripts/pc.sh",
                     "/assets/../../../CMakeLists.txt", "/assets/%2e%2e/app.js", "/app.js?x=1"):
            assert request(path)[0] == 404, path
        assert request("/api/pc/session", headers={"Host": "evil.example"})[0] == 403
        assert request("/api/pc/connect", connect,
                       headers={"Content-Type": "application/json", "Origin": "https://evil.example"})[0] == 403
        assert request("/api/pc/connect", connect, method="PUT")[0] == 403
        for payload in ({}, {**connect, "shell": "echo bad"}, {**connect, "interface": "lo; echo bad"},
                        {**connect, "product_id": "unknown"}, {**connect, "interface": ["lo"]}):
            assert request("/api/pc/connect", payload)[0] == 400
        # With no ready wired interface, real mode rejects loopback too.
        assert not any(item["name"] == "lo" for item in pc.interfaces(False))
        status, manifest = request("/api/robot/manifest")
        assert status == 200 and manifest["runtime_mode"] == "pc_readonly"
        assert manifest["physical_identity_verified"] is False
        assert all(c["key"] == "telemetry" or not c["available"] for c in manifest["capabilities"])
        products = request("/api/robot/products")[1]["products"]
        assert {p["product_id"] for p in products if p["pc_connect_supported"]} == {"g1", "r1", "go2", "a2"}
        assert all(p["pc_direct_validation"] == "pending" for p in products)

        # Every existing write route must be rejected by manager AND direct worker.
        source = (project / "src/http_server.cpp").read_text()
        routes = set(re.findall(r'target == "(/api/[^\"]+)"', source)) - pc.READ_APIS
        for path in routes:
            assert request(path, {})[0] in (400, 403) or path == "/api/robot/switch", path
            conn = http.client.HTTPConnection("127.0.0.1", session.port, timeout=3)
            conn.request("POST", path, "{}", {"Content-Type": "application/json"})
            response = conn.getresponse()
            assert response.status == 403, path
            assert json.loads(response.read())["error"] == "pc_readonly_runtime"
            conn.close()
        assert request("/api/voice/llm/customer-config")[0] == 403
        assert request("/api/perception/map-file?map_name=test1.pcd")[0] == 403

        old_id = session.id
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
            replies = list(pool.map(lambda _: request("/api/pc/connect", connect), range(3)))
        assert all(code in (200, 409) for code, _ in replies)
        assert len({body["session"]["session_id"] for code, body in replies if code == 200}) == 1
        assert session.id != old_id
        worker = session.worker
        time.sleep(0.5)
        assert worker.poll() is None, "Worker must outlive the HTTP thread that created it"
        for _ in range(30):
            if request("/api/pc/session")[1]["state"] == "online":
                break
            time.sleep(0.05)
        else:
            raise AssertionError("Mock telemetry did not become online")
        snapshot = request("/api/snapshot")[1]
        assert snapshot["pc_session_id"] == session.id
        assert snapshot["dds_initialized"] and len(snapshot["joints"]) == 29
        assert snapshot["robot"]["model_supported"]
        assert snapshot["control"]["initialized"] and snapshot["voice"]["initialized"]
        manifest = request("/api/robot/manifest")[1]
        assert manifest["runtime_mode"] == "pc_sdk2"
        caps = {item["key"]: item for item in manifest["capabilities"]}
        assert all(caps[key]["available"] for key in ("locomotion", "joint_debug", "joint_teach", "audio", "slam"))
        assert not caps["camera_rgb"]["available"] and not caps["camera_depth"]["available"]
        assert caps["audio"]["parameters"]["tts_backends"] == "unitree"
        assert request("/api/voice/volume", {"volume": 20})[0] == 200
        assert request("/api/voice/tts", {"text": "Mock test", "speaker_id": -1, "tts_backend": "unitree"})[0] == 202
        status, body = request("/api/voice/tts", {"text": "Mock test", "speaker_id": -1, "tts_backend": "kokoro"})
        assert status >= 400 and body["error"] == "local_tts_unavailable"
        assert request("/api/camera/command", {})[0] == 403
        assert request("/api/perception/topic-config", {})[0] == 403
        assert request("/api/unknown", {})[0] == 403
        assert request("/api/control/velocity", {"active": False}, headers={"Content-Type": "application/json", "X-Unirobo-PC-Session": old_id})[0] == 409
        assert request("/api/control/command", {"request_key": "mock-unconfirmed", "category": "mode", "command": "start", "argument": 0, "confirmed": False})[0] >= 400
        assert request("/api/control/joint-debug/heartbeat", {})[0] != 403
        assert request("/api/perception/status")[0] == 200
        assert request("/api/perception/frame")[0] == 200
        assert request("/api/perception/global-map")[0] == 200
        def eventually(check):
            for _ in range(100):
                if check():
                    return
                time.sleep(0.03)
            raise AssertionError("Mock state transition timed out")

        # Exercise the actual shared services through the PC proxy, not just status fixtures.
        velocity = {"vx": 0.1, "vy": 0, "vyaw": 0, "speed_mode": 0, "active": True}
        code, result = request("/api/control/velocity", velocity)
        assert result.get("accepted"), (code, result)
        assert request("/api/pc/disconnect", {})[0] == 409
        assert request("/api/control/velocity", dict(velocity, active=False, vx=0))[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["control"]["motion"]["state"] == "stopped")
        code, result = request("/api/control/joint-debug/apply", {
            "mode": "upper_body", "joints": [{"index": 15, "q": 0.03}], "confirmed": True})
        assert result.get("accepted"), (code, result)
        assert request("/api/pc/disconnect", {})[0] == 409
        assert request("/api/control/joint-debug/heartbeat", {})[1]["accepted"]
        assert request("/api/control/joint-debug/stop", {"confirmed": True})[1]["accepted"]
        eventually(lambda: request("/api/control/joint-debug/status")[1]["dds_state"] == "idle")
        assert request("/api/control/joint-debug/teach/record", {"name": "pc-mock-test", "confirmed": True})[1]["accepted"]
        assert request("/api/pc/disconnect", {})[0] == 409
        assert request("/api/control/joint-debug/stop", {"confirmed": True})[1]["accepted"]
        eventually(lambda: request("/api/control/joint-debug/status")[1]["dds_state"] == "idle")
        for command in ("start_mapping", "stop_slam"):
            code, result = request("/api/perception/command", {"command": command, "confirmed": True,
                "request_key": "pc-mock-" + command, "map_name": "test1.pcd"})
            assert result.get("accepted"), (code, result)
            if command == "start_mapping":
                assert request("/api/pc/disconnect", {})[0] == 409

        action = {"request_key": "pc-arm-wave", "category": "arm_action", "command": "execute",
                  "argument": 25, "confirmed": True}
        assert request("/api/control/command", action)[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["control"]["last_command"]["state"] == "succeeded")
        assert request("/api/control/status")[1]["arm_action_state"] == "completed_mock"
        assert request("/api/control/status")[1]["control_owned"] is False
        assert request("/api/control/command", dict(action, request_key="pc-arm-hold", argument=15))[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["arm_action_state"] == "holding_reported")
        assert request("/api/pc/disconnect", {})[0] == 409
        assert request("/api/control/command", dict(action, request_key="pc-arm-release", argument=99))[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["control_owned"] is False)
        # Preparation blocks both HTTP and physical-remote entry points in the shared service.
        assert pc.upstream(session.port, "/api/pc/prepare-disconnect", "POST", "{}")[0] == 200
        assert not request("/api/control/velocity", velocity)[1]["accepted"]
        assert not request("/api/control/joint-debug/apply", {
            "mode": "upper_body", "joints": [{"index": 15, "q": 0.03}], "confirmed": True})[1]["accepted"]
        assert pc.upstream(session.port, "/api/pc/resume", "POST", "{}")[0] == 200

        # Test each safety prerequisite independently; all checks still use the live worker otherwise.
        original_upstream = pc.upstream
        check_partial_telemetry("g1")
        def override_status(path, transform):
            def replacement(port, target, *args, **kwargs):
                code, kind, raw = original_upstream(port, target, *args, **kwargs)
                if target == path:
                    data = json.loads(raw)
                    transform(data)
                    raw = json.dumps(data).encode()
                return code, kind, raw
            return replacement
        for path, transform, error in (
            ("/api/control/status", lambda data: data.update(control_owned=True), "pc_control_active"),
            ("/api/control/status", lambda data: data["control"]["motion"].update(active=True), "pc_motion_active"),
            ("/api/control/joint-debug/status", lambda data: data.update(teach_state="holding", arm_sdk_weight=1), "pc_joint_control_active"),
            ("/api/perception/status", lambda data: data.update(mode="paused", target_set=True), "pc_navigation_active"),
            ("/api/snapshot", lambda data: data["sources"]["low_state"].update(age_ms=2001), "pc_stop_state_unavailable"),
            ("/api/control/status", lambda data: data["control"].update(sport_state_age_ms=2001), "pc_stop_state_unavailable"),
        ):
            with patch.object(pc, "upstream", side_effect=override_status(path, transform)):
                for action, payload in (("/api/pc/disconnect", {}), ("/api/robot/switch", {"product_id": "r1"})):
                    status, body = request(action, payload)
                    assert status == 409 and body["error"] == error, (path, body)
                    assert session.worker is worker and worker.poll() is None
        # Squatting/non-walking telemetry may contain estimator drift even without control.
        def squat_status(port, path, *args, **kwargs):
            code, kind, raw = original_upstream(port, path, *args, **kwargs)
            data = json.loads(raw)
            if path == "/api/control/status":
                data["control"].update(fsm_id=706, fsm_mode=1)
            if path == "/api/snapshot":
                data["odometry"].update(velocity_m_s=[-0.147, 0.016, 0.263], yaw_speed_rad_s=-0.15)
            return code, kind, json.dumps(data).encode()
        with patch.object(session, "mock", False), patch.object(pc, "upstream", side_effect=squat_status):
            session.check_stopped()

        # Ignore vertical estimator noise and tolerate minor planar jitter in walking mode.
        with patch.object(session, "mock", False), patch.object(pc, "upstream", side_effect=override_status(
                "/api/snapshot", lambda data: data["odometry"].update(
                    velocity_m_s=[0.01, -0.01, 0.3], yaw_speed_rad_s=0.02))):
            session.check_stopped()
        with patch.object(session, "mock", False), patch.object(pc, "upstream", side_effect=override_status(
                "/api/snapshot", lambda data: data["odometry"].update(
                    velocity_m_s=[0, 0, 0], yaw_speed_rad_s=0.2))):
            status, body = request("/api/pc/disconnect", {})
            assert status == 409 and body["error"] == "pc_robot_moving"

        # Real physical velocity cannot be inferred from a locally stopped command queue.
        with patch.object(session, "mock", False), patch.object(pc, "upstream", side_effect=override_status(
                "/api/snapshot", lambda data: data["odometry"].update(velocity_m_s=[0.2, 0, 0]))):
            status, body = request("/api/pc/disconnect", {})
            assert status == 409 and body["error"] == "pc_robot_moving"

        entered, release = threading.Event(), threading.Event()
        def slow_voice(port, path, *args, **kwargs):
            if path == "/api/voice/volume":
                entered.set()
                assert release.wait(3)
            return original_upstream(port, path, *args, **kwargs)
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            with patch.object(pc, "upstream", side_effect=slow_voice):
                pending = pool.submit(request, "/api/voice/volume", {"volume": 20})
                try:
                    assert entered.wait(2)
                    assert request("/api/pc/disconnect", {})[0] == 409
                    assert request("/api/pc/recover", {"session_id": session.id})[0] == 409
                    assert request("/api/control/velocity", dict(velocity, active=False, vx=0))[1]["accepted"]
                finally:
                    release.set()
                assert pending.result()[0] == 200

        websocket = socket.create_connection(("127.0.0.1", port), timeout=3)
        websocket.sendall((f"GET /ws/telemetry HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n"
                           "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                           "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                           "Sec-WebSocket-Version: 13\r\n\r\n").encode())
        handshake = b""
        while not handshake.endswith(b"\r\n\r\n"):
            handshake += websocket.recv(1)
        assert b"101 Switching Protocols" in handshake
        reader = websocket.makefile("rb")
        try:
            frame = json.loads(read_websocket_text_message(reader))
            assert frame["pc_session_id"] == session.id and frame["dds_initialized"]
        finally:
            reader.close()
            websocket.close()
        fake_health = {"dds_initialized": True, "status": "degraded", "server_time_ms": 10000,
                       "sources": {"low_state": {"age_ms": 2500, "received": True}}}
        with patch.object(pc, "upstream", return_value=(200, "application/json", json.dumps(fake_health).encode())):
            state = session.state()
            assert state["state"] == "stale" and state["last_received_ms"] == 7500
        with patch.object(pc, "interfaces", return_value=[]):
            assert session.state()["state"] == "network_unavailable"
        status, body = request("/api/pc/disconnect", {})
        assert status == 200 and worker.poll() is not None
        assert not body["session"]["dds_initialized"] and body["session"]["state"] == "disconnected"
        empty = request("/api/snapshot")[1]
        assert empty["pc_session_id"] != snapshot["pc_session_id"]
        assert not empty["sources"]["low_state"]["received"] and empty["sequence"] == 0
        idle_worker = session.worker
        assert request("/api/pc/disconnect", {})[0] == 200 and session.worker is idle_worker
        assert request("/api/robot/switch", {"product_id": "r1"})[0] == 200
        assert idle_worker.poll() is not None and request("/api/robot/manifest")[1]["identity"]["product_id"] == "r1"
        # R1 PC mode reuses its sparse profile and existing SDK2 services.
        r1_connect = {"product_id": "r1", "interface": "lo"}
        assert request("/api/pc/connect", r1_connect)[0] == 200
        r1_snapshot = request("/api/snapshot")[1]
        assert len(r1_snapshot["joints"]) == 26
        assert r1_snapshot["robot"]["model_dof"] == 26
        r1_manifest = request("/api/robot/manifest")[1]
        r1_caps = {item["key"]: item for item in r1_manifest["capabilities"]}
        assert all(r1_caps[key]["available"] for key in (
            "telemetry", "locomotion", "joint_debug", "joint_teach", "head_control", "audio"))
        assert not any(r1_caps[key]["available"] for key in (
            "camera_depth", "slam"))
        assert not r1_caps["camera_rgb"]["available"]
        assert r1_caps["camera_rgb"]["parameters"]["reason_zh"] == "电脑端尚未接入机器人相机传输"
        assert request("/api/camera/status")[0] == 403
        assert request("/api/camera/command", {"request_key": "pc-deny-rgb", "command": "start_v4l2", "confirmed": True})[0] == 403
        for key in ("camera_rgb", "camera_depth"):
            assert "receiver_start_allowed_when_unavailable" not in r1_caps[key]["parameters"]
            assert r1_caps[key]["parameters"]["reason_zh"] == "电脑端尚未接入机器人相机传输"
        assert request("/api/camera/command", {"request_key": "pc-deny-depth", "command": "start_realsense", "confirmed": True})[0] == 403
        assert request("/api/camera/command", {"request_key": "pc-deny-source", "command": "start_v4l2", "rgb_source": "/dev/video0", "confirmed": True})[0] == 403
        assert request("/api/camera/rgb/frame.jpg")[0] == 403
        assert request("/api/camera/depth/frame.jpg")[0] == 403
        assert r1_caps["locomotion"]["parameters"]["real_control_policy"] == "operator_validation"
        assert r1_caps["audio"]["parameters"]["tts_backends"] == "unitree"
        r1_control = request("/api/control/status")[1]["control"]
        check_partial_telemetry("r1")
        assert r1_control["sport_state_received"] and r1_control["sport_state_age_ms"] <= 1000
        original_upstream = pc.upstream
        def stale_r1_odometry(port, path, *args, **kwargs):
            code, kind, raw = original_upstream(port, path, *args, **kwargs)
            if path == "/api/snapshot":
                data = json.loads(raw)
                data["sources"]["odometry"]["age_ms"] = 2001
                raw = json.dumps(data).encode()
            return code, kind, raw
        with patch.object(pc, "upstream", side_effect=stale_r1_odometry):
            try:
                session.check_stopped()
            except ValueError as error:
                assert str(error) == "pc_stop_state_unavailable"
            else:
                raise AssertionError("R1 disconnect must require fresh odometry")
        r1_session_id = session.id
        assert request("/api/pc/recover", {"session_id": r1_session_id})[0] == 200
        r1_recovery_id = session.id
        readonly_status = request("/api/control/status")[1]["control"]
        assert readonly_status["sport_state_received"] and readonly_status["sport_state_age_ms"] <= 1000
        assert request("/api/control/velocity", {"active": True, "vx": 0.1})[0] == 409
        assert request("/api/pc/confirm-recovery", {
            "session_id": r1_recovery_id, "confirmed": True})[0] == 200
        assert not request("/api/pc/session")[1]["recovery_required"]
        assert request("/api/pc/disconnect", {})[0] == 200
        a2_connect = {"product_id": "a2", "interface": "lo"}
        assert request("/api/pc/connect", a2_connect)[0] == 200
        a2_manifest = request("/api/robot/manifest")[1]
        assert a2_manifest["identity"]["product_id"] == "a2"
        check_partial_telemetry("a2")
        a2_caps = {item["key"]: item for item in a2_manifest["capabilities"]}
        assert a2_caps["locomotion"]["available"]
        assert a2_caps["joint_debug"]["available"]
        assert a2_caps["camera_rgb"]["available"]
        assert not a2_caps["slam"]["available"]
        assert a2_caps["audio"]["parameters"]["tts_backends"] == "unitree"
        a2_debug = request("/api/control/joint-debug/status")[1]
        assert a2_debug["motion_mode_switch_supported"]
        assert a2_debug["motion_mode_switch_service"] == "ai_sport"
        for mode, active in [("high_level", True), ("low_level", False)]:
            assert request("/api/control/joint-debug/mode", {
                "mode": mode, "confirmed": True})[1]["accepted"]
            assert request("/api/control/joint-debug/status")[1]["motion_service_active"] is active
        assert request("/api/control/status")[1]["control"]["fsm_id"] == 1
        assert request("/api/control/joint-debug/mode", {
            "mode": "high_level", "confirmed": True})[1]["accepted"]
        assert request("/api/control/command", {"request_key": "a2-mock-reset-after-crouch",
            "category": "mode", "command": "balance_stand", "argument": 0,
            "confirmed": True})[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["control"]["last_command"]["state"] == "succeeded")
        a2_velocity = {"vx": 0.1, "vy": 0, "vyaw": 0, "speed_mode": 0, "active": True}
        assert request("/api/control/velocity", a2_velocity)[1]["accepted"]
        assert request("/api/control/velocity", dict(a2_velocity, active=False, vx=0))[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["control"]["motion"]["state"] == "stopped")
        def a2_moving_status(port, path, *args, **kwargs):
            code, kind, raw = original_upstream(port, path, *args, **kwargs)
            data = json.loads(raw)
            if path == "/api/control/status":
                data["control"].update(fsm_id=3)
            if path == "/api/snapshot":
                data["odometry"].update(velocity_m_s=[0.2, 0, 0], yaw_speed_rad_s=0)
            return code, kind, json.dumps(data).encode()
        with patch.object(session, "mock", False), patch.object(pc, "upstream", side_effect=a2_moving_status):
            try:
                session.check_stopped()
            except ValueError as error:
                assert str(error) == "pc_robot_moving"
            else:
                raise AssertionError("A2 walking FSM 3 must not inherit G1 passive-state exemptions")
        assert request("/api/pc/disconnect", {})[0] == 200
        go2_connect = {"product_id": "go2", "interface": "lo"}
        assert request("/api/pc/connect", go2_connect)[0] == 200
        go2_manifest = request("/api/robot/manifest")[1]
        assert go2_manifest["identity"]["product_id"] == "go2"
        go2_caps = {item["key"]: item for item in go2_manifest["capabilities"]}
        assert go2_caps["locomotion"]["available"]
        assert go2_caps["joint_debug"]["available"]
        check_partial_telemetry("go2")
        assert go2_caps["camera_rgb"]["available"]
        go2_velocity = {"vx": 0.1, "vy": 0, "vyaw": 0, "speed_mode": 0, "active": True}
        assert request("/api/control/velocity", go2_velocity)[1]["accepted"]
        assert request("/api/control/velocity", dict(go2_velocity, active=False, vx=0))[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["control"]["motion"]["state"] == "stopped")
        assert request("/api/pc/disconnect", {})[0] == 200
        assert request("/api/pc/connect", connect)[0] == 200
        assert request("/api/control/command", {"request_key": "crash-with-arm-hold", "category": "arm_action",
            "command": "execute", "argument": 15, "confirmed": True})[1]["accepted"]
        eventually(lambda: request("/api/control/status")[1]["arm_action_state"] == "holding_reported")
        session.worker.kill()  # Abrupt crash of this disposable local Mock fixture only.
        session.worker.wait(timeout=5)
        assert session.worker.returncode != 0
        assert request("/api/pc/session")[1]["state"] == "error"
        assert request("/api/snapshot")[0] == 503
        assert request("/")[0] == 200  # Recovery UI survives a worker crash.
        # A dead real worker is not stop evidence. Recovery must first create a
        # new, read-only session, never replay a command or silently enable control.
        crashed_id = session.id
        with patch.object(session, "mock", False):
            assert request("/api/pc/disconnect", {})[0] == 409
        assert request("/api/pc/recover", {"session_id": crashed_id})[0] == 200
        recovery_id = session.id
        assert recovery_id != crashed_id and session.worker.poll() is None
        assert request("/api/robot/manifest")[1]["runtime_mode"] == "pc_readonly"
        assert request("/api/pc/session")[1]["recovery_required"]
        assert request("/api/control/velocity", velocity)[0] == 409
        assert request("/api/robot/switch", {"product_id": "r1"})[0] == 409
        assert request("/api/pc/confirm-recovery", {"session_id": crashed_id, "confirmed": True})[0] == 409
        assert request("/api/pc/confirm-recovery", {"session_id": recovery_id, "confirmed": False})[0] == 400
        eventually(lambda: request("/api/snapshot")[1]["sources"]["low_state"]["received"])
        assert request("/api/pc/confirm-recovery", {"session_id": recovery_id, "confirmed": True})[0] == 200
        assert not request("/api/pc/session")[1]["recovery_required"]
        assert request("/api/pc/session")[1]["state"] == "disconnected"
        assert request("/api/snapshot")[1]["control"]["last_command"]["request_id"] == 0
        assert request("/api/pc/connect", connect)[0] == 200
        # Persistent cable/data loss cannot prove stop, but allows local retirement
        # into a read-only worker with all business writes blocked at both layers.
        live_worker = session.worker
        live_id = session.id
        with patch.object(pc, "upstream", side_effect=override_status(
                "/api/snapshot", lambda data: data["sources"]["low_state"].update(age_ms=60000))):
            assert request("/api/pc/disconnect", {})[0] == 409
            assert request("/api/pc/recover", {"session_id": live_id})[0] == 200
            assert live_worker.poll() is not None and session.id != live_id
            assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": False})[0] == 400
            assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 200
            assert not session.interface and not session.recovery_required
            assert request("/api/robot/switch", {"product_id": "r1"})[0] == 200
            assert request("/api/pc/connect", connect)[0] == 200
            assert request("/api/pc/recover", {"session_id": session.id})[0] == 200
        # Failed read-only DDS initialization must still have a manual retry path.
        readonly_id = session.id
        assert request("/api/pc/recover", {"session_id": readonly_id})[0] == 200
        assert session.id != readonly_id and session.worker_readonly
        assert request("/api/control/command", {"confirmed": True})[0] == 409
        assert pc.upstream(session.port, "/api/control/command", "POST", '{"confirmed":true}')[0] == 403
        assert request("/api/pc/connect", connect)[0] == 409
        eventually(lambda: request("/api/snapshot")[1]["sources"]["low_state"]["received"])
        with patch.object(session, "mock", False), patch.object(pc, "upstream", side_effect=override_status(
                "/api/snapshot", lambda data: data["odometry"].update(velocity_m_s=[0.2, 0, 0]))):
            assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 409
        with patch.object(session, "inflight_writes", 1):
            assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 409
        # A failed observer must also be disposable without restoring DDS first.
        session.worker.kill()  # Disposable Mock observer only.
        session.worker.wait(timeout=5)
        assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 200
        assert not session.interface and not session.recovery_required
        assert request("/api/pc/connect", connect)[0] == 200
        assert request("/api/pc/recover", {"session_id": session.id})[0] == 200
        # If the idle view fails to start after the confirmed observer exits,
        # there must still be an ordinary recovery route instead of a targetless lock.
        with patch.object(pc.subprocess, "Popen", side_effect=OSError("idle start fixture")):
            assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 503
        assert not session.interface and not session.recovery_required and session.worker is None
        assert request("/api/pc/session")[1]["state"] == "error"
        assert request("/api/pc/disconnect", {})[0] == 200
        assert request("/api/pc/connect", connect)[0] == 200
        # Neither active local controllers nor unresolved navigation can be retired.
        for path, transform in (
            ("/api/control/status", lambda data: data.update(recovery_blocked=True)),
            ("/api/control/status", lambda data: data["control"]["motion"].update(active=True)),
            ("/api/control/joint-debug/status", lambda data: data.update(teach_state="holding", arm_sdk_weight=1)),
            ("/api/perception/status", lambda data: data.update(mode="navigating", target_set=True)),
            ("/api/perception/status", lambda data: data.update(control_confirmation_pending=True)),
        ):
            unchanged = session.worker
            with patch.object(pc, "upstream", side_effect=override_status(path, transform)):
                assert request("/api/pc/recover", {"session_id": session.id})[0] == 409
                assert session.worker is unchanged and not session.recovery_required
        unchanged = session.worker
        with patch.object(pc, "upstream", side_effect=OSError("unresponsive fixture")):
            assert request("/api/pc/recover", {"session_id": session.id})[0] == 409
            assert session.worker is unchanged
        # A response started against an old worker cannot leak into its replacement.
        read_entered, read_release = threading.Event(), threading.Event()
        def delayed_old_read(port, path, *args, **kwargs):
            result = original_upstream(port, path, *args, **kwargs)
            if path == "/api/snapshot" and not read_entered.is_set():
                read_entered.set()
                assert read_release.wait(8)
            return result
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            with patch.object(pc, "upstream", side_effect=delayed_old_read):
                old_read = pool.submit(request, "/api/snapshot")
                try:
                    assert read_entered.wait(2)
                    assert request("/api/pc/recover", {"session_id": session.id})[0] == 200
                finally:
                    read_release.set()
                assert old_read.result()[0] == 409
        assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 200
        assert request("/api/pc/connect", connect)[0] == 200
        # A stuck old worker must block replacement rather than overlap sessions.
        current_worker = session.worker
        current_id = session.id
        with patch.object(pc.subprocess, "Popen", wraps=pc.subprocess.Popen) as spawn, patch.object(current_worker, "terminate"), patch.object(
            current_worker, "wait", side_effect=subprocess.TimeoutExpired("worker", 5)
        ):
            assert request("/api/pc/recover", {"session_id": session.id})[0] == 400
            assert session.worker is current_worker and session.id == current_id
            assert not spawn.called and session.recovery_required
            assert request("/api/control/velocity", velocity)[0] == 409
        assert request("/api/pc/recover", {"session_id": session.id})[0] == 200
        assert request("/api/pc/confirm-recovery", {"session_id": session.id, "confirmed": True})[0] == 200
        assert request("/api/pc/disconnect", {})[0] == 200
        # A manager crash must not leave its DDS worker running.
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            crash_port = reservation.getsockname()[1]
        manager = subprocess.Popen(
            [sys.executable, str(project / "scripts/pc_session.py"),
             "--binary", str(Path(binary).resolve()), "--mock", "--port", str(crash_port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        child = None
        try:
            for _ in range(80):
                try:
                    children = Path(f"/proc/{manager.pid}/task/{manager.pid}/children").read_text().split()
                    if children:
                        child = int(children[0])
                        probe = http.client.HTTPConnection("127.0.0.1", crash_port, timeout=1)
                        probe.request("GET", "/api/pc/session")
                        response = probe.getresponse()
                        ready = response.status == 200
                        response.read()
                        probe.close()
                        if ready:
                            break
                except (OSError, http.client.HTTPException):
                    pass
                time.sleep(0.05)
            else:
                raise AssertionError("Crash fixture manager did not start")
            manager.kill()  # Disposable local Mock fixture only.
            manager.wait(timeout=5)
            for _ in range(60):
                try:
                    process_state = Path(f"/proc/{child}/stat").read_text().split(") ", 1)[1][0]
                    if process_state == "Z":
                        break
                except FileNotFoundError:
                    break
                time.sleep(0.05)
            else:
                raise AssertionError("Mock worker survived manager crash")
        finally:
            if manager.poll() is None:
                manager.terminate()
                manager.wait(timeout=5)
    finally:
        server.shutdown()
        server.server_close()
        session.close()
        thread.join(timeout=5)
    print("PC session, isolation, WebSocket, crash recovery and SDK2 proxy, safety gates and read-only worker API tests passed")


if __name__ == "__main__":
    main(sys.argv[1])
