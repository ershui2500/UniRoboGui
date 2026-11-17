#!/usr/bin/env python3
"""Loopback-only PC session manager. Workers own DDS; no robot SSH is used."""
import argparse
import http.client
import json
import mimetypes
import math
import re
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote


READ_APIS = frozenset({
    "/api/health", "/api/snapshot", "/api/robot/manifest",
    "/api/robot/products", "/api/control/status",
})
DEVICE_READ_APIS = frozenset({
    "/api/voice/status", "/api/voice/llm/customer-config",
    "/api/control/joint-debug/status", "/api/perception/status",
    "/api/camera/status", "/api/camera/rgb/frame.jpg",
    "/api/perception/frame", "/api/perception/global-map",
    "/api/perception/navigation-scene", "/api/perception/topics",
})
WRITE_APIS = frozenset({
    "/api/voice/tts", "/api/voice/asr", "/api/voice/volume",
    "/api/voice/llm/mode", "/api/voice/llm/chat", "/api/voice/llm/customer-config",
    "/api/control/command", "/api/control/velocity",
    "/api/control/joint-debug/apply", "/api/control/joint-debug/stop",
    "/api/control/joint-debug/mode", "/api/control/joint-debug/heartbeat",
    "/api/control/joint-debug/teach/record", "/api/control/joint-debug/teach/save",
    "/api/control/joint-debug/teach/play", "/api/control/joint-debug/teach/delete",
    "/api/control/joint-debug/teach/bind", "/api/perception/command",
    "/api/camera/command",
})
SESSION_APIS = frozenset({"/api/pc/connect", "/api/pc/disconnect", "/api/robot/switch",
                          "/api/pc/recover", "/api/pc/confirm-recovery"})
ROOT_ASSETS = frozenset({
    "index.html", "styles.css", "app.js", "i18n.js", "workspace.js",
    "imu-gauges.js", "robot-viewer.js", "perception.js", "joint-debug.js",
    "motion-policy.js", "battery-panels.js",
})


def interfaces(mock=False):
    result = []
    for _, name in socket.if_nameindex():
        root = Path("/sys/class/net") / name
        try:
            # Physical Ethernet only: exclude wireless, bridges, tunnels and loopback.
            wired = (root / "device").exists() and not (root / "wireless").exists()
            wired = wired and (root / "type").read_text().strip() == "1"
            if not wired and not (mock and name == "lo"):
                continue
            addresses = []
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                import fcntl
                try:
                    data = fcntl.ioctl(sock, 0x8915, struct.pack("256s", name.encode()))
                    addresses.append(socket.inet_ntoa(data[20:24]))
                except OSError:
                    pass
            result.append({"name": name, "up": (root / "operstate").read_text().strip()
                           in ("up", "unknown"), "ipv4": addresses, "wired": wired})
        except OSError:
            continue
    return result


def upstream(port, path, method="GET", body=None):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=35 if method == "POST" else 12 if path == "/api/control/joint-debug/status" else 2)
    try:
        conn.request(method, path, body, {"Content-Type": "application/json"} if body is not None else {})
        response = conn.getresponse()
        return response.status, response.getheader("Content-Type"), response.read()
    finally:
        conn.close()


class Session:
    def __init__(self, binary, web_root, mock=False, a2_joint_gains=None):
        self.binary = str(Path(binary).resolve(strict=True))
        self.web_root = Path(web_root).resolve(strict=True)
        self.mock = mock
        self.a2_joint_gains = str(Path(a2_joint_gains).resolve(strict=True)) if a2_joint_gains else ""
        self.lock = threading.RLock()
        self.worker = None
        self.port = None
        self.product = "g1"
        self.interface = ""
        self.id = ""
        self.error = ""
        self.closing = threading.Event()
        self.inflight_writes = 0
        self.recovery_required = False
        self.worker_readonly = False
        self.recovery_reason = ""
        self.products = []
        try:
            with self.lock:
                self.replace("g1", "")
                _, _, body = upstream(self.port, "/api/robot/products")
                self.products = json.loads(body)["products"]
        except Exception:
            self.close()
            raise

    def stop_worker(self):
        if self.worker is not None:
            if self.worker.poll() is None:
                self.worker.terminate()
                try:
                    self.worker.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.error = "worker_stop_timeout"
                    raise ValueError(self.error)
            self.worker = None
        self.port = None

    def check_stopped(self, recovery=False):
        if not self.interface:
            return
        if self.inflight_writes:
            raise ValueError("pc_request_in_progress")
        if recovery and self.worker_readonly:
            return  # This worker never constructed control/audio/perception adapters.
        if self.worker is None or self.worker.poll() is not None:
            if recovery:
                return
            raise ValueError("pc_stop_state_unavailable")
        try:
            def get(path):
                status, _, body = upstream(self.port, path)
                if status != 200:
                    raise ValueError("pc_stop_state_unavailable")
                return json.loads(body)

            checked_at = time.monotonic()
            snapshot = get("/api/snapshot")
            control_status = get("/api/control/status")
            if not self.worker_readonly:
                joint = get("/api/control/joint-debug/status")
                perception = get("/api/perception/status")
            else:
                joint = {"dds_state": "idle", "teach_state": "idle", "arm_sdk_weight": 0}
                perception = {"control_confirmation_pending": False, "target_set": False}
            control = control_status["control"]
            motion = control["motion"]
            if (motion.get("active") is not False or motion.get("state") != "stopped" or
                    control.get("last_command", {}).get("state") in ("queued", "running")):
                raise ValueError("pc_motion_active")
            if (joint.get("dds_state") != "idle" or
                    joint.get("teach_state") not in ("idle", "stopped") or
                    joint.get("arm_sdk_weight") != 0):
                raise ValueError("pc_joint_control_active")
            if (perception.get("control_confirmation_pending") is not False or
                    perception.get("mode") in ("navigating", "paused", "mapping") or
                    perception.get("target_set") is not False):
                raise ValueError("pc_navigation_active")
            ownership = "recovery_blocked" if recovery else "control_owned"
            if control_status.get(ownership) is not False:
                raise ValueError("pc_control_active")
            velocities = [motion.get(key) for key in ("vx_m_s", "vy_m_s", "vyaw_rad_s")]
            if any(type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 0.001
                   for value in velocities):
                raise ValueError("pc_motion_active")
            if recovery:
                return  # Retire local publishers only; this is never physical stop evidence.
            # Demo telemetry intentionally animates; physical velocity is required only in real mode.
            sources = snapshot["sources"]
            elapsed_ms = (time.monotonic() - checked_at) * 1000
            def fresh(source):
                age = source.get("age_ms")
                return (source.get("received") is True and type(age) in (int, float) and
                        0 <= age and age + elapsed_ms <= 1000)
            if (not fresh(sources["low_state"]) or not fresh(sources["odometry"]) or
                    control.get("sport_state_received") is not True or
                    not isinstance(control.get("sport_state_age_ms"), (int, float)) or
                    not 0 <= control["sport_state_age_ms"] + elapsed_ms <= 1000):
                raise ValueError("pc_stop_state_unavailable")
            if not self.mock and perception.get("mode") in ("localizing", "arrived"):
                ages = [perception.get("info_age_ms"), perception.get("key_info_age_ms")]
                if not any(type(age) in (int, float) and 0 <= age <= 1000 for age in ages):
                    raise ValueError("pc_stop_state_unavailable")
            manifest = get("/api/robot/manifest")
            locomotion = next((item for item in manifest.get("capabilities", [])
                               if item.get("key") == "locomotion"), {})
            exempt_fsms = locomotion.get("parameters", {}).get("disconnect_odometry_exempt_fsm", "").split(",")
            if not self.mock and str(control.get("fsm_id")) not in exempt_fsms:
                # Crouched/passive states report noisy odometry; vertical estimates are not walking.
                velocity = snapshot["odometry"]["velocity_m_s"]
                yaw = snapshot["odometry"]["yaw_speed_rad_s"]
                if (not isinstance(velocity, list) or len(velocity) != 3 or
                        any(type(value) not in (int, float) or not math.isfinite(value)
                            for value in [*velocity[:2], yaw])):
                    raise ValueError("pc_stop_state_unavailable")
                if math.hypot(*velocity[:2]) > 0.03 or abs(yaw) > 0.05:
                    raise ValueError("pc_robot_moving")
        except (OSError, http.client.HTTPException, KeyError, TypeError, json.JSONDecodeError):
            raise ValueError("pc_stop_state_unavailable") from None

    def prepare_stop(self, recovery=False):
        self.check_stopped(recovery)
        if (not self.interface or self.worker_readonly or
                self.worker is None or self.worker.poll() is not None):
            return
        target = "/api/pc/prepare-recovery" if recovery else "/api/pc/prepare-disconnect"
        code, _, body = upstream(self.port, target, "POST", "{}")
        if code != 200 or not json.loads(body).get("accepted"):
            raise ValueError("pc_control_active")
        try:
            self.check_stopped(recovery)
        except Exception:
            upstream(self.port, "/api/pc/resume", "POST", "{}")
            raise

    def replace(self, product, interface, recovery=False, readonly=False):
        # Called under lock: old process must exit before a new DDS session exists.
        self.prepare_stop(recovery)
        if recovery:
            self.recovery_required = True
            self.recovery_reason = "previous_tasks_unknown"
        self.stop_worker()
        self.product, self.interface = product, interface
        self.worker_readonly = readonly
        self.id = uuid.uuid4().hex
        self.error = ""
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        args = [self.binary, "--pc-worker", "--robot", product,
                "--bind", "127.0.0.1", "--port", str(port),
                "--web-root", str(self.web_root)]
        if not interface:
            args.append("--pc-idle")
        elif readonly:
            args.extend(["--interface", interface, "--read-only"])
        else:
            args.extend(["--interface", interface, "--enable-navigation"])
            if product == "a2" and self.a2_joint_gains:
                args.extend(["--a2-joint-gains", self.a2_joint_gains])
        if self.mock:
            args.append("--mock")
        env = dict(os.environ, UNIROBO_PC_SESSION_ID=self.id)
        self.worker = subprocess.Popen(args, env=env, stdin=subprocess.DEVNULL,
                                       close_fds=True)
        self.port = port
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if self.worker.poll() is not None:
                break
            try:
                status, _, body = upstream(port, "/api/robot/manifest")
                manifest = json.loads(body)
                if status == 200 and manifest.get("pc_session_id") == self.id:
                    return
            except (OSError, http.client.HTTPException, ValueError):
                pass
            self.closing.wait(0.05)
        self.error = "worker_start_failed"
        self.stop_worker()
        raise ValueError(self.error)

    def state(self):
        with self.lock:
            alive = self.worker is not None and self.worker.poll() is None
            if not alive and not self.error:
                self.error = "worker_exited"
            info = next((item for item in interfaces(self.mock)
                         if item["name"] == self.interface), None)
            state = "disconnected" if not self.interface else "waiting_telemetry"
            initialized = False
            dds_error = ""
            last_received_ms = None
            arm_action_state = "unknown" if self.interface else "idle"
            if self.error:
                state = "error"
            elif self.interface:
                try:
                    _, _, body = upstream(self.port, "/api/health")
                    health = json.loads(body)
                    initialized = health.get("dds_initialized", False)
                    dds_error = health.get("dds_error", "")
                    low = health.get("sources", {}).get("low_state", {})
                    age = low.get("age_ms")
                    if isinstance(age, (int, float)):
                        last_received_ms = health["server_time_ms"] - age
                    # Health requirements include auxiliary services; LowState proves connectivity.
                    if initialized is True and low.get("received") is True and low.get("status") == "online":
                        state = "online"
                    elif low.get("received") or age is not None:
                        state = "stale"
                    if dds_error and not initialized:
                        self.error = "dds_initialization_failed"
                        state = "error"
                    if not info or not info["up"] or not info["ipv4"]:
                        state = "network_unavailable"
                    if not self.worker_readonly:
                        code, _, raw = upstream(self.port, "/api/control/status")
                        if code == 200:
                            arm_action_state = json.loads(raw).get("arm_action_state", "unknown")
                except (OSError, http.client.HTTPException, ValueError):
                    state = "error"
                if not info or not info["up"] or not info["ipv4"]:
                    state = "network_unavailable"
            return {"runtime_mode": "pc_sdk2", "mock": self.mock,
                    "session_id": self.id, "selected_product_id": self.product,
                    "physical_identity_verified": False,
                    "interface": self.interface, "network": info,
                    "state": state, "dds_initialized": initialized,
                    "dds_error": dds_error,
                    "last_received_ms": last_received_ms, "error": self.error,
                    "worker_alive": alive, "worker_readonly": self.worker_readonly,
                    "recovery_required": self.recovery_required,
                    "recovery_reason": self.recovery_reason,
                    "arm_action_state": arm_action_state,
                    "direct_validation": "mock_only" if self.mock else "pending"}

    def action(self, path, payload):
        with self.lock:
            if path in ("/api/pc/recover", "/api/pc/confirm-recovery"):
                confirm = path == "/api/pc/confirm-recovery"
                if set(payload) != ({"session_id", "confirmed"} if confirm else {"session_id"}):
                    raise ValueError("invalid_parameters")
                if payload["session_id"] != self.id:
                    raise ValueError("pc_session_changed")
                if not self.interface or (confirm and (not self.recovery_required or not self.worker_readonly)):
                    raise ValueError("pc_recovery_required")
                if confirm:
                    if payload["confirmed"] is not True:
                        raise ValueError("confirmation_required")
                    # Read-only telemetry cannot prove old firmware/SLAM tasks ended.
                    try:
                        self.check_stopped()
                    except ValueError as error:
                        if str(error) != "pc_stop_state_unavailable":
                            raise
                        # Field confirmation can retire an observer even while offline.
                    try:
                        self.replace(self.product, "", recovery=True)
                    finally:
                        # Once the confirmed read-only worker exited, an idle-start
                        # failure must not strand a session with no recovery target.
                        if not self.interface:
                            self.recovery_required = False
                            self.recovery_reason = ""
                else:
                    self.replace(self.product, self.interface, recovery=True, readonly=True)
                return {"accepted": True, "session": self.state()}
            if self.recovery_required:
                raise ValueError("pc_recovery_required")
            if path == "/api/pc/disconnect":
                if payload:
                    raise ValueError("invalid_parameters")
                if self.interface or self.error:
                    self.replace(self.product, "")
            elif path == "/api/robot/switch":
                if set(payload) != {"product_id"} or not any(
                    p["product_id"] == payload["product_id"] for p in self.products
                ):
                    raise ValueError("unknown_robot_product")
                if payload["product_id"] != self.product or self.error:
                    self.replace(payload["product_id"], "")
            elif path == "/api/pc/connect":
                if set(payload) != {"product_id", "interface"}:
                    raise ValueError("invalid_parameters")
                if payload["product_id"] not in ("g1", "r1", "go2", "a2"):
                    raise ValueError("pc_product_not_supported")
                info = next((item for item in interfaces(self.mock)
                             if item["name"] == payload["interface"]), None)
                if not info or not info["up"] or not info["ipv4"]:
                    raise ValueError("wired_interface_not_ready")
                if (self.product != payload["product_id"] or self.interface != info["name"] or self.error
                        or self.worker is None or self.worker.poll() is not None):
                    self.replace(payload["product_id"], info["name"])
            else:
                raise ValueError("pc_endpoint_unavailable")
            return {"accepted": True, "restarting": False,
                    "product_id": self.product, "session": self.state()}

    def close(self):
        self.closing.set()
        with self.lock:
            self.stop_worker()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    @property
    def session(self):
        return self.server.session

    def safe_origin(self):
        port = self.server.server_port
        hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        origin = self.headers.get("Origin")
        return self.headers.get("Host") in hosts and (
            origin is None or origin in {f"http://{host}" for host in hosts})

    def send(self, status, body, content_type="application/json; charset=utf-8"):
        if not isinstance(body, bytes):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)
        self.close_connection = True

    def do_POST(self):
        if not self.safe_origin():
            return self.send(403, {"error": "invalid_origin"})
        if self.path not in SESSION_APIS | WRITE_APIS:
            return self.send(403, {"accepted": False, "error": "pc_endpoint_unavailable"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            limit = 1024 if self.path in SESSION_APIS else 1024 * 1024
            heartbeat = self.path == "/api/control/joint-debug/heartbeat" and length == 0
            if (not heartbeat and not 0 < length <= limit) or self.headers.get("Transfer-Encoding"):
                raise ValueError("invalid_request_size")
            if not heartbeat and self.headers.get_content_type() != "application/json":
                raise ValueError("json_required")
            self.connection.settimeout(3)
            payload = {} if heartbeat else json.loads(self.rfile.read(length))
            if not isinstance(payload, dict):
                raise ValueError("invalid_json")
            with self.session.lock:
                if self.session.closing.is_set():
                    return self.send(503, {"accepted": False, "error": "pc_session_closing"})
                if self.path in SESSION_APIS:
                    if self.headers.get("X-Unirobo-PC-Session") != self.session.id:
                        return self.send(409, {"accepted": False, "error": "pc_session_changed"})
                    result = self.session.action(self.path, payload)
                    return self.send(200, result)
                if self.headers.get("X-Unirobo-PC-Session") != self.session.id:
                    return self.send(409, {"accepted": False, "error": "pc_session_changed"})
                if self.session.recovery_required:
                    return self.send(409, {"accepted": False, "error": "pc_recovery_required"})
                if not self.session.interface:
                    return self.send(403, {"accepted": False, "error": "pc_not_connected"})
                port = self.session.port
                self.session.inflight_writes += 1
            try:
                status, kind, body = upstream(port, self.path, "POST", json.dumps(payload))
                return self.send(status, body, kind)
            finally:
                with self.session.lock:
                    self.session.inflight_writes -= 1
        except (ValueError, TypeError) as error:
            code = str(error) if str(error) in {
                "invalid_request_size", "json_required", "invalid_json", "invalid_parameters",
                "unknown_robot_product", "pc_product_not_supported", "wired_interface_not_ready",
                "worker_stop_timeout", "worker_start_failed", "pc_stop_state_unavailable", "pc_control_active",
                "pc_request_in_progress", "pc_motion_active", "pc_joint_control_active",
                "pc_navigation_active", "pc_robot_moving",
                "pc_recovery_required", "pc_session_changed", "confirmation_required",
            } else "invalid_request"
            self.send(409 if code in {"pc_stop_state_unavailable", "pc_control_active", "pc_request_in_progress",
                                     "pc_motion_active", "pc_joint_control_active", "pc_navigation_active", "pc_robot_moving",
                                     "pc_recovery_required", "pc_session_changed"} else 400,
                      {"accepted": False, "error": code})
        except (OSError, http.client.HTTPException):
            self.send(503, {"accepted": False, "error": "worker_unavailable"})

    def do_GET(self):
        if not self.safe_origin():
            return self.send(403, {"error": "invalid_origin"})
        if self.path == "/api/pc/session":
            return self.send(200, self.session.state())
        if self.path == "/api/pc/interfaces":
            return self.send(200, {"interfaces": interfaces(self.session.mock)})
        if self.path == "/ws/telemetry":
            return self.tunnel()
        map_download = re.fullmatch(r"/api/perception/map-file\?map_name=test(?:[1-9]|10)\.pcd", self.path)
        if self.path in READ_APIS | DEVICE_READ_APIS or map_download:
            try:
                with self.session.lock:
                    if self.session.worker is None or self.session.worker.poll() is not None:
                        raise OSError("worker_unavailable")
                    port, session_id = self.session.port, self.session.id
                status, kind, body = upstream(port, self.path)
                with self.session.lock:
                    if self.session.id != session_id:
                        return self.send(409, {"error": "pc_session_changed"})
                    if self.path in ("/api/robot/manifest", "/api/robot/products"):
                        payload = json.loads(body)
                        payload["pc_session"] = self.session.state()
                        if "products" in payload:
                            for product in payload["products"]:
                                product["pc_connect_supported"] = product["product_id"] in ("g1", "r1", "go2", "a2")
                                product["pc_direct_validation"] = "pending"
                        body = json.dumps(payload).encode()
                    return self.send(status, body, kind)
            except (OSError, http.client.HTTPException, ValueError):
                return self.send(503, {"error": "worker_unavailable"})
        if self.path.startswith("/api/"):
            return self.send(403, {"error": "pc_endpoint_unavailable"})
        return self.static_file()

    def static_file(self):
        raw = self.path
        try:
            decoded = unquote(raw, errors="strict")
            relative = "index.html" if decoded == "/" else decoded[1:]
            if (not decoded.startswith("/") or any(c in raw for c in "?#")
                    or "\\" in decoded or "\0" in decoded
                    or any(part in ("", ".", "..") for part in relative.split("/"))
                    or (relative not in ROOT_ASSETS and not relative.startswith("assets/"))):
                raise ValueError("invalid_asset")
            path = (self.session.web_root / relative).resolve(strict=True)
            if not path.is_relative_to(self.session.web_root) or not path.is_file():
                raise ValueError("invalid_asset")
            kind = {".js": "application/javascript", ".urdf": "application/xml",
                    ".stl": "model/stl"}.get(path.suffix)
            kind = kind or mimetypes.guess_type(str(path))[0] or "application/octet-stream"
            return self.send(200, path.read_bytes(), kind)
        except (OSError, ValueError):
            return self.send(404, {"error": "not_found"})

    def tunnel(self):
        if self.headers.get("Upgrade", "").lower() != "websocket":
            return self.send(400, {"error": "websocket_required"})
        try:
            with self.session.lock:
                worker = self.session.worker
                if worker is None or worker.poll() is not None:
                    raise OSError("worker_unavailable")
                peer = socket.create_connection(("127.0.0.1", self.session.port), timeout=3)
            with peer:
                # Only the fixed, read-only telemetry endpoint can be upgraded.
                request = ["GET /ws/telemetry HTTP/1.1", "Host: 127.0.0.1",
                           "Connection: Upgrade", "Upgrade: websocket"]
                for key in ("Sec-WebSocket-Key", "Sec-WebSocket-Version", "Sec-WebSocket-Extensions"):
                    value = self.headers.get(key)
                    if value:
                        request.append(f"{key}: {value}")
                peer.sendall(("\r\n".join(request) + "\r\n\r\n").encode())
                self.connection.settimeout(3)
                while not self.session.closing.is_set() and self.session.worker is worker:
                    ready, _, _ = select.select([peer, self.connection], [], [], 0.25)
                    for source in ready:
                        data = source.recv(65536)
                        if not data:
                            return
                        target = self.connection if source is peer else peer
                        target.sendall(data)
        except (OSError, ValueError):
            pass
        finally:
            self.close_connection = True

    def deny_write(self):
        self.send(403, {"accepted": False, "error": "pc_endpoint_unavailable"})

    do_PUT = deny_write
    do_DELETE = deny_write
    do_PATCH = deny_write


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description="UniRoboGui PC SDK2 session manager")
    project = Path(__file__).resolve().parent.parent
    parser.add_argument("--binary", type=Path, default=project / "build-pc/g1_web_server")
    parser.add_argument("--web-root", type=Path, default=project / "web")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--mock", action="store_true", help="Mock only; no DDS initialization")
    parser.add_argument("--a2-joint-gains", type=Path, help="Operator-provided A2 joint gains file")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    if any(os.environ.get(key) for key in (
        "ROS_DISTRO", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"
    )) or any("/ros/" in os.environ.get(key, "") for key in ("LD_LIBRARY_PATH", "PYTHONPATH")):
        parser.error("Start from a clean terminal without ROS/RMW/CycloneDDS environment")
    # Bind before creating any worker; a second manager cannot create an orphan DDS session.
    with Server(("127.0.0.1", args.port), Handler) as server:
        session = Session(args.binary, args.web_root, args.mock, args.a2_joint_gains)
        server.session = session
        def stop(*_):
            try:
                with session.lock:
                    if not (session.recovery_required and session.worker_readonly):
                        session.prepare_stop()
                    session.closing.set()
            except ValueError as error:
                print("Cannot close / 无法退出: " + str(error) +
                      "; end control tasks and restore fresh telemetry first / 请先结束控制任务并恢复新鲜遥测", flush=True)
                return
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        print(f"UniRoboGui PC: http://127.0.0.1:{args.port} (disconnected)", flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        finally:
            session.close()


if __name__ == "__main__":
    main()
