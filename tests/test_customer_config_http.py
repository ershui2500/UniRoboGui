#!/usr/bin/env python3
import http.client
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


def request_json(url, payload=None):
    request = urllib.request.Request(
        url,
        data=None if payload is None else json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        return json.load(response)


def wait_for_server(url):
    for _ in range(30):
        try:
            request_json(url)
            return
        except Exception:
            time.sleep(0.1)
    raise AssertionError("mock web server did not become ready")


def wait_for_product(port, product_id):
    url = f"http://127.0.0.1:{port}/api/robot/manifest"
    for _ in range(150):
        try:
            manifest = request_json(url)
            if manifest.get("identity", {}).get("product_id") == product_id:
                return manifest
        except Exception:
            pass
        time.sleep(0.1)
    raise AssertionError(f"mock web server did not switch to {product_id}")


def wait_for_health(port, status="ok"):
    url = f"http://127.0.0.1:{port}/api/health"
    for _ in range(30):
        try:
            health = request_json(url)
            if health.get("status") == status:
                return health
        except Exception:
            pass
        time.sleep(0.1)
    raise AssertionError(f"mock web server health did not become {status}")


def wait_for_camera_frame(base, stream):
    url = f"{base}/api/camera/{stream}/frame.jpg"
    for _ in range(30):
        try:
            with urllib.request.urlopen(url, timeout=3) as response:
                frame = response.read()
                if (
                    response.status == 200
                    and response.headers.get_content_type() == "image/jpeg"
                    and frame.startswith(b"\xff\xd8")
                    and frame.endswith(b"\xff\xd9")
                ):
                    return frame
        except urllib.error.HTTPError as error:
            if error.code != 503:
                raise
        time.sleep(0.1)
    raise AssertionError(f"mock {stream} camera frame did not become fresh")


def stop_process(process):
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def open_websocket(port):
    websocket = socket.create_connection(("127.0.0.1", port), timeout=3)
    websocket.sendall(
        b"GET /ws/telemetry HTTP/1.1\r\n"
        b"Host: 127.0.0.1\r\n"
        b"Upgrade: websocket\r\n"
        b"Connection: Upgrade\r\n"
        b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        b"Sec-WebSocket-Version: 13\r\n\r\n"
    )
    handshake = b""
    while b"\r\n\r\n" not in handshake:
        handshake += websocket.recv(4096)
    assert b"101 Switching Protocols" in handshake
    return websocket


def read_websocket_text_message(reader):
    chunks = []
    first_frame = True
    while True:
        header = reader.read(2)
        assert len(header) == 2
        assert header[0] & 0x70 == 0
        opcode = header[0] & 0x0F
        assert opcode == (1 if first_frame else 0)
        assert header[1] & 0x80 == 0
        length = header[1] & 0x7F
        if length == 126:
            length = int.from_bytes(reader.read(2), "big")
        elif length == 127:
            length = int.from_bytes(reader.read(8), "big")
        payload = reader.read(length)
        assert len(payload) == length
        chunks.append(payload)
        if header[0] & 0x80:
            return b"".join(chunks).decode("utf-8")
        first_frame = False


def assert_public_schema(port):
    base = f"http://127.0.0.1:{port}"

    snapshot = request_json(f"{base}/api/snapshot")
    assert snapshot["schema_version"] == 1
    assert snapshot["application"] == "UniRoboGui"
    assert snapshot["dds_initialized"] is True
    assert snapshot["robot"]["mode_machine_raw"] == 2
    assert snapshot["robot"]["model_supported"] is True
    assert snapshot["robot"]["urdf_file"] == "g1_29dof.urdf"
    assert len(snapshot["joints"]) == 29
    assert len(snapshot["reserved_motor_slots"]) == 6
    assert snapshot["joints"][0]["index"] == 0
    assert snapshot["joints"][0]["name"] == "left_hip_pitch"
    assert snapshot["joints"][28]["index"] == 28
    assert snapshot["joints"][28]["name"] == "right_wrist_yaw"
    assert snapshot["reserved_motor_slots"][0]["index"] == 29
    assert snapshot["reserved_motor_slots"][-1]["index"] == 34
    assert {"hip", "torso"} <= snapshot["imu"].keys()
    assert {"voice", "control", "sources"} <= snapshot.keys()

    manifest = request_json(f"{base}/api/robot/manifest")
    assert manifest["schema_version"] == 1
    assert manifest["identity"]["vendor"] == "unitree"
    assert manifest["identity"]["product_id"] == "g1"
    assert manifest["identity"]["product_id"] != str(
        manifest["diagnostics"]["mode_machine_raw"]
    )
    assert manifest["diagnostics"]["mode_machine_raw"] == 2
    assert manifest["model"] == {
        "supported": True,
        "asset_root": "/assets/unitree/g1_description",
        "package": "g1_description",
        "dof": 29,
        "model_name": "G1 29DOF",
        "urdf_file": "g1_29dof.urdf",
    }
    assert len(manifest["joints"]) == 29
    assert manifest["joints"][0] == {
        "name": "left_hip_pitch",
        "name_zh": "左髋俯仰",
        "urdf_joint_name": "left_hip_pitch_joint",
        "display_group": "left_leg",
        "motor_slot": 0,
    }
    assert manifest["joints"][-1]["name"] == "right_wrist_yaw"
    assert manifest["joints"][-1]["motor_slot"] == 28
    capabilities = {item["key"]: item for item in manifest["capabilities"]}
    assert capabilities["telemetry"]["verification_level"] == "readonly_verified"
    assert capabilities["locomotion"]["verification_level"] == "mock_verified"
    assert capabilities["lidar"]["available"] is True
    assert capabilities["lidar"]["parameters"]["product_supported"] == "true"
    assert capabilities["lidar"]["parameters"]["hardware_presence"] == "present"
    assert capabilities["slam"]["parameters"]["service_available"] == "true"
    assert capabilities["camera_rgb"]["parameters"]["hardware_presence"] == "present"
    assert capabilities["camera_depth"]["parameters"]["hardware_presence"] == "present"
    assert capabilities["head_control"]["available"] is False
    assert capabilities["head_control"]["verification_level"] == "unsupported"
    assert {"voice", "control", "dds_error"}.isdisjoint(manifest.keys())
    assert "api_key" not in json.dumps(manifest)

    health = request_json(f"{base}/api/health")
    assert health["schema_version"] == 1
    assert health["status"] == "ok"
    assert health["motion_control_enabled"] is True
    assert health["voice_tts_enabled"] is True
    assert health["llm_mode"] == "builtin"
    assert health["llm_ready"] is True

    control = request_json(f"{base}/api/control/status")
    assert control["schema_version"] == 1
    assert control["control"]["initialized"] is True
    assert control["control"]["enabled"] is True
    assert control["control"]["mock"] is True
    assert control["control"]["fsm_id"] == 500
    assert control["control"]["motion"]["state"] == "stopped"
    assert control["control"]["motion"]["active"] is False

    voice = request_json(f"{base}/api/voice/status")
    assert voice["schema_version"] == 1
    assert voice["voice"]["initialized"] is True
    assert voice["voice"]["asr_subscribed"] is True
    assert voice["voice"]["llm"]["mode"] == "builtin"

    joint_debug = request_json(f"{base}/api/control/joint-debug/status")
    assert joint_debug["schema_version"] == 1
    assert joint_debug["mock"] is True
    assert joint_debug["mode_machine"] == 2
    assert len(joint_debug["joints"]) == 29

    perception = request_json(f"{base}/api/perception/status")
    assert perception["schema_version"] == 1
    assert perception["mock"] is True
    assert perception["navigation_enabled"] is True
    assert {"services", "topics", "point_filter"} <= perception.keys()

    perception_frame = request_json(f"{base}/api/perception/frame")
    assert perception_frame["schema_version"] == 3
    assert perception_frame["live_points_encoding"] == "base64_u16le_xyz"

    global_map = request_json(f"{base}/api/perception/global-map")
    assert global_map["schema_version"] == 3
    assert global_map["points_encoding"] == "base64_u16le_xyz"

    camera = request_json(f"{base}/api/camera/status")
    assert camera["schema_version"] == 1
    assert camera["mock"] is True
    assert camera["bandwidth_profile"] == "wifi_low_bandwidth"
    assert {"rgb", "depth", "first_person_service"} <= camera.keys()

    with socket.create_connection(("127.0.0.1", port), timeout=3) as websocket:
        websocket.sendall(
            b"GET /ws/telemetry HTTP/1.1\r\n"
            b"Host: 127.0.0.1\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
            b"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        reader = websocket.makefile("rb")
        assert b"101 Switching Protocols" in reader.readline()
        while reader.readline() not in (b"\r\n", b""):
            pass
        telemetry = json.loads(read_websocket_text_message(reader))
        assert telemetry["schema_version"] == 1
        assert telemetry["robot"]["mode_machine_raw"] == 2
        assert len(telemetry["joints"]) == 29
        assert len(telemetry["reserved_motor_slots"]) == 6


def assert_r1_schema(port):
    base = f"http://127.0.0.1:{port}"
    health = wait_for_health(port)
    assert health["motion_control_enabled"] is True
    assert health["voice_tts_enabled"] is True

    snapshot = request_json(f"{base}/api/snapshot")
    assert snapshot["robot"]["mode_machine_raw"] == 1
    assert snapshot["robot"]["urdf_file"] == "R1.urdf"
    assert len(snapshot["joints"]) == 26
    assert len(snapshot["reserved_motor_slots"]) == 5
    assert [joint["index"] for joint in snapshot["joints"][-2:]] == [29, 30]
    assert [joint["name"] for joint in snapshot["joints"][-2:]] == ["head_pitch", "head_yaw"]
    assert [slot["index"] for slot in snapshot["reserved_motor_slots"]] == [14, 20, 21, 27, 28]

    manifest = request_json(f"{base}/api/robot/manifest")
    assert manifest["identity"]["product_id"] == "r1"
    assert manifest["identity"]["display_name"] == "Unitree R1"
    assert manifest["model"]["dof"] == 26
    assert manifest["model"]["asset_root"] == "/assets/unitree/r1_description"
    assert manifest["model"]["urdf_file"] == "R1.urdf"
    assert manifest["joints"][-2]["display_group"] == "head"
    assert manifest["joints"][-1]["display_group"] == "head"
    capabilities = {item["key"]: item for item in manifest["capabilities"]}
    assert capabilities["telemetry"]["available"] is True
    assert capabilities["telemetry"]["verification_level"] == "readonly_verified"
    for key in ("locomotion", "audio"):
        assert capabilities[key]["available"] is True
        assert capabilities[key]["verification_level"] == "mock_verified"
    for key in ("joint_debug", "head_control"):
        assert capabilities[key]["available"] is True
        assert capabilities[key]["verification_level"] == "mock_verified"
        assert capabilities[key]["parameters"]["real_control_policy"] == "operator_validation"
    assert capabilities["joint_debug"]["parameters"]["upper_body_topic"] == "rt/arm_sdk"
    assert capabilities["joint_debug"]["parameters"]["full_body_topic"] == "rt/lowcmd"
    assert capabilities["joint_debug"]["parameters"]["arm_action_ids"] == (
        "11,12,13,15,17,18,19,22,23,24,25,26,27,28,29,30,31,33,34,35,36,99"
    )
    assert capabilities["joint_debug"]["parameters"]["firmware_teach_actions"] == "true"
    assert capabilities["joint_debug"]["parameters"]["arm_action_interrupts"] == "true"
    assert capabilities["joint_debug"]["parameters"]["arm_action_service"] == "arm"
    assert capabilities["joint_debug"]["parameters"]["arm_action_api_ids"] == "7106,7107,7108,7113"
    assert capabilities["head_control"]["parameters"]["control_topic"] == "rt/arm_sdk"
    assert capabilities["joint_teach"]["available"] is True
    assert capabilities["joint_teach"]["verification_level"] == "mock_verified"
    assert capabilities["joint_teach"]["parameters"]["control_topic"] == "rt/arm_sdk"
    assert capabilities["joint_teach"]["parameters"]["remote_binding"] == "true"
    assert capabilities["joint_teach"]["parameters"]["real_control_policy"] == "operator_validation"
    assert capabilities["locomotion"]["parameters"]["allowed_fsm"] == "811"
    assert capabilities["locomotion"]["parameters"]["speed_modes"] == "0,1,3"
    assert capabilities["locomotion"]["parameters"]["speed_presets"] == (
        "0:0.40:0.40:0.90,1:0.50:0.50:1.00,3:1.00:0.60:1.20"
    )
    assert capabilities["locomotion"]["parameters"]["web_mode_commands"] == (
        "zero_torque,damp,stand_up,lie_to_stand,stand_to_lie,start"
    )
    assert capabilities["locomotion"]["parameters"]["mode_targets"] == (
        "zero_torque:0,damp:1,stand_up:4,lie_to_stand:701,stand_to_lie:702,start:811"
    )
    assert capabilities["locomotion"]["parameters"]["mode_sources"] == (
        "zero_torque:1;stand_up:1|811;start:4;lie_to_stand:4|702;stand_to_lie:811"
    )
    assert capabilities["locomotion"]["parameters"]["fsm_mode_unknown"] == "4294967295"
    for key in ("camera_rgb", "camera_depth"):
        assert capabilities[key]["available"] is True
        assert capabilities[key]["verification_level"] == "mock_verified"
        assert capabilities[key]["parameters"]["receiver_start_allowed_when_unavailable"] == "true"
        assert capabilities[key]["parameters"]["external_preparation_ready"] == "true"
    assert capabilities["camera_rgb"]["parameters"]["port"] == "5003"
    assert capabilities["camera_rgb"]["parameters"]["img_port"] == "5001"
    assert capabilities["camera_rgb"]["parameters"]["output"] == "480x360"
    assert capabilities["camera_depth"]["parameters"]["fixed_source"] == "/dev/video-dep"
    assert capabilities["lidar"]["available"] is False
    assert capabilities["lidar"]["verification_level"] == "unsupported"
    assert capabilities["slam"]["available"] is False
    assert capabilities["slam"]["verification_level"] == "mock_verified"
    assert capabilities["slam"]["reason"] == "required_attachment_disabled"
    assert capabilities["slam"]["parameters"]["required_devices"] == "mid360"
    assert capabilities["slam"]["parameters"]["raw_input_required"] == "false"

    control = request_json(f"{base}/api/control/status")["control"]
    assert control["initialized"] is True
    assert control["fsm_id"] == 811
    assert control["fsm_mode"] == 0
    action_list = json.loads(control["action_list_raw"])
    assert [item["id"] for item in action_list[0]] == [
        99, 11, 12, 13, 15, 17, 18, 19, 22, 23, 24,
        25, 26, 27, 28, 29, 30, 31, 33, 34, 35, 36,
    ]
    assert [item["name"] for item in action_list[1]] == ["r1_demo"]

    joint_debug = request_json(f"{base}/api/control/joint-debug/status")
    assert joint_debug["initialized"] is True
    assert joint_debug["fsm_id"] == 811
    assert joint_debug["upper_body_allowed"] is True
    assert joint_debug["control_topics"]["upper_body"] == "rt/arm_sdk"
    assert "head" not in joint_debug["control_topics"]
    assert joint_debug["control_topics"]["full_body"] == "rt/lowcmd"
    head_joints = {
        joint["name"]: joint for joint in joint_debug["joints"]
        if joint["display_group"] == "head"
    }
    assert set(head_joints) == {"head_pitch", "head_yaw"}
    assert all(joint["control_modes"] == ["upper_body", "full_body"]
               for joint in head_joints.values())
    joints = {joint["name"]: joint for joint in joint_debug["joints"]}
    assert joints["waist_yaw"]["control_modes"] == ["upper_body", "full_body"]
    assert joints["waist_roll"]["control_modes"] == ["full_body"]
    assert joint_debug["remote_binding_supported"] is True
    assert joint_debug["remote_control_ready"] is True
    assert len(joint_debug["remote_binding_options"]) == 16

    camera = request_json(f"{base}/api/camera/status")
    assert camera["schema_version"] == 1
    assert camera["mock"] is True
    assert camera["fixed_policy"] is True
    assert camera["provider"] == "unitree_r1_edu_stereo"
    assert camera["service_version_status"] == "external_check_required"
    assert camera["device"]["manage_external_services"] is True
    assert camera["device"]["manage_privileged_receiver"] is True
    assert camera["external_services"]["depth_receiver"]["helper"] == "/usr/local/sbin/r1-web-camera-service"
    assert camera["external_services"]["depth_receiver"]["started_by_web"] is False
    assert camera["first_person_service"]["managed_by_web"] is False
    assert camera["first_person_service"]["paused_by_web"] is False
    streams = {item["role"]: item for item in camera["device"]["streams"]}
    assert streams["rgb"]["port"] == 5003
    assert streams["rgb"]["width"] == 544
    assert streams["rgb"]["height"] == 448
    assert streams["rgb"]["fps"] == 10
    assert streams["rgb_left"]["port"] == 5002
    assert streams["rgb_right"]["port"] == 5003
    assert streams["depth"]["fixed_source"] == "/dev/video-dep"
    assert streams["depth"]["width"] == 544
    assert streams["depth"]["height"] == 448
    assert streams["depth"]["fps"] == 10

    for stream in ("rgb", "depth"):
        wait_for_camera_frame(base, stream)

    for payload, expected_error in (
        ({
            "request_key": "r1-http-source-override",
            "command": "start_v4l2",
            "confirmed": True,
            "rgb_source": "/dev/video0",
        }, "camera_source_override_forbidden"),
        ({
            "request_key": "r1-http-pipeline-override",
            "command": "start_v4l2",
            "confirmed": True,
            "pipeline": "udpsrc port=9999 ! fakesink",
        }, "camera_override_forbidden"),
        ({
            "request_key": "r1-http-service-override",
            "command": "start_v4l2",
            "confirmed": True,
            "service_name": "video_hub",
        }, "camera_override_forbidden"),
    ):
        try:
            request_json(f"{base}/api/camera/command", payload)
            raise AssertionError("malicious R1 camera override unexpectedly accepted")
        except urllib.error.HTTPError as error:
            assert error.code == 400
            body = json.loads(error.read().decode("utf-8"))
            assert body["accepted"] is False
            assert body["error"] == expected_error


def assert_transport_behavior(port):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    connection.request("GET", "/api/health")
    health = connection.getresponse()
    health.read()
    assert not health.will_close
    assert health.getheader("Cache-Control") == "no-store"
    first_socket = connection.sock

    connection.request("GET", "/api/control/status")
    control = connection.getresponse()
    control.read()
    assert not control.will_close
    assert connection.sock is first_socket

    connection.request("GET", "/assets/vendor/three/three.module.js")
    asset = connection.getresponse()
    asset.read()
    assert asset.getheader("Cache-Control") == "public, max-age=31536000, immutable"

    connection.request("GET", "/")
    page = connection.getresponse()
    page.read()
    assert page.getheader("Cache-Control") == "no-cache"
    connection.close()

    with socket.create_connection(("127.0.0.1", port), timeout=3) as websocket:
        websocket.sendall(
            b"GET /ws/telemetry HTTP/1.1\r\n"
            b"Host: 127.0.0.1\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
            b"Sec-WebSocket-Version: 13\r\n"
            b"Sec-WebSocket-Extensions: permessage-deflate\r\n\r\n"
        )
        handshake = b""
        while b"\r\n\r\n" not in handshake:
            handshake += websocket.recv(4096)
        assert b"101 Switching Protocols" in handshake
        assert b"Sec-WebSocket-Extensions: permessage-deflate" in handshake


def main():
    server, web_root = sys.argv[1:3]

    help_result = subprocess.run(
        [server, "--help"], capture_output=True, text=True, timeout=3)
    assert help_result.returncode == 0
    assert "--robot <产品>" in help_result.stdout
    assert "缺省为 g1" in help_result.stdout

    unknown = subprocess.run(
        [server, "--mock", "--robot", "unknown"],
        capture_output=True,
        text=True,
        timeout=3,
    )
    assert unknown.returncode != 0
    assert "未注册机器人产品: unknown" in unknown.stderr

    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        explicit_port = sock.getsockname()[1]
    with tempfile.TemporaryDirectory() as workdir:
        explicit = subprocess.Popen(
            [server, "--mock", "--robot", "g1", "--bind", "127.0.0.1",
             "--port", str(explicit_port), "--web-root", web_root],
            cwd=workdir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            wait_for_server(f"http://127.0.0.1:{explicit_port}/api/health")
            products = request_json(
                f"http://127.0.0.1:{explicit_port}/api/robot/products")
            assert products["active_product_id"] == "g1"
            assert [item["product_id"] for item in products["products"]] == ["g1", "r1"]

            websocket = open_websocket(explicit_port)
            switched = request_json(
                f"http://127.0.0.1:{explicit_port}/api/robot/switch",
                {"product_id": "r1"},
            )
            assert switched == {
                "accepted": True,
                "product_id": "r1",
                "restarting": True,
            }
            websocket.settimeout(0.5)
            websocket_closed = False
            deadline = time.time() + 3
            while time.time() < deadline and not websocket_closed:
                try:
                    websocket_closed = websocket.recv(4096) == b""
                except socket.timeout:
                    pass
                except OSError:
                    websocket_closed = True
            websocket.close()
            assert websocket_closed, "robot exec must close pre-switch WebSocket connections"
            wait_for_product(explicit_port, "r1")
            assert explicit.poll() is None
            assert_r1_schema(explicit_port)
            products = request_json(
                f"http://127.0.0.1:{explicit_port}/api/robot/products")
            assert products["active_product_id"] == "r1"

            switched_back = request_json(
                f"http://127.0.0.1:{explicit_port}/api/robot/switch",
                {"product_id": "g1"},
            )
            assert switched_back["accepted"] is True
            assert switched_back["restarting"] is True
            wait_for_product(explicit_port, "g1")
            assert explicit.poll() is None
        finally:
            stop_process(explicit)

    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    with tempfile.TemporaryDirectory() as workdir:
        process = subprocess.Popen(
            [server, "--mock", "--bind", "127.0.0.1", "--port", str(port),
             "--web-root", web_root],
            cwd=workdir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            url = f"http://127.0.0.1:{port}/api/voice/llm/customer-config"
            wait_for_server(url)

            assert_transport_behavior(port)
            assert_public_schema(port)

            initial = {
                "api_url": "https://example.test/v1",
                "proxy_url": "http://192.0.2.1:7890",
                "api_key": "test-key-1",
                "preserve_api_key": False,
                "model": "model-a",
                "role_prompt": "role",
                "wake_word": "wake",
                "wake_enabled": False,
                "tts_backend": "kokoro",
                "qa_entries": [
                    {"question": "Q1", "answer": "A1"},
                    {"question": "Q2", "answer": "A2"},
                ],
            }
            invalid_proxy = dict(initial)
            invalid_proxy["proxy_url"] = "socks5://192.0.2.1:1080"
            try:
                request_json(url, invalid_proxy)
                raise AssertionError("invalid customer proxy URL was accepted")
            except urllib.error.HTTPError as error:
                assert error.code == 400
                assert json.load(error)["error"] == "invalid_customer_proxy_url"
            assert request_json(url, initial)["accepted"]

            tts = request_json(
                f"http://127.0.0.1:{port}/api/voice/tts",
                {"text": "你好 Hello", "speaker_id": -1, "tts_backend": "kokoro"},
            )
            assert tts["accepted"]
            assert tts["tts_backend"] == "kokoro"

            after_delete = dict(initial)
            after_delete.update({
                "api_key": "",
                "preserve_api_key": True,
                "qa_entries": [{"question": "Q2", "answer": "A2"}],
            })
            saved = request_json(url, after_delete)
            assert saved["accepted"]
            assert saved["qa_entries"] == [{"question": "Q2", "answer": "A2"}]

            loaded = request_json(url)
            assert loaded["qa_delete_semantics"] is True
            assert loaded["qa_entries"] == [{"question": "Q2", "answer": "A2"}]
            assert loaded["api_url"] == "https://example.test/v1/chat/completions"
            assert loaded["proxy_url"] == "http://192.0.2.1:7890"
            assert loaded["model"] == "model-a"
            assert loaded["api_key_configured"] is True
            assert loaded["api_key"] != "test-key-1"

            replacement = dict(after_delete)
            replacement.update({
                "api_url": "https://replacement.test/v1",
                "api_key": "test-key-2",
                "preserve_api_key": False,
                "model": "model-b",
            })
            assert request_json(url, replacement)["accepted"]

            mode_url = f"http://127.0.0.1:{port}/api/voice/llm/mode"
            builtin_mode = request_json(mode_url, {"mode": "builtin"})
            assert builtin_mode["accepted"] and builtin_mode["mode"] == "builtin"
            customer_mode = request_json(mode_url, {"mode": "customer"})
            assert customer_mode["accepted"] and customer_mode["mode"] == "customer"

            with open(os.path.join(workdir, "config", "customer_voice.json"),
                      encoding="utf-8") as stream:
                disk = json.load(stream)
            assert disk["qa_entries"] == [{"question": "Q2", "answer": "A2"}]
            assert disk["api_url"] == "https://replacement.test/v1/chat/completions"
            assert disk["proxy_url"] == "http://192.0.2.1:7890"
            assert disk["model"] == "model-b"
            assert disk["api_key"] == "test-key-2"
        finally:
            stop_process(process)


if __name__ == "__main__":
    main()
