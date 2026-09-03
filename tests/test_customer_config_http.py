#!/usr/bin/env python3
import http.client
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
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
            for _ in range(30):
                try:
                    request_json(url)
                    break
                except Exception:
                    time.sleep(0.1)
            else:
                raise AssertionError("mock web server did not become ready")

            assert_transport_behavior(port)

            initial = {
                "api_url": "https://example.test/v1",
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
            assert disk["model"] == "model-b"
            assert disk["api_key"] == "test-key-2"
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)


if __name__ == "__main__":
    main()
