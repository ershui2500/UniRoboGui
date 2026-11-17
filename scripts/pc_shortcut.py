#!/usr/bin/env python3
"""Create user desktop entries and launch the existing PC workbench."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

PROJECT = Path(__file__).resolve().parent.parent


def message(zh, en):
    return en if os.environ.get("UNIROBOGUI_LANG", "zh") == "en" else zh


def desktop_argument(value):
    # Desktop Entry string escaping precedes Exec argument escaping; % is a field code.
    if any(ord(char) < 32 for char in value):
        raise ValueError("Shortcut arguments cannot contain control characters / 快捷方式参数不能包含控制字符")
    return '"' + value.replace('\\', r'\\\\').replace('"', r'\\"').replace(
        '$', r'\\$').replace('`', r'\\`').replace('%', '%%') + '"'


def start_arguments(args):
    values = ["--build-dir", str(args.build_dir.resolve()), "--port", str(args.port)]
    if args.mock:
        values.append("--mock")
    if args.a2_joint_gains:
        values.extend(["--a2-joint-gains", str(args.a2_joint_gains.resolve(strict=True))])
    return values


def create(args):
    if not shutil.which("xdg-open"):
        raise ValueError("Desktop requires xdg-open / 桌面入口需要 xdg-open（xdg-utils）")
    command = ["env", "UNIROBOGUI_LANG=" + os.environ.get("UNIROBOGUI_LANG", "zh"),
               sys.executable, str(Path(__file__).resolve()), "launch", *start_arguments(args)]
    contents = ("[Desktop Entry]\nType=Application\nName=UniRoboGui PC\n"
                "Name[zh_CN]=UniRoboGui PC 工作台\n"
                "Comment=Optional PC mode; robot installation offers the fullest feature set\n"
                "Comment[zh_CN]=可选 PC 方式；推荐机器人安装以使用最完整功能\n"
                "Terminal=true\nCategories=Development;Science;Robotics;\n"
                "Exec=" + " ".join(desktop_argument(value) for value in command) + "\n")
    application_dir = Path(os.environ.get("XDG_DATA_HOME", str(Path.home() / ".local/share"))) / "applications"
    application_dir.mkdir(parents=True, exist_ok=True)
    destinations = [application_dir / "unirobogui-pc.desktop"]
    if shutil.which("xdg-user-dir"):
        result = subprocess.run(["xdg-user-dir", "DESKTOP"], capture_output=True, text=True, check=False)
        desktop = Path(result.stdout.strip())
        if result.returncode == 0 and desktop.is_absolute() and desktop.is_dir() and desktop != Path.home():
            destinations.append(desktop / "unirobogui-pc.desktop")
    for destination in destinations:
        destination.write_text(contents, encoding="utf-8")
        destination.chmod(0o755)
        print(message("已创建：", "Created: ") + str(destination))
    print(message("首次双击时，桌面环境可能要求选择“允许启动”。移动项目后请重新创建快捷方式。",
                  "Your desktop may require Allow Launching on first use. Recreate shortcuts after moving the project."))


def probe(port):
    """Return absent, ready or occupied; never use an HTTP proxy for loopback."""
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1):
            pass
    except ConnectionRefusedError:
        return "absent"
    except OSError:
        return "occupied"
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(f"http://127.0.0.1:{port}/api/pc/session", timeout=2) as response:
            data = json.load(response)
        return "ready" if isinstance(data, dict) and data.get("runtime_mode") == "pc_sdk2" else "occupied"
    except (OSError, ValueError, urllib.error.URLError):
        return "occupied"


def open_browser(port):
    url = f"http://127.0.0.1:{port}"
    try:
        subprocess.run(["xdg-open", url], check=True, timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        print(message("无法自动打开浏览器，请手动访问：", "Open your browser manually: ") + url, flush=True)


def wait_for_exit(child):
    # Keep the terminal attached if the session refuses shutdown due to active control.
    while True:
        try:
            return child.wait()
        except KeyboardInterrupt:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)


def launch(args):
    lock_dir = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "unirobogui"
    lock_dir.mkdir(parents=True, exist_ok=True)
    with (lock_dir / f"pc-launch-{args.port}.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError(message("工作台正在启动，请稍后重试。", "Workbench is starting; retry shortly."))
        state = probe(args.port)
        if state == "ready":
            print(message("打开已运行的 PC 工作台（沿用其当前配置）。", "Opening the running PC workbench with its current settings."))
            open_browser(args.port)
            return 0
        if state != "absent":
            raise ValueError(message("端口被其他或无响应服务占用，未启动新进程。", "Port occupied by another or unresponsive service; no process started."))
        child = subprocess.Popen(["bash", str(PROJECT / "scripts/pc.sh"), "start", *start_arguments(args)], cwd=PROJECT)
        try:
            deadline = time.monotonic() + 30
            while child.poll() is None and time.monotonic() < deadline:
                if probe(args.port) == "ready":
                    # A competing launcher may have bound first. A failed new manager must exit.
                    time.sleep(0.2)
                    if child.poll() is None:
                        open_browser(args.port)
                    break
                time.sleep(0.2)
            else:
                if child.poll() is None:
                    print(message("工作台暂未就绪，请检查本终端输出；进程仍由本终端管理。",
                                  "Workbench is not ready; inspect this terminal. The process remains attached."), flush=True)
        except KeyboardInterrupt:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)
        return wait_for_exit(child)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("create", "launch"))
    parser.add_argument("--build-dir", type=Path, default=PROJECT / "build-pc")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--a2-joint-gains", type=Path)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535 / 端口必须为 1–65535")
    try:
        result = create(args) if args.command == "create" else launch(args)
        if result:
            raise ValueError(message(f"PC 启动失败，退出码 {result}。", f"PC startup failed (exit {result})."))
        return 0
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        if args.command == "launch" and sys.stdin.isatty():
            try:
                input(message("按回车关闭此窗口…", "Press Enter to close…"))
            except (EOFError, KeyboardInterrupt):
                pass
        return 1


if __name__ == "__main__":
    sys.exit(main())
