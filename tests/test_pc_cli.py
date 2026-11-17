#!/usr/bin/env python3
"""Test PC installer dispatch with fake build tools; no packages/services changed."""
import json
import os
from pathlib import Path
import subprocess
import tempfile


project = Path(__file__).resolve().parent.parent
revision = "7740f8b67e386ab09c3b333187fd5f8582a75ddc"
with tempfile.TemporaryDirectory(prefix="unirobo pc cli ") as temporary:
    root = Path(temporary)
    tools = root / "bin"
    tools.mkdir()
    prefix = root / "SDK with spaces"
    build = root / "build with spaces"
    prefix.mkdir()
    (prefix / ".unirobo-sdk-revision").write_text(revision + "\n")
    for relative in ("lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake",
                     "include/unitree/robot/r1/audio/audio_client.hpp",
                     "include/unitree/robot/a2/sport/sport_client.hpp",
                     "lib/libunitree_sdk2.a", "lib/libddsc.so", "lib/libddscxx.so"):
        path = prefix / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    build.mkdir()
    (build / "g1_web_server").touch(mode=0o755)
    log = root / "calls"
    package_log = root / "packages"
    fixture = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
name = Path(sys.argv[0]).name
if name == 'dpkg-query':
    with open(os.environ['PC_TEST_PACKAGE_LOG'], 'a') as f: f.write(sys.argv[-1] + '\\n')
    print('unknown ok not-installed' if sys.argv[-1] == os.environ.get('PC_TEST_MISSING_PACKAGE')
          else 'install ok installed', end='')
elif name == 'uname':
    print('x86_64')
elif name == 'ldd':
    for lib in ('libddsc.so', 'libddscxx.so'):
        print(lib + ' => ' + os.environ['PC_TEST_PREFIX'] + '/lib/' + lib + ' (0x0)')
else:
    with open(os.environ['PC_TEST_LOG'], 'a') as f: f.write(json.dumps([name] + sys.argv[1:]) + '\\n')
'''
    for name in ("dpkg-query", "uname", "cmake", "ctest", "ldd", "sudo", "ssh", "systemctl", "python3"):
        path = tools / name
        # Fake python is only used by start; other fixture shebangs use the real interpreter.
        path.write_text(fixture.replace("#!/usr/bin/env python3", "#!/usr/bin/python3", 1))
        path.chmod(0o755)
    env = dict(os.environ, PATH=str(tools) + ":" + os.environ["PATH"],
               PC_TEST_PREFIX=str(prefix), PC_TEST_LOG=str(log), PC_TEST_PACKAGE_LOG=str(package_log))
    for key in ("ROS_DISTRO", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        env.pop(key, None)

    def run(command, extra=(), overrides=None):
        return subprocess.run(["bash", str(project / "scripts/pc.sh"), command,
                               "--sdk-prefix", str(prefix), "--build-dir", str(build), *extra],
                              env=env | (overrides or {}), capture_output=True, text=True)

    assert run("check").returncode == 0
    video_packages = {"gstreamer1.0-tools", "gstreamer1.0-plugins-good",
                      "gstreamer1.0-plugins-bad", "gstreamer1.0-libav"}
    assert video_packages <= set(package_log.read_text().splitlines()), "PC preflight must check every RTP/H264 pipeline dependency"
    assert not log.exists(), "check must not invoke mutating commands"
    for _ in range(2):
        result = run("install")
        assert result.returncode == 0, result.stderr
    assert (build / "pc-sdk-prefix").read_text().strip() == str(prefix)
    calls = log.read_text()
    assert 'sudo' not in calls and 'systemctl' not in calls and 'ssh' not in calls
    assert '-DUNIROBO_PC_BUILD=ON' in calls and '-DCMAKE_BUILD_TYPE=Release' in calls
    assert run("start", ("--mock", "--port", "18080")).returncode == 0
    calls = log.read_text()
    assert 'pc_session.py' in calls and '--mock' in calls
    gains = root / "operator gains.json"
    gains.write_text("{}")
    assert run("start", ("--a2-joint-gains", str(gains))).returncode == 0
    assert '--a2-joint-gains' in log.read_text()
    assert run("start", ("--a2-joint-gains", str(root / "missing.json"))).returncode != 0
    assert run("start", ("--port", "0")).returncode != 0
    assert run("start", ("--port", "65536")).returncode != 0
    assert run("check", overrides={"ROS_DISTRO": "humble"}).returncode != 0
    for package in sorted(video_packages):
        before = log.read_text()
        result = run("check", overrides={"PC_TEST_MISSING_PACKAGE": package})
        assert result.returncode != 0 and package in result.stderr
        assert log.read_text() == before, "preflight must report missing video packages without mutating the system"
        assert run("install", overrides={"PC_TEST_MISSING_PACKAGE": package}).returncode == 0
        commands = [json.loads(line) for line in log.read_text()[len(before):].splitlines()]
        assert ["sudo", "apt-get", "install", "-y", "--", package] in commands
    (prefix / ".unirobo-sdk-revision").write_text("unknown\n")
    assert run("check").returncode != 0
    assert run("install").returncode != 0
print("PC installer, repeat install, paths with spaces, preflight and startup tests passed")
