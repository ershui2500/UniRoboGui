#!/usr/bin/env python3
"""Offline end-to-end A2 console contract; starts only --mock processes."""
import json
import pathlib
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

from test_customer_config_http import (
    open_websocket, read_websocket_text_message, request_json, stop_process,
    wait_for_server,
)


def eventually(check):
    for _ in range(60):
        if check():
            return
        time.sleep(.05)
    raise AssertionError('condition did not become true')


def run(server, web_root, readonly):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    base = f'http://127.0.0.1:{port}'
    with tempfile.TemporaryDirectory() as workdir:
        isolated_web = pathlib.Path(workdir) / "web"
        isolated_web.symlink_to(web_root, target_is_directory=True)
        process = subprocess.Popen(
            [server, '--mock', '--robot', 'a2', '--device', 'jt128', '--bind', '127.0.0.1',
             '--port', str(port), '--web-root', str(isolated_web)] +
            (['--read-only'] if readonly else []), cwd=workdir,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            wait_for_server(base + '/api/health')
            manifest = request_json(base + '/api/robot/manifest')
            assert manifest['identity']['product_id'] == 'a2'
            assert len(manifest['joints']) == 12
            caps = {item['key']: item for item in manifest['capabilities']}
            assert caps['locomotion']['available'] is not readonly
            assert caps['locomotion']['parameters']['speed_modes'] == '0,1'
            assert caps['camera_rgb']['implemented'] and caps['camera_rgb']['available']
            assert caps['camera_rgb']['verification_level'] == 'mock_verified'
            assert caps['camera_rgb']['parameters']['fixed_source'] == 'udp://230.1.1.1:1720'
            assert all(not cap['available'] for key, cap in caps.items()
                       if key not in ('telemetry', 'locomotion', 'joint_debug',
                                      'camera_rgb', 'audio', 'slam', 'lidar', 'joint_teach'))
            assert caps['joint_debug']['implemented']
            assert caps['joint_debug']['available'] is not readonly
            assert caps['joint_debug']['verification_level'] == 'mock_verified'
            assert caps['joint_debug']['parameters']['control_modes'] == 'full_body'
            assert caps['joint_debug']['parameters']['control_topic'] == 'rt/lowcmd'
            assert caps['joint_debug']['parameters']['real_control_policy'] == 'operator_validation'
            assert caps['joint_teach']['implemented'] and caps['joint_teach']['available'] is not readonly
            assert caps['joint_teach']['parameters']['real_control_policy'] == 'operator_validation'
            assert caps['audio']['available'] and caps['audio']['verification_level'] == 'mock_verified'
            assert caps['audio']['parameters']['tts_backends'] == 'kokoro,unitree'
            assert caps['audio']['parameters']['play_stream_transport'] == 'a2_audio_client'
            assert caps['audio']['parameters']['default_tts_backend'] == 'unitree'
            assert caps['audio']['parameters']['builtin_llm'] == 'false'
            assert 'ui_placeholder' not in caps['audio']['parameters']
            assert caps['slam']['available'] is not readonly
            assert caps['slam']['verification_level'] == 'mock_verified'
            assert caps['slam']['parameters']['navigation_max_distance_m'] == '30'
            assert 'ui_placeholder' not in caps['slam']['parameters']
            perception = request_json(base + '/api/perception/status')
            assert perception['navigation_max_distance_m'] == 30
            assert perception['control_confirmation_pending'] is False
            assert perception['initialized'] is True  # Read-only also receives raw preview.
            assert 'ui_placeholder' not in caps['camera_depth']['parameters']
            camera = request_json(base + '/api/camera/status')
            assert camera['fixed_policy'] is True and camera['device']['provider'] == 'unitree_a2'
            assert [stream['role'] for stream in camera['device']['streams']] == ['rgb']
            snapshot = request_json(base + '/api/snapshot')
            assert len(snapshot['batteries']) == 2
            assert snapshot['battery']['soc_pct'] == snapshot['batteries'][0]['soc_pct']
            assert snapshot['sources']['secondary_imu']['received'] is False
            assert snapshot['voice']['initialized'] is True
            assert snapshot['voice']['asr_subscribed'] is True
            assert snapshot['voice']['llm']['mode'] == 'customer'
            tts = request_json(base + '/api/voice/tts',
                               {'text': '你好', 'speaker_id': 0, 'tts_backend': 'unitree'})
            assert tts['accepted'] and tts['tts_backend'] == 'unitree'
            eventually(lambda: request_json(base + '/api/voice/status')['voice']['tts']['state'] == 'succeeded')
            assert request_json(base + '/api/voice/volume', {'volume': 42})['accepted']
            assert request_json(base + '/api/voice/asr', {'enabled': False})['accepted']
            assert request_json(base + '/api/voice/asr', {'enabled': True})['accepted']
            assert len(snapshot['reserved_motor_slots']) == 23
            with open_websocket(port) as websocket:
                frame = json.loads(read_websocket_text_message(websocket.makefile('rb')))
                assert len(frame['batteries']) == 2 and len(frame['joints']) == 12
            root = manifest['model']['asset_root']
            urdf_url = base + root + '/' + manifest['model']['urdf_file']
            with urllib.request.urlopen(urdf_url) as response:
                model = ET.fromstring(response.read())
            movable = {joint.attrib['name'] for joint in model.findall('joint')
                       if joint.attrib['type'] != 'fixed'}
            assert movable == {joint['urdf_joint_name'] for joint in manifest['joints']}
            for mesh in model.findall('.//mesh'):
                url = urllib.parse.urljoin(urdf_url, mesh.attrib['filename'])
                with urllib.request.urlopen(url) as response:
                    assert response.status == 200 and len(response.read()) > 84
            for name in ('motion-policy.js', 'battery-panels.js'):
                with urllib.request.urlopen(base + '/' + name) as response:
                    assert response.status == 200
            if readonly:
                debug = request_json(base + '/api/control/joint-debug/status')
                assert not debug['initialized']
                try:
                    request_json(base + '/api/control/velocity',
                                 {'active': True, 'vx': .1, 'vy': 0, 'vyaw': 0, 'speed_mode': 0})
                    raise AssertionError('readonly accepted motion')
                except urllib.error.HTTPError:
                    pass
                return

            debug = request_json(base + '/api/control/joint-debug/status')
            assert debug['initialized'] and len(debug['joints']) == 12
            assert debug['full_body_allowed'] and not debug['upper_body_allowed']
            assert debug['control_topics'] == {'full_body': 'rt/lowcmd'}
            assert not debug['motion_mode_switch_supported']
            assert debug['remote_binding_supported'] and not debug['teach_actions']
            targets = [
                {'index': index, 'q': .8 if index % 3 == 1 else
                 (-1.5 if index % 3 == 2 else 0)}
                for index in range(12)
            ]
            for endpoint, body, expected in (
                    ('mode', {'mode': 'low_level', 'confirmed': True},
                     'motion_mode_switch_unsupported'),
                    ('apply', {'mode': 'upper_body', 'confirmed': True,
                               'joints': targets},
                     'invalid_control_mode'),
                    ('apply', {'mode': 'full_body', 'confirmed': False,
                               'joints': targets},
                     'confirmation_required')):
                try:
                    request_json(base + '/api/control/joint-debug/' + endpoint, body)
                    raise AssertionError(f'A2 Joint Debug accepted {expected}')
                except urllib.error.HTTPError as error:
                    assert json.loads(error.read())['error'] == expected
            targets[0]['q'] = .1
            applied = request_json(base + '/api/control/joint-debug/apply', {
                'mode': 'full_body', 'confirmed': True, 'joints': targets})
            assert applied['accepted']
            assert request_json(base + '/api/control/joint-debug/heartbeat', {})['accepted']
            assert request_json(base + '/api/control/joint-debug/stop',
                                {'confirmed': True})['accepted']
            eventually(lambda: request_json(
                base + '/api/control/joint-debug/status')['dds_state'] == 'idle')

            def control():
                return request_json(base + '/api/control/status')['control']

            def command(name, fsm):
                reply = request_json(base + '/api/control/command', {
                    'request_key': f'a2-{name}-{time.monotonic_ns()}',
                    'category': 'mode', 'command': name, 'argument': 0, 'confirmed': True})
                assert reply['accepted']
                eventually(lambda: control()['last_command']['state'] == 'succeeded')
                assert control()['fsm_id'] == fsm

            def reject(name):
                try:
                    request_json(base + '/api/control/command', {
                        'request_key': f'reject-{name}', 'category': 'mode',
                        'command': name, 'argument': 0, 'confirmed': True})
                    raise AssertionError('disallowed mode accepted')
                except urllib.error.HTTPError:
                    pass

            reject('left_side_gait_off')
            reject('recovery_stand')
            command('front_flip', 3)
            command('back_flip', 3)
            for name, fsm in [('run', 4), ('climb', 5), ('walk', 3),
                              ('damp', 0), ('recovery_stand', 3),
                              ('stand_down', 1), ('stand_up', 2), ('balance_stand', 3)]:
                command(name, fsm)
            for name, fsm, speed, velocity in [('walk', 3, 0, .8), ('walk', 3, 1, 1.5),
                                              ('run', 4, 0, 2.5), ('run', 4, 1, 4),
                                              ('climb', 5, 0, .6), ('climb', 5, 1, .6)]:
                command(name, fsm)
                result = request_json(base + '/api/control/velocity', {
                    'active': True, 'vx': velocity, 'vy': 0, 'vyaw': 0, 'speed_mode': speed})
                assert result['accepted']
                eventually(lambda: control()['motion']['state'] == 'active')
                # Browser release uses the existing lease and zero-vector stop path.
                request_json(base + '/api/control/velocity', {
                    'active': False, 'vx': 0, 'vy': 0, 'vyaw': 0, 'speed_mode': 0})
                eventually(lambda: control()['motion']['state'] == 'stopped')
            command('walk', 3)
            request_json(base + '/api/control/velocity', {
                'active': True, 'vx': .2, 'vy': 0, 'vyaw': 0, 'speed_mode': 0})
            eventually(lambda: control()['motion']['state'] == 'active')
            eventually(lambda: control()['motion']['state'] == 'stopped')
            assert control()['motion']['active'] is False
            command('left_side_gait_on', 6)
            command('left_side_gait_off', 6)
        finally:
            stop_process(process)


if __name__ == '__main__':
    server = str(pathlib.Path(sys.argv[1]).resolve())
    web_root = str(pathlib.Path(sys.argv[2]).resolve())
    run(server, web_root, False)
    run(server, web_root, True)
    print('A2 HTTP/WebSocket, model assets, control workflow, release/lease and readonly passed')
