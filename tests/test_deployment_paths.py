#!/usr/bin/env python3
"""Path regression checks; all builds, transfers and service operations are mocked."""
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('render_unit', ROOT / 'scripts/render_deploy_unit.py')
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


class DeploymentPaths(unittest.TestCase):
    def bash(self, script, *args, env=None):
        prefix = 'source "$1/scripts/deploy_i18n.sh"; source "$1/scripts/deploy_product.sh"; shift; '
        env = dict(os.environ if env is None else env)
        for name in ['INSTALL_DIR', 'SDK_PREFIX', 'SDK2_SOURCE', 'REALSENSE_SOURCE',
                     'REALSENSE_PREFIX', 'INSTALL_SDK2']:
            env.pop('UNIROBOGUI_' + name, None)
        return subprocess.run(['bash', '-eu', '-c', prefix + script, 'test', str(ROOT), *args],
                              env=env, text=True, capture_output=True)

    def test_normalization_and_shell_quoting(self):
        for value in ["/tmp/客户 apps/a'b/$x`echo bad`%", '/tmp/a/../b/', '/tmp/double"quote']:
            result = self.bash('value="$(normalize_deploy_path "$1")"; q="$(remote_quote "$value")"; sh -c "printf \'%s\' $q"', value)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, os.path.normpath(value))
        for value in ['', '/', '/tmp/..', 'relative', '/tmp/a:b', '/tmp/a;b', '/tmp/a\nb', '/tmp/a\x01']:
            result = self.bash('normalize_deploy_path "$1"', value)
            self.assertNotEqual(result.returncode, 0, value)

    def test_option_rejection_precedes_network(self):
        for option in ['--install-dir', '--sdk-prefix', '--sdk2-source', '--realsense-source', '--realsense-prefix']:
            for args in [[option], [option, '/'], [option, 'relative']]:
                result = subprocess.run(['bash', str(ROOT / 'scripts/deploy.sh'), 'from-pc',
                                         '--product', 'a2', *args], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2, result.stderr)

    def test_render_all_products_and_scopes(self):
        project = '/srv/客户 robot/Uni"Robo$Gui%'
        sdk = '/opt/SDK customer'
        for template in sorted((ROOT / 'deploy').glob('*.service')):
            if 'web-control' not in template.name and 'local-tts' not in template.name:
                continue
            result = renderer.render(template.read_text(), project, '/home/developer',
                                     'developer', 'robots', sdk, sdk + '/lib:/usr/local/lib',
                                     project + '/venv', '/data/语音 models')
            self.assertNotIn('/home/unitree', result, template.name)
            self.assertIn('WorkingDirectory=' + project.replace('%', '%%') + '/.', result)
            self.assertIn('Environment="HOME=/home/developer"', result)
            if '.user.' in template.name:
                self.assertNotIn('\nUser=', result)
            else:
                self.assertIn('\nUser=developer\nGroup=robots\n', result)
            if 'web-control' in template.name:
                self.assertIn('Environment=' + renderer.quote('CMAKE_PREFIX_PATH=' + sdk), result)
                self.assertIn(renderer.quote(project + '/web', escape_dollars=True), result)
            else:
                self.assertIn(renderer.quote(project + '/venv/bin/python', escape_dollars=True), result)
                self.assertIn('/data/语音 models/', result)
            # The wait scripts contain their own shell variables; rendering must preserve them.
            before = [line for line in template.read_text().splitlines() if line.startswith('ExecStartPre=')]
            after = [line for line in result.splitlines() if line.startswith('ExecStartPre=')]
            self.assertEqual(before, after)

    @unittest.skipUnless(shutil.which('systemd-analyze'), 'systemd-analyze unavailable')
    def test_units_pass_systemd_parser(self):
        with tempfile.TemporaryDirectory(prefix='unirobo systemd ') as directory:
            directory = Path(directory)
            project = directory / '客户 path $cash%percent"quote'
            (project / 'build').mkdir(parents=True)
            (project / 'build/g1_web_server').symlink_to('/bin/true')
            (project / 'tts_kokoro/bin').mkdir(parents=True)
            (project / 'tts_kokoro/bin/python').symlink_to('/bin/true')
            for name in ['sysinit', 'basic', 'shutdown', 'network', 'network-online',
                         'sound', 'default', 'multi-user']:
                (directory / (name + '.target')).write_text('[Unit]\nDescription=Fixture\nDefaultDependencies=no\n')
            units = []
            for template in (ROOT / 'deploy').glob('*.service'):
                if 'web-control' not in template.name and 'local-tts' not in template.name:
                    continue
                unit = directory / template.name
                unit.write_text(renderer.render(template.read_text(), str(project), '/home/developer',
                                                'developer', 'robots', '/sdk path', '/sdk path/lib',
                                                str(project / 'tts_kokoro'), str(project / 'tts_models')))
                units.append(str(unit))
            result = subprocess.run(['systemd-analyze', 'verify', '--man=no', *units],
                                    env=dict(os.environ, SYSTEMD_UNIT_PATH=str(directory)),
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_render_rejects_unit_injection(self):
        with self.assertRaises(ValueError):
            renderer.render('WorkingDirectory=x', '/tmp/a\nExecStart=/bad', '/tmp',
                            'developer', 'robots', '/sdk', '/sdk/lib', '/venv', '/models')

    def installer_functions(self):
        source = (ROOT / 'scripts/install.sh').read_text()
        names = ['resolve_sdk_prefix', 'install_sdk2', 'have_realsense']
        return '\n'.join(name + '() {' + source.split(name + '() {', 1)[1].split('\n}', 1)[0] + '\n}' for name in names)

    def sdk_fixture(self, prefix):
        for file in ['lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake', 'lib/libunitree_sdk2.a',
                     'lib/libddsc.so', 'lib/libddscxx.so',
                     'include/unitree/robot/r1/audio/audio_client.hpp',
                     'include/unitree/robot/r1/loco/r1_loco_client.hpp',
                     'include/unitree/robot/go2/vui/vui_client.hpp',
                     'include/unitree/robot/go2/sport/sport_client.hpp',
                     'include/unitree/robot/a2/audio/audio_client.hpp',
                     'include/unitree/robot/a2/sport/sport_client.hpp']:
            path = prefix / file
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()

    def test_sdk_reuse_install_and_no_overwrite(self):
        with tempfile.TemporaryDirectory(prefix='unirobo paths ') as directory:
            directory = Path(directory)
            fixture = directory / 'existing sdk'
            self.sdk_fixture(fixture)
            common = self.installer_functions() + '\nfail() { echo "$2" >&2; exit 1; }; ui_line() { :; };'
            for product in ['g1', 'r1', 'go2', 'a2']:
                script = common + '\nload_product_profile "$1"; sdk_prefix="$2"; install_sdk2'
                result = self.bash(script, product, str(fixture))
                self.assertEqual(result.returncode, 0, result.stderr)
                prefix = directory / ('new sdk ' + product)
                source = directory / 'sources'
                source.mkdir(exist_ok=True)
                (source / 'CMakeLists.txt').touch()
                script = common + '''
load_product_profile "$1"
sdk_prefix="$2"; sdk2_source="$3"; fixture="$4"; SDK_POLICY=install_if_missing; install_scope=user
cmake() { [[ "$1" != --install ]] || cp -R "$fixture/." "$sdk_prefix/"; }
install_sdk2
'''
                result = self.bash(script, product, str(prefix), str(source), str(fixture))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue((prefix / 'lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake').is_file())
            # An incomplete existing SDK must fail before invoking cmake.
            (fixture / 'lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake').unlink()
            script = common + '\nload_product_profile a2; sdk_prefix="$1"; SDK_POLICY=install_if_missing; cmake() { exit 99; }; install_sdk2'
            result = self.bash(script, str(fixture))
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertIn('refusing to overwrite', result.stderr)

    def test_missing_sdk_noninteractive_is_actionable(self):
        script = self.installer_functions() + '''
fail() { echo "$2" >&2; exit 1; }; ui_line() { :; }
load_product_profile a2
sdk_prefix="$1"; sdk2_source=/tmp/source; realsense_source=; realsense_prefix=/usr/local; check_only=false
resolve_sdk_prefix
'''
        result = self.bash(script, '/tmp/absent unirobo-sdk')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--sdk-prefix', result.stderr)
        self.assertIn('--install-sdk2', result.stderr)

    def test_sdk_detection_and_custom_realsense(self):
        with tempfile.TemporaryDirectory(prefix='unirobo deps ') as directory:
            directory = Path(directory)
            sdk = directory / 'sdk'
            rs = directory / 'realsense'
            self.sdk_fixture(sdk)
            config = rs / 'lib/cmake/realsense2/realsense2Config.cmake'
            config.parent.mkdir(parents=True)
            config.touch()
            script = self.installer_functions() + '''
fail() { echo "$2" >&2; exit 1; }; ui_line() { :; }
load_product_profile a2
SYSTEM_SDK_PREFIX="$1"; USER_SDK_PREFIX="$1"; sdk_prefix=; sdk2_source=/tmp/source
realsense_source=; realsense_prefix="$2"; check_only=true
resolve_sdk_prefix
[[ "$sdk_prefix" == "$1" ]]
have_realsense
'''
            result = self.bash(script, str(sdk), str(rs))
            self.assertEqual(result.returncode, 0, result.stderr)
            script += '\nexport UNIROBOGUI_REALSENSE_PREFIX=/tmp/missing-rs; realsense_prefix=$UNIROBOGUI_REALSENSE_PREFIX; pkg-config() { return 0; }; ! have_realsense'
            result = self.bash(script, str(sdk), str(rs))
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
