#!/usr/bin/env python3
"""Render reviewed systemd templates with the selected installation paths."""
import argparse
import os
import pathlib
import shlex


def quote(value, escape_dollars=False):
    if any(ord(c) < 32 or ord(c) == 127 for c in value):
        raise ValueError('Control characters are not allowed in unit values')
    value = value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    if escape_dollars:
        value = value.replace('$', '$$')
    return '"' + value + '"'


def render(template, project, home, user, group, sdk, libraries, venv, models):
    replacements = [
        ('/home/unitree/UniRoboGui/tts_kokoro', venv),
        ('/home/unitree/UniRoboGui/tts_models', models),
        ('/home/unitree/UniRoboGui', project),
    ]
    lines = []
    for line in template.splitlines():
        key, sep, value = line.partition('=')
        if key == 'User':
            line = 'User=' + user
        elif key == 'Group':
            line = 'Group=' + group
        elif key == 'WorkingDirectory':
            quote(project)  # Validate without adding quotes: WorkingDirectory is a literal path.
            line = key + '=' + project.replace('%', '%%') + '/.'
        elif key == 'Environment':
            name = value.split('=', 1)[0]
            resolved = {'HOME': home, 'CMAKE_PREFIX_PATH': sdk,
                        'LD_LIBRARY_PATH': libraries}.get(name)
            if resolved is not None:
                line = 'Environment=' + quote(name + '=' + resolved)
        elif key == 'ExecStart':
            args = shlex.split(value)
            for i, arg in enumerate(args):
                for old, new in replacements:
                    if arg == old or arg.startswith(old + '/'):
                        args[i] = new + arg[len(old):]
                        break
            # systemd rejects quotes in executable names; env execs the path as an argument.
            args = ['/usr/bin/env', '--'] + args
            line = key + '=' + ' '.join(quote(arg, escape_dollars=i > 0) for i, arg in enumerate(args))
        lines.append(line)
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('template', type=pathlib.Path)
    parser.add_argument('output', type=pathlib.Path)
    parser.add_argument('--project', required=True)
    parser.add_argument('--sdk', required=True)
    parser.add_argument('--libraries', required=True)
    args = parser.parse_args()
    args.output.write_text(render(
        args.template.read_text(), args.project, os.environ['HOME'],
        os.environ['USER'], os.environ['UNIROBOGUI_GROUP'], args.sdk, args.libraries,
        os.environ.get('KOKORO_VENV', args.project + '/tts_kokoro'),
        os.environ.get('KOKORO_MODEL_ROOT', args.project + '/tts_models')))


if __name__ == '__main__':
    main()
