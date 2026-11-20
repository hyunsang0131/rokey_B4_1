"""Install an idempotent desktop shortcut for the HMI and order page."""

import os
import shlex
import subprocess
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)


LAUNCHER_NAME = 'cushion-hmi-launcher.sh'
DESKTOP_NAME = 'Cushion HMI.desktop'


def _desktop_directory():
    """Return the user's configured desktop directory."""
    try:
        result = subprocess.run(
            ['xdg-user-dir', 'DESKTOP'],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        configured = os.environ.get('XDG_DESKTOP_DIR')
        return Path(configured).expanduser() if configured else (
            Path.home() / 'Desktop')

    directory = result.stdout.strip()
    return Path(directory).expanduser() if directory else Path.home() / 'Desktop'


def _shell_quote(path):
    """Quote a filesystem path for the generated Bash launcher."""
    return shlex.quote(str(path))


def _desktop_quote(value):
    """Quote one value for the Desktop Entry Exec field."""
    escaped = (
        value.replace('\\', '\\\\')
        .replace('"', '\\"')
        .replace('`', '\\`')
        .replace('$', '\\$')
    )
    return f'"{escaped}"'


def _write_if_changed(path, content, mode):
    """Write a generated launcher file only when its content differs."""
    if path.exists() and path.read_text(encoding='utf-8') == content:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding='utf-8')
    path.chmod(mode)
    return True


def ensure_desktop_launcher():
    """Create the desktop shortcut once, without replacing user files."""
    package_prefix = Path(get_package_prefix('cushion_hmi'))
    overlay_setup = package_prefix.parent / 'setup.bash'
    share_directory = Path(get_package_share_directory('cushion_hmi'))
    order_page = share_directory / 'web' / 'bed_ordering.html'
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy')
    ros_setup = Path('/opt/ros') / ros_distro / 'setup.bash'

    missing = [
        path for path in (overlay_setup, order_page, ros_setup)
        if not path.is_file()
    ]
    if missing:
        raise FileNotFoundError(
            '실행기에 필요한 파일을 찾을 수 없습니다: '
            + ', '.join(str(path) for path in missing))

    local_bin = Path.home() / '.local' / 'bin'
    launcher_path = local_bin / LAUNCHER_NAME
    launcher_content = '\n'.join((
        '#!/usr/bin/env bash',
        'set -e',
        f'source {_shell_quote(ros_setup)}',
        f'source {_shell_quote(overlay_setup)}',
        f'xdg-open {_shell_quote(order_page)} >/dev/null 2>&1 &',
        'exec ros2 run cushion_hmi hmi',
        '',
    ))
    _write_if_changed(launcher_path, launcher_content, 0o755)

    desktop_directory = _desktop_directory()
    desktop_path = desktop_directory / DESKTOP_NAME
    desktop_content = '\n'.join((
        '[Desktop Entry]',
        'Version=1.0',
        'Type=Application',
        'Name=쿠션 HMI 및 주문 페이지',
        'Comment=HMI와 HTML 주문 페이지를 함께 실행합니다',
        f'Exec={_desktop_quote(str(launcher_path))}',
        'Terminal=false',
        'Categories=Utility;',
        '',
    ))

    if desktop_path.exists():
        if desktop_path.read_text(encoding='utf-8') == desktop_content:
            return desktop_path, False
        raise FileExistsError(
            f'기존 파일을 덮어쓰지 않았습니다: {desktop_path}')

    for existing in desktop_directory.glob('*.desktop'):
        if existing.read_text(encoding='utf-8') == desktop_content:
            return existing, False

    _write_if_changed(desktop_path, desktop_content, 0o755)
    try:
        subprocess.run(
            ['gio', 'set', str(desktop_path), 'metadata::trusted', 'true'],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        pass
    return desktop_path, True
