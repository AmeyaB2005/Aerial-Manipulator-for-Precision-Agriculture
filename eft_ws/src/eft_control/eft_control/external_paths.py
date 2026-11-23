"""Where to find the external tools the launch files start (no absolute paths).

PX4-Autopilot and the Micro XRCE-DDS Agent are installed separately from this repository.
Each one is looked up in this order:

    1. launch argument        px4_dir:=...            xrce_agent:=...
    2. environment variable   PX4_DIR                 MICRO_XRCE_AGENT
    3. on the PATH            -                       MicroXRCEAgent (e.g. after `make install`)
    4. default location       ~/PX4-Autopilot         ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent

so the repository works for any user, wherever it is cloned.
"""
import os
import shutil

from launch.actions import DeclareLaunchArgument


def px4_dir() -> str:
    """PX4-Autopilot source folder (where `make px4_sitl ...` is run)."""
    return os.path.expanduser(os.environ.get('PX4_DIR', '~/PX4-Autopilot'))


def xrce_agent() -> str:
    """Micro XRCE-DDS Agent executable."""
    from_env = os.environ.get('MICRO_XRCE_AGENT')
    if from_env:
        return os.path.expanduser(from_env)
    on_path = shutil.which('MicroXRCEAgent')
    if on_path:
        return on_path
    return os.path.expanduser('~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent')


def external_path_args() -> list:
    """Launch arguments `px4_dir` and `xrce_agent`, with the defaults above."""
    return [
        DeclareLaunchArgument(
            'px4_dir', default_value=px4_dir(),
            description='PX4-Autopilot folder (or set the PX4_DIR environment variable)'),
        DeclareLaunchArgument(
            'xrce_agent', default_value=xrce_agent(),
            description='MicroXRCEAgent executable (or set MICRO_XRCE_AGENT, or put it on the PATH)'),
    ]
