#!/usr/bin/env python3
"""Pick (and, on Humble, patch) the Nav2 params file for the running distro.

config/nav2_params.yaml is written for Jazzy's pluginlib registration
convention: a plugin's XML <class> entry there has no explicit ``name=``
attribute, so pluginlib's ClassLoader falls back to the fully-qualified C++
type itself as the lookup key (e.g. "nav2_navfn_planner::NavfnPlanner").
Humble's apt-installed ros-humble-navigation2 (1.1.20) still registers two
specific plugin sets under the old, explicit slash-style name instead:
nav2_navfn_planner's global planner, and all five nav2_behaviors recovery
behaviors -- confirmed by reading the actual installed plugin XML under
/opt/ros/humble/share/nav2_navfn_planner/ and
/opt/ros/humble/share/nav2_behaviors/ on a Humble host. Everything else this
repo's nav2_params.yaml references (costmap layers, the path-follow
controller, the progress/goal checkers, the smoother) already registers
unnamed/::-style even on Humble 1.1.20, so only those two packages actually
need patching -- this is not a blanket "::" -> "/" replace, which would
break the plugins that already work.

On Jazzy (or any non-Humble distro) this is a no-op: the original file path
is returned untouched, since nav2_params.yaml already matches that registry
as-is.

On Humble, a patched copy (only the known-affected plugin type strings
rewritten to their slash-style equivalents) is written under
~/.cache/big_bertha_sim_bringup/ and that path is returned instead. This
script runs from the *installed* share directory under a normal `ros2
launch` (colcon), not the git checkout, so the generated file can't live
next to the source the way scripts/isaac/prepare_urdf.sh's scratch tree
does -- a user cache dir is the only location guaranteed writable and
present regardless of which workspace/prefix this package was built into.
"""
import os
import sys

# new (Jazzy, as written in nav2_params.yaml) -> old (Humble 1.1.20's actual
# registered lookup name for the same, unchanged binary/class).
HUMBLE_PLUGIN_ALIASES = {
    'nav2_navfn_planner::NavfnPlanner': 'nav2_navfn_planner/NavfnPlanner',
    'nav2_behaviors::Spin': 'nav2_behaviors/Spin',
    'nav2_behaviors::BackUp': 'nav2_behaviors/BackUp',
    'nav2_behaviors::DriveOnHeading': 'nav2_behaviors/DriveOnHeading',
    'nav2_behaviors::Wait': 'nav2_behaviors/Wait',
    'nav2_behaviors::AssistedTeleop': 'nav2_behaviors/AssistedTeleop',
}


def select_params_file(source_path: str) -> str:
    distro = os.environ.get('ROS_DISTRO', '')
    if distro != 'humble':
        return source_path

    with open(source_path) as f:
        text = f.read()

    patched = text
    for new, old in HUMBLE_PLUGIN_ALIASES.items():
        patched = patched.replace(f'"{new}"', f'"{old}"')

    if patched == text:
        # Nothing to patch (e.g. nav2_params.yaml already changed upstream
        # of this script) -- don't silently hand back a stale generated
        # file from a previous run.
        return source_path

    out_dir = os.path.expanduser('~/.cache/big_bertha_sim_bringup')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'nav2_params_humble.yaml')
    with open(out_path, 'w') as f:
        f.write(patched)
    return out_path


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(
            'usage: select_nav2_params.py <source_nav2_params.yaml>',
            file=sys.stderr,
        )
        sys.exit(1)
    print(select_params_file(sys.argv[1]))
