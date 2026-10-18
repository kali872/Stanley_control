"""Where a launch file should read this package's config/*.yaml from.

The SOURCE tree when there is one, so editing config/*.yaml and relaunching is
enough - colcon copies the YAML into install/ at build time, and a launch that
read that copy would keep using whatever the last build captured. That bit the
closed-loop simulation on 2026-09-22: it read the installed copy and drove a
10/5 km/h profile while the source had said 20/10 since the morning.

Kept out of the launch files so both of them (and anything else that starts
these nodes) resolve it the same way.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def source_config(package: str, name: str) -> Path:
    """config/<name> from `package`'s source tree, else its installed copy.

    The workspace is found from the install prefix (install/<pkg>/share/<pkg>
    -> workspace) and searched a few levels deep, because packages sit at
    src/<pkg> as well as src/<group>/<pkg>. The installed copy is the fallback
    when no source checkout exists (a deployed install).
    """

    share = Path(get_package_share_directory(package))
    installed = share / "config" / name
    workspace = share.parents[3]
    for depth in range(4):
        pattern = "src/" + "*/" * depth + package
        for candidate in sorted(workspace.glob(pattern)):
            source = candidate / "config" / name
            if (candidate / "package.xml").is_file() and source.is_file():
                return source
    return installed
