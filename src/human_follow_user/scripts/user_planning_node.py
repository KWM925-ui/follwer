#!/usr/bin/env python3
import os
import runpy
import subprocess
import sys


def resolve_template_path():
    try:
        package_path = subprocess.check_output(
            ["rospack", "find", "human_follow_control"],
            stderr=subprocess.STDOUT,
            text=True,
        ).strip()
    except Exception as exc:  # pylint: disable=broad-except
        raise RuntimeError("failed to resolve human_follow_control via rospack") from exc

    template_path = os.path.join(package_path, "scripts", "planning_algorithm_template_node.py")
    if not os.path.isfile(template_path):
        raise RuntimeError("missing planning template: %s" % template_path)
    return template_path


def main():
    try:
        runpy.run_path(resolve_template_path(), run_name="__main__")
    except Exception as exc:  # pylint: disable=broad-except
        print("user_planning_node bootstrap failed: %s" % exc, file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
