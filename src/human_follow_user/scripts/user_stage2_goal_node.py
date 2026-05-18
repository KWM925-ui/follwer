#!/usr/bin/env python3
import os
import runpy
import subprocess
import sys


def resolve_template_path():
    try:
        package_path = subprocess.check_output(
            ["rospack", "find", "human_follow_bringup"],
            stderr=subprocess.STDOUT,
            text=True,
        ).strip()
    except Exception as exc:  # pylint: disable=broad-except
        raise RuntimeError("failed to resolve human_follow_bringup via rospack") from exc

    template_path = os.path.join(package_path, "scripts", "stage2_follow_goal_generator_node.py")
    if not os.path.isfile(template_path):
        raise RuntimeError("missing stage2 goal template: %s" % template_path)
    return template_path


def main():
    try:
        runpy.run_path(resolve_template_path(), run_name="__main__")
    except Exception as exc:  # pylint: disable=broad-except
        print("user_stage2_goal_node bootstrap failed: %s" % exc, file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
