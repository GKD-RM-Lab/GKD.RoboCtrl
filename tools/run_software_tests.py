#!/usr/bin/env python3
"""Compile/run hardware-free tests without xmake or SocketCAN implementation.

Dependencies must already exist. This runner never downloads packages, opens CAN
or serial devices, or builds/starts the robot executable.
"""
import argparse
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCES = [
    "tests/unit_tests.cpp", "tests/configuration_migration_tests.cpp",
    "tests/motor_protocol_tests.cpp", "tests/network_protocol_tests.cpp",
    "tests/referee_protocol_tests.cpp", "tests/shoot_integration_tests.cpp",
    "tests/motion_migration_tests.cpp", "tests/power_control_tests.cpp",
    "tests/ballistics_tests.cpp", "src/device/base.cpp",
    "src/device/gimbal/base.cpp", "src/device/gimbal/gkd_sentry_gimbal.cpp",
    "src/device/referee/referee.cpp", "src/ctrl/shoot.cpp",
    "src/ctrl/power_manager.cpp", "src/utils/ballistics.cpp",
    "src/io/serial.cpp", "src/core/async.cpp", "src/core/logger.cpp",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="clang++")
    for dependency in ("asio-include", "reflect-include", "yaml-include",
                       "reflect-library", "yaml-library"):
        parser.add_argument("--" + dependency, type=pathlib.Path, required=True)
    args = parser.parse_args()
    dependencies = [args.asio_include, args.reflect_include, args.yaml_include,
                    args.reflect_library, args.yaml_library]
    for path in dependencies:
        if not path.exists():
            parser.error(f"dependency does not exist: {path}")
    with tempfile.TemporaryDirectory(prefix="roboctrl-software-tests-") as directory:
        binary = pathlib.Path(directory) / "unit-tests"
        command = [args.compiler, "-std=c++23", "-O0", "-UNDEBUG", "-pthread",
                   "-I" + str(ROOT / "include")]
        command += ["-I" + str(path.resolve()) for path in dependencies[:3]]
        command += [str(ROOT / source) for source in SOURCES]
        command += [str(path.resolve()) for path in dependencies[3:]]
        command += ["-o", str(binary)]
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run([str(binary)], cwd=ROOT, check=True)
        print("C++ hardware-free aggregate: PASS", flush=True)
        subprocess.run(["python3", "-B", "-m", "unittest", "discover", "-s", "tests",
                        "-p", "test_tuning_tools.py"], cwd=ROOT, check=True)
        print("Python tuning tools: PASS", flush=True)


if __name__ == "__main__":
    main()
