# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------


import os
import sys
import signal
import socket
from subprocess import run, Popen


backend_dir = "./backend"
python_path = "./venv/bin/python3"
pip_path = "./venv/bin/pip"

frontend_dir = "./frontend"

processes: list[Popen] = []


def setup_backend():
    print("Setting up backend...")

    # if there is not a venv, setup the venv
    if not os.path.isdir(f"{backend_dir}/venv"):
        run(["python3", "-m", "venv", "venv"], cwd=backend_dir)

        run([pip_path, "install", "-r", "requirements.txt"], cwd=backend_dir)

    print("Backend setup!")


def setup_frontend():
    print("Setting up frontend...")

    # if there is not a node_modules, setup the node_modules
    if not os.path.isdir(f"{frontend_dir}/node_modules"):
        run(["npm", "install"], cwd=frontend_dir)

    print("Frontend setup!")


def run_edge_impulse_service():
    print("Running edge impulse service!")

    processes.append(Popen(["edge-impulse-linux-runner",
                            "--enable-camera",
                            "--force-target", "runner-linux-aarch64-qnn",
                            "--force-engine", "tflite",
                            "--force-variant", "int8",
                            "--run-http-server", "8760",
                            "--dont-print-predictions"], cwd=backend_dir))


def run_backend():
    print("Running backend!")

    processes.append(Popen([python_path, "main.py"], cwd=backend_dir))


def run_frontend():
    print("Running frontend!")

    processes.append(
        Popen(["npm", "run", "dev", "--", "--host"], cwd=frontend_dir))


def cleanup():
    print("Cleaning up subprocesses...")

    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            print(f"Terminated process group {p.pid}")
        except Exception as e:
            print(f"Error terminating process {p.pid}: {e}")

    print("Subprocesses cleaned!")


if __name__ == "__main__":
    try:
        setup_backend()
        setup_frontend()

        run_edge_impulse_service()
        run_backend()
        run_frontend()

        print(
            f"Connect to the frontend at: http://{socket.gethostbyname(socket.gethostname())}:3000")

        # keep this script alive to allow for cleanup
        while True:
            pass

    except KeyboardInterrupt:
        cleanup()

        sys.exit(0)
