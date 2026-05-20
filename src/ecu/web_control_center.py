"""
Robot Controller – FastAPI backend
====================================
Установка зависимостей:
    pip install fastapi uvicorn[standard]

Запуск:
    python3 web_control_center.py

Открыть в браузере: http://<jetson-ip>:8000
"""

import asyncio
import threading
import time
import argparse
import getpass
import math
import os
import pwd
import re
import shlex
import shutil
import subprocess
import urllib.request
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse

# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #
GO2RTC_HOST = "localhost"  # go2rtc всегда локальный
GO2RTC_PORT = 1984
CAMERA_SRC = "webcam"  # имя источника в go2rtc (--camera-src)
BATTERY_EMPTY_V = 13.2  # 4S Li-Ion/LiPo, приблизительная оценка
BATTERY_FULL_V = 16.8
WHEEL_BASE_M = 0.21294
ROS_CONTAINER = "ros2_humble_tx2"

# --------------------------------------------------------------------------- #
# Глобальные объекты
# --------------------------------------------------------------------------- #
ws_clients: List[WebSocket] = []
main_loop: Optional[asyncio.AbstractEventLoop] = None
stop_event = threading.Event()
ros_reader_thread: Optional[threading.Thread] = None
ros_reader_process: Optional[subprocess.Popen] = None
ros_command_process: Optional[subprocess.Popen] = None
ros_command_lock = threading.Lock()
telemetry_lock = threading.Lock()
latest_power: Dict[str, Optional[float]] = {
    "voltage_v": None,
    "current_a": None,
    "updated_at": None,
}
last_cpu_sample: Optional[Tuple[int, int]] = None

PWR_RE = re.compile(r"PWR V=([\d.\-]+|nan) I=([\d.\-]+|nan) A")

ROS_COMMAND_WRITER_SCRIPT = f"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

WHEEL_BASE_M = {WHEEL_BASE_M!r}

rclpy.init()
node = Node("ecu_web_command_writer")
string_publishers = {{
    "/SET_LED": node.create_publisher(String, "/SET_LED", 10),
    "/SET_COEFF": node.create_publisher(String, "/SET_COEFF", 10),
    "/SHUTDOWN": node.create_publisher(String, "/SHUTDOWN", 10),
}}
cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)

deadline = time.monotonic() + 0.5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

for line in sys.stdin:
    command = line.strip()
    if not command:
        continue
    parts = command.split()
    try:
        name = parts[0]
        if name == "SET_WHEELS_SPEED" and len(parts) >= 3:
            left_mmps = float(parts[1])
            right_mmps = float(parts[2])
            msg = Twist()
            msg.linear.x = ((left_mmps + right_mmps) / 2.0) / 1000.0
            msg.angular.z = (right_mmps - left_mmps) / (WHEEL_BASE_M * 1000.0)
            cmd_vel_pub.publish(msg)
        elif name in ("SET_LED", "SET_COEFF", "SHUTDOWN"):
            msg = String()
            msg.data = "SHUTDOWN" if name == "SHUTDOWN" else command
            string_publishers["/" + name].publish(msg)
        else:
            print("unsupported command: " + command, file=sys.stderr, flush=True)
            continue
        rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as exc:
        print("failed to publish " + command + ": " + str(exc), file=sys.stderr, flush=True)
"""

app = FastAPI(title="Robot Controller")


# --------------------------------------------------------------------------- #
# ROS
# --------------------------------------------------------------------------- #
def ros_shell_command(command: str, interactive: bool = False) -> List[str]:
    docker_command = [
        "docker",
        "exec",
    ]
    if interactive:
        docker_command.append("-i")
    return docker_command + [
        ROS_CONTAINER,
        "bash",
        "-lc",
        "source /opt/ros/humble/setup.bash "
        "&& if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi "
        f"&& {command}",
    ]


def run_ros_command(command: str, timeout: float = 3.0) -> bool:
    try:
        result = subprocess.run(
            ros_shell_command(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            timeout=timeout,
        )
    except Exception as exc:
        print(f"[ros] command failed: {exc}")
        return False
    if result.returncode != 0:
        print(f"[ros] command failed: {result.stderr.strip()}")
        return False
    return True


def quote_ros_string(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def ros_python_command(script: str) -> str:
    return "python3 -u -c " + shlex.quote(script)


def ros_raw_reader_thread() -> None:
    global ros_reader_process
    command = (
        "python3 -u -c '"
        "import rclpy\n"
        "from rclpy.node import Node\n"
        "from std_msgs.msg import String\n"
        "rclpy.init()\n"
        'node = Node("ecu_web_raw_reader")\n'
        'node.create_subscription(String, "/ESP32_RAW", lambda msg: print(msg.data, flush=True), 50)\n'
        "rclpy.spin(node)\n"
        "'"
    )
    while not stop_event.is_set():
        try:
            ros_reader_process = subprocess.Popen(
                ros_shell_command(command),
                stdout=subprocess.PIPE,
                stderr=None,
                universal_newlines=True,
                bufsize=1,
            )
            assert ros_reader_process.stdout is not None
            for raw_line in ros_reader_process.stdout:
                if stop_event.is_set():
                    break
                line = raw_line.strip()
                if not line or line == "---":
                    continue
                update_power_telemetry(line)
                if main_loop:
                    asyncio.run_coroutine_threadsafe(broadcast(line), main_loop)
            if ros_reader_process.poll() is None:
                ros_reader_process.terminate()
        except Exception as exc:
            print(f"[ros] raw reader error: {exc}")
        finally:
            ros_reader_process = None
        if not stop_event.is_set():
            time.sleep(1)


def start_ros_command_writer() -> None:
    global ros_command_process
    if ros_command_process and ros_command_process.poll() is None:
        return
    ros_command_process = subprocess.Popen(
        ros_shell_command(ros_python_command(ROS_COMMAND_WRITER_SCRIPT), interactive=True),
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=None,
        universal_newlines=True,
        bufsize=1,
    )


def send_ros_command(command: str) -> bool:
    global ros_command_process
    command = command.strip()
    if not command:
        return False
    with ros_command_lock:
        try:
            start_ros_command_writer()
            if not ros_command_process or not ros_command_process.stdin:
                return False
            if ros_command_process.poll() is not None:
                ros_command_process = None
                start_ros_command_writer()
                if not ros_command_process or not ros_command_process.stdin:
                    return False
            ros_command_process.stdin.write(command + "\n")
            ros_command_process.stdin.flush()
            return True
        except Exception as exc:
            print(f"[ros] command writer failed: {exc}")
            if ros_command_process and ros_command_process.poll() is None:
                ros_command_process.terminate()
            ros_command_process = None
            return False


def start_ros_bridge() -> None:
    global ros_reader_thread
    stop_event.clear()
    start_ros_command_writer()
    ros_reader_thread = threading.Thread(target=ros_raw_reader_thread, daemon=True)
    ros_reader_thread.start()
    print("[ros] ECU web bridge started through docker exec")


def stop_ros_bridge() -> None:
    global ros_reader_process, ros_reader_thread, ros_command_process
    stop_event.set()
    if ros_reader_process and ros_reader_process.poll() is None:
        ros_reader_process.terminate()
        try:
            ros_reader_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            ros_reader_process.kill()
    ros_reader_process = None
    if ros_reader_thread and ros_reader_thread.is_alive():
        ros_reader_thread.join(timeout=2)
    ros_reader_thread = None
    with ros_command_lock:
        if ros_command_process and ros_command_process.poll() is None:
            ros_command_process.terminate()
            try:
                ros_command_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                ros_command_process.kill()
        ros_command_process = None


def parse_optional_float(value: str) -> Optional[float]:
    try:
        parsed = float(value)
    except ValueError:
        return None
    if math.isnan(parsed):
        return None
    return parsed


def update_power_telemetry(line: str) -> None:
    match = PWR_RE.search(line)
    if not match:
        return
    with telemetry_lock:
        latest_power["voltage_v"] = parse_optional_float(match.group(1))
        latest_power["current_a"] = parse_optional_float(match.group(2))
        latest_power["updated_at"] = time.time()


def read_cpu_percent() -> Optional[float]:
    global last_cpu_sample
    try:
        parts = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
        values = [int(part) for part in parts]
    except Exception:
        return None

    idle = values[3] + (values[4] if len(values) > 4 else 0)
    total = sum(values)
    sample = (idle, total)
    if last_cpu_sample is None:
        last_cpu_sample = sample
        return None

    prev_idle, prev_total = last_cpu_sample
    last_cpu_sample = sample
    total_delta = total - prev_total
    idle_delta = idle - prev_idle
    if total_delta <= 0:
        return None
    return round(max(0.0, min(100.0, (1 - idle_delta / total_delta) * 100)), 1)


def read_memory_percent() -> Optional[float]:
    try:
        data = {}
        for line in Path("/proc/meminfo").read_text().splitlines():
            key, raw_value = line.split(":", 1)
            data[key] = int(raw_value.strip().split()[0])
        total = data.get("MemTotal")
        available = data.get("MemAvailable")
        if not total or available is None:
            return None
        return round((1 - available / total) * 100, 1)
    except Exception:
        return None


def read_cpu_temperature_c() -> Optional[float]:
    zones = sorted(Path("/sys/class/thermal").glob("thermal_zone*"))
    fallback: Optional[float] = None
    for zone in zones:
        try:
            zone_type = (zone / "type").read_text().strip().lower()
            raw_temp = int((zone / "temp").read_text().strip()) / 1000
        except Exception:
            continue
        if fallback is None:
            fallback = raw_temp
        if any(name in zone_type for name in ("cpu", "core", "soc")):
            return round(raw_temp, 1)
    return round(fallback, 1) if fallback is not None else None


def read_system_uptime_s() -> Optional[int]:
    try:
        uptime_raw = Path("/proc/uptime").read_text().split()[0]
        return int(float(uptime_raw))
    except Exception:
        return None


def voltage_to_battery_percent(voltage: Optional[float]) -> Optional[int]:
    if voltage is None:
        return None
    pct = (voltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100
    return int(round(max(0, min(100, pct))))


def get_status_payload() -> Dict[str, Any]:
    with telemetry_lock:
        voltage = latest_power["voltage_v"]
        current = latest_power["current_a"]
        power_updated_at = latest_power["updated_at"]

    return {
        "cpu_percent": read_cpu_percent(),
        "memory_percent": read_memory_percent(),
        "cpu_temperature_c": read_cpu_temperature_c(),
        "battery_percent": voltage_to_battery_percent(voltage),
        "battery_voltage_v": voltage,
        "robot_current_a": abs(current) if current is not None else None,
        "power_updated_at": power_updated_at,
        "uptime_s": read_system_uptime_s(),
    }


def publish_string(topic: str, command: str) -> bool:
    return send_ros_command(command)


def publish_cmd_vel(linear_mps: float, angular_radps: float) -> bool:
    left_mmps = (linear_mps - angular_radps * WHEEL_BASE_M / 2.0) * 1000.0
    right_mmps = (linear_mps + angular_radps * WHEEL_BASE_M / 2.0) * 1000.0
    return publish_wheel_speeds(left_mmps, right_mmps)


def publish_wheel_speeds(left_mmps: float, right_mmps: float) -> bool:
    return send_ros_command(f"SET_WHEELS_SPEED {left_mmps:.6f} {right_mmps:.6f}")


def route_mcu_command(command: str) -> bool:
    parts = command.strip().split()
    if not parts:
        return False

    name = parts[0]
    try:
        if name == "SET_WHEELS_SPEED" and len(parts) >= 3:
            float(parts[1])
            float(parts[2])
            return send_ros_command(command)
        if name == "SET_LED":
            return publish_string("/SET_LED", command)
        if name == "SET_COEFF":
            return publish_string("/SET_COEFF", command)
        if name == "SHUTDOWN":
            return publish_string("/SHUTDOWN", "SHUTDOWN")
    except ValueError as exc:
        print(f"[ros] invalid command '{command}': {exc}")
        return False

    print(f"[ros] unsupported web command: {command}")
    return False


def request_system_power_action(action: str) -> Dict[str, Any]:
    helper_path = f"/usr/local/sbin/tb_ecu_{action}"
    sudo_path = shutil.which("sudo")
    identity = {
        "uid": os.getuid(),
        "euid": os.geteuid(),
        "user": getpass.getuser(),
        "pw_name": pwd.getpwuid(os.getuid()).pw_name,
        "sudo_path": sudo_path,
        "helper_exists": Path(helper_path).exists(),
    }
    if Path(helper_path).exists():
        commands = [
            ["sudo", "-n", "-u", "root", helper_path],
            ["sudo", "-n", helper_path],
        ]
    else:
        if action == "reboot":
            commands = [
                ["sudo", "-n", "systemctl", "--no-block", "reboot"],
                ["sudo", "-n", "loginctl", "reboot", "--no-wall"],
                ["sudo", "-n", "shutdown", "-r", "now"],
                ["sudo", "-n", "reboot"],
                ["systemctl", "--no-block", "reboot"],
                ["loginctl", "reboot", "--no-wall"],
                ["shutdown", "-r", "now"],
                ["reboot"],
            ]
        else:
            commands = [
                ["sudo", "-n", "systemctl", "--no-block", "poweroff"],
                ["sudo", "-n", "loginctl", "poweroff", "--no-wall"],
                ["sudo", "-n", "shutdown", "-h", "now"],
                ["sudo", "-n", "poweroff"],
                ["systemctl", "--no-block", "poweroff"],
                ["loginctl", "poweroff", "--no-wall"],
                ["shutdown", "-h", "now"],
                ["poweroff"],
            ]

    attempts = []
    for command in commands:
        executable = (
            command[0] if command[0].startswith("/") else shutil.which(command[0])
        )
        if not executable:
            attempts.append(
                {
                    "command": command[0],
                    "ok": False,
                    "error": "not found",
                }
            )
            continue

        full_command = [executable, *command[1:]]
        try:
            result = subprocess.run(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=3,
            )
        except Exception as exc:
            attempts.append(
                {
                    "command": " ".join(full_command),
                    "ok": False,
                    "error": str(exc),
                }
            )
            continue

        attempt = {
            "command": " ".join(full_command),
            "ok": result.returncode == 0,
            "returncode": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        }
        attempts.append(attempt)
        if result.returncode == 0:
            return {"ok": True, "identity": identity, "attempts": attempts}

    return {"ok": False, "identity": identity, "attempts": attempts}


def request_system_poweroff() -> Dict[str, Any]:
    return request_system_power_action("poweroff")


def request_system_reboot() -> Dict[str, Any]:
    return request_system_power_action("reboot")


def check_power_permission(action: str) -> Dict[str, Any]:
    helper_path = f"/usr/local/sbin/tb_ecu_{action}"
    command = ["sudo", "-n", "-l", helper_path]
    executable = shutil.which(command[0])
    identity = {
        "uid": os.getuid(),
        "euid": os.geteuid(),
        "user": getpass.getuser(),
        "pw_name": pwd.getpwuid(os.getuid()).pw_name,
        "sudo_path": executable,
        "helper_exists": Path(helper_path).exists(),
    }
    if not executable:
        return {"ok": False, "identity": identity, "error": "sudo not found"}

    result = subprocess.run(
        [executable, *command[1:]],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        timeout=3,
    )
    return {
        "ok": result.returncode == 0,
        "identity": identity,
        "command": " ".join([executable, *command[1:]]),
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }


def check_poweroff_permission() -> Dict[str, Any]:
    return check_power_permission("poweroff")


def check_reboot_permission() -> Dict[str, Any]:
    return check_power_permission("reboot")


# --------------------------------------------------------------------------- #
# WebSocket broadcast
# --------------------------------------------------------------------------- #
async def broadcast(message: str) -> None:
    dead = []
    for ws in list(ws_clients):
        try:
            await ws.send_text(message)
        except Exception:
            dead.append(ws)
    for ws in dead:
        if ws in ws_clients:
            ws_clients.remove(ws)


# --------------------------------------------------------------------------- #
# Lifecycle
# --------------------------------------------------------------------------- #
@app.on_event("startup")
async def startup_event() -> None:
    global main_loop
    main_loop = asyncio.get_event_loop()
    read_cpu_percent()
    start_ros_bridge()


@app.on_event("shutdown")
async def shutdown_event() -> None:
    # Явно закрываем WS, чтобы uvicorn не зависал на ожидании клиентов.
    clients = list(ws_clients)
    ws_clients.clear()
    if clients:
        await asyncio.gather(
            *(ws.close(code=1001, reason="Server shutdown") for ws in clients),
            return_exceptions=True,
        )

    stop_ros_bridge()


# --------------------------------------------------------------------------- #
# Роуты
# --------------------------------------------------------------------------- #
@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return (Path(__file__).parent / "index.html").read_text(encoding="utf-8")


@app.get("/camera")
def camera_proxy():
    """
    Проксирует MJPEG-поток с локального go2rtc → браузеру.
    Браузер делает GET /camera на тот же хост, никаких внешних IP не нужно.
    """
    url = f"http://{GO2RTC_HOST}:{GO2RTC_PORT}/api/stream.mjpeg?src={CAMERA_SRC}"
    try:
        resp = urllib.request.urlopen(url, timeout=5)
    except Exception as exc:
        return HTMLResponse(f"Камера недоступна: {exc}", status_code=502)

    content_type = resp.headers.get(
        "Content-Type", "multipart/x-mixed-replace; boundary=frame"
    )

    def stream_chunks():
        try:
            while True:
                chunk = resp.read(8192)
                if not chunk:
                    break
                yield chunk
        finally:
            resp.close()

    return StreamingResponse(stream_chunks(), media_type=content_type)


@app.get("/api/status")
def api_status() -> Dict[str, Any]:
    return get_status_payload()


@app.post("/api/shutdown")
def api_shutdown() -> Dict[str, Any]:
    mcu_command_sent = publish_string("/SHUTDOWN", "SHUTDOWN")
    poweroff_result = request_system_poweroff()
    return {
        "ok": poweroff_result["ok"],
        "mcu_command_sent": mcu_command_sent,
        "poweroff": poweroff_result,
    }


@app.post("/api/reboot")
def api_reboot() -> Dict[str, Any]:
    reboot_result = request_system_reboot()
    return {
        "ok": reboot_result["ok"],
        "mcu_command_sent": False,
        "reboot": reboot_result,
    }


@app.get("/api/shutdown/check")
def api_shutdown_check() -> Dict[str, Any]:
    return check_poweroff_permission()


@app.get("/api/reboot/check")
def api_reboot_check() -> Dict[str, Any]:
    return check_reboot_permission()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    ws_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            route_mcu_command(data)
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in ws_clients:
            ws_clients.remove(websocket)


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Controller Web Server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument(
        "--go2rtc-port", type=int, default=1984, help="go2rtc HTTP port (default 1984)"
    )
    parser.add_argument(
        "--camera-src",
        default="webcam",
        help="Имя источника в go2rtc (default: webcam)",
    )
    args = parser.parse_args()

    GO2RTC_PORT = args.go2rtc_port
    CAMERA_SRC = args.camera_src

    print(f"[server]  http://{args.host}:{args.http_port}")
    print(
        f"[camera]  проксируем http://localhost:{GO2RTC_PORT}/api/stream.mjpeg?src={CAMERA_SRC}"
    )
    uvicorn.run(
        app,
        host=args.host,
        port=args.http_port,
        log_level="info",
        timeout_graceful_shutdown=5,
    )
