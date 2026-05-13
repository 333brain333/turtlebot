"""
Robot Controller – FastAPI backend
====================================
Установка зависимостей:
    pip install fastapi uvicorn[standard] pyserial

Запуск:
    python3 main.py --port /dev/ttyTHS2 --baud 115200

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
import shutil
import subprocess
import urllib.request
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple

import serial
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse

# --------------------------------------------------------------------------- #
# Config (переопределяются через CLI)
# --------------------------------------------------------------------------- #
SERIAL_PORT   = "/dev/ttyTHS2"
BAUDRATE      = 115200
GO2RTC_HOST   = "localhost"   # go2rtc всегда локальный
GO2RTC_PORT   = 1984
CAMERA_SRC    = "webcam"      # имя источника в go2rtc (--camera-src)
BATTERY_EMPTY_V = 13.2        # 4S Li-Ion/LiPo, приблизительная оценка
BATTERY_FULL_V  = 16.8

# --------------------------------------------------------------------------- #
# Глобальные объекты
# --------------------------------------------------------------------------- #
serial_conn: Optional[serial.Serial] = None
serial_lock  = threading.Lock()
ws_clients:  List[WebSocket] = []
main_loop:   Optional[asyncio.AbstractEventLoop] = None
stop_event = threading.Event()
reader_thread: Optional[threading.Thread] = None
telemetry_lock = threading.Lock()
latest_power: Dict[str, Optional[float]] = {
    "voltage_v": None,
    "current_a": None,
    "updated_at": None,
}
last_cpu_sample: Optional[Tuple[int, int]] = None

PWR_RE = re.compile(r"PWR V=([\d.\-]+|nan) I=([\d.\-]+|nan) A")

app = FastAPI(title="Robot Controller")


# --------------------------------------------------------------------------- #
# Serial
# --------------------------------------------------------------------------- #
def open_serial() -> None:
    global serial_conn
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"[serial] открыт {SERIAL_PORT} @ {BAUDRATE}")
    except Exception as exc:
        print(f"[serial] ошибка открытия: {exc}")
        serial_conn = None


def serial_reader_thread() -> None:
    """Фоновый поток: читает строки из serial и рассылает всем WS-клиентам."""
    while not stop_event.is_set():
        try:
            if serial_conn and serial_conn.is_open:
                raw  = serial_conn.readline()
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    update_power_telemetry(line)
                    if main_loop:
                        asyncio.run_coroutine_threadsafe(broadcast(line), main_loop)
            else:
                time.sleep(1)
                if not stop_event.is_set():
                    open_serial()
        except Exception as exc:
            print(f"[serial] ошибка чтения: {exc}")
            time.sleep(1)


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


def write_serial_command(command: str) -> bool:
    with serial_lock:
        if not serial_conn or not serial_conn.is_open:
            return False
        try:
            serial_conn.write((command + "\n").encode("utf-8"))
            return True
        except Exception as exc:
            print(f"[serial] ошибка записи: {exc}")
            return False


def request_system_poweroff() -> Dict[str, Any]:
    helper_path = "/usr/local/sbin/tb_ecu_poweroff"
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
        executable = command[0] if command[0].startswith("/") else shutil.which(command[0])
        if not executable:
            attempts.append({
                "command": command[0],
                "ok": False,
                "error": "not found",
            })
            continue

        full_command = [executable, *command[1:]]
        try:
            result = subprocess.run(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3,
            )
        except Exception as exc:
            attempts.append({
                "command": " ".join(full_command),
                "ok": False,
                "error": str(exc),
            })
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


def check_poweroff_permission() -> Dict[str, Any]:
    helper_path = "/usr/local/sbin/tb_ecu_poweroff"
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
        text=True,
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
    global main_loop, reader_thread
    main_loop = asyncio.get_event_loop()
    stop_event.clear()
    read_cpu_percent()
    open_serial()
    reader_thread = threading.Thread(target=serial_reader_thread, daemon=True)
    reader_thread.start()


@app.on_event("shutdown")
async def shutdown_event() -> None:
    stop_event.set()

    # Явно закрываем WS, чтобы uvicorn не зависал на ожидании клиентов.
    clients = list(ws_clients)
    ws_clients.clear()
    if clients:
        await asyncio.gather(
            *(ws.close(code=1001, reason="Server shutdown") for ws in clients),
            return_exceptions=True,
        )

    with serial_lock:
        global serial_conn
        if serial_conn and serial_conn.is_open:
            try:
                serial_conn.close()
            except Exception as exc:
                print(f"[serial] ошибка закрытия: {exc}")
            finally:
                serial_conn = None

    if reader_thread and reader_thread.is_alive():
        reader_thread.join(timeout=2)


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

    content_type = resp.headers.get("Content-Type",
                                    "multipart/x-mixed-replace; boundary=frame")

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
    mcu_command_sent = write_serial_command("SHUTDOWN")
    poweroff_result = request_system_poweroff()
    return {
        "ok": poweroff_result["ok"],
        "mcu_command_sent": mcu_command_sent,
        "poweroff": poweroff_result,
    }


@app.get("/api/shutdown/check")
def api_shutdown_check() -> Dict[str, Any]:
    return check_poweroff_permission()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    ws_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            write_serial_command(data)
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
    parser.add_argument("--port",        default="/dev/ttyTHS2",
                        help="Serial port")
    parser.add_argument("--baud",        type=int, default=115200)
    parser.add_argument("--host",        default="0.0.0.0")
    parser.add_argument("--http-port",   type=int, default=8000)
    parser.add_argument("--go2rtc-port", type=int, default=1984,
                        help="go2rtc HTTP port (default 1984)")
    parser.add_argument("--camera-src",  default="webcam",
                        help="Имя источника в go2rtc (default: webcam)")
    args = parser.parse_args()

    SERIAL_PORT = args.port
    BAUDRATE    = args.baud
    GO2RTC_PORT = args.go2rtc_port
    CAMERA_SRC  = args.camera_src

    print(f"[server]  http://{args.host}:{args.http_port}")
    print(f"[camera]  проксируем http://localhost:{GO2RTC_PORT}/api/stream.mjpeg?src={CAMERA_SRC}")
    uvicorn.run(
        app,
        host=args.host,
        port=args.http_port,
        log_level="info",
        timeout_graceful_shutdown=5,
    )
