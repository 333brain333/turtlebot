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
import urllib.request
from pathlib import Path
from typing import Optional, List

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

# --------------------------------------------------------------------------- #
# Глобальные объекты
# --------------------------------------------------------------------------- #
serial_conn: Optional[serial.Serial] = None
serial_lock  = threading.Lock()
ws_clients:  List[WebSocket] = []
main_loop:   Optional[asyncio.AbstractEventLoop] = None
stop_event = threading.Event()
reader_thread: Optional[threading.Thread] = None

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
                if line and main_loop:
                    asyncio.run_coroutine_threadsafe(broadcast(line), main_loop)
            else:
                time.sleep(1)
                if not stop_event.is_set():
                    open_serial()
        except Exception as exc:
            print(f"[serial] ошибка чтения: {exc}")
            time.sleep(1)


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


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    ws_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            with serial_lock:
                if serial_conn and serial_conn.is_open:
                    try:
                        serial_conn.write((data + "\n").encode("utf-8"))
                    except Exception as exc:
                        print(f"[serial] ошибка записи: {exc}")
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
