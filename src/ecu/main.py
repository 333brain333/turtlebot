"""
Robot Controller – FastAPI backend
====================================
Установка зависимостей:
    pip install fastapi uvicorn pyserial

Запуск:
    python main.py --port /dev/ttyUSB0 --baud 115200
    python main.py --port COM5          --baud 115200

Открыть в браузере: http://<jetson-ip>:8000
"""

import asyncio
import threading
import time
import argparse
from pathlib import Path

import serial
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

# --------------------------------------------------------------------------- #
# Config (переопределяются через CLI)
# --------------------------------------------------------------------------- #
SERIAL_PORT = "/dev/ttyTHS2"
BAUDRATE    = 115200

# --------------------------------------------------------------------------- #
# Глобальные объекты
# --------------------------------------------------------------------------- #
serial_conn: serial.Serial | None = None
serial_lock   = threading.Lock()
ws_clients:   list[WebSocket] = []
main_loop:    asyncio.AbstractEventLoop | None = None

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
    while True:
        try:
            if serial_conn and serial_conn.is_open:
                raw  = serial_conn.readline()
                line = raw.decode("utf-8", errors="ignore").strip()
                if line and main_loop:
                    asyncio.run_coroutine_threadsafe(broadcast(line), main_loop)
            else:
                time.sleep(1)
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
    global main_loop
    main_loop = asyncio.get_event_loop()
    open_serial()
    threading.Thread(target=serial_reader_thread, daemon=True).start()


# --------------------------------------------------------------------------- #
# Роуты
# --------------------------------------------------------------------------- #
@app.get("/", response_class=HTMLResponse)
async def index() -> str:
    return (Path(__file__).parent / "index.html").read_text(encoding="utf-8")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    ws_clients.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # Пересылаем команду в serial
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
    parser.add_argument("--port",      default="/dev/ttyTHS2",
                        help="Serial port (e.g. /dev/ttyUSB0 or COM5)")
    parser.add_argument("--baud",      type=int, default=115200)
    parser.add_argument("--host",      default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8000)
    args = parser.parse_args()

    SERIAL_PORT = args.port
    BAUDRATE    = args.baud

    print(f"[server] http://{args.host}:{args.http_port}")
    uvicorn.run(app, host=args.host, port=args.http_port, log_level="info")
