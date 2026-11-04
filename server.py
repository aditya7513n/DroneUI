"""Drone GCS backend: login, MAVLink bridge, camera + object detection. Run: python server.py"""
import asyncio, hmac, json, os, secrets, threading, time
import cv2
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, StreamingResponse
from pymavlink import mavutil
import uvicorn

HERE = os.path.dirname(os.path.abspath(__file__))
USER, PASS = os.getenv("GCS_USER", "admin"), os.getenv("GCS_PASS", "drone123")   # CHANGE THESE
CONN = os.getenv("MAV_CONN", "udp:127.0.0.1:14550")
BAUD = int(os.getenv("MAV_BAUD", 57600))
SERVO = int(os.getenv("DROP_SERVO", 9))
PWM_OPEN, PWM_CLOSED = int(os.getenv("DROP_OPEN", 2000)), int(os.getenv("DROP_CLOSED", 1000))
CAM = os.getenv("CAM_URL", "0")            # "0" = laptop webcam, or Pi URL http://PI_IP:8080/stream.mjpg
CAM = int(CAM) if CAM.isdigit() else CAM

app = FastAPI()
tokens = set()
S = dict(connected=False, armed=False, mode="-", lat=0, lon=0, alt=0, hdg=0, gs=0, roll=0, pitch=0,
         volt=0, batt=-1, sats=0, fix=0, dropped=False, cam=False, detect=True, fps=0, objects={}, log=[])
mav, lock, last_hb, frame_jpg = None, threading.Lock(), 0.0, None

def say(t): S["log"] = (S["log"] + [time.strftime("%H:%M:%S ") + t])[-30:]
def authed(c):
    t = c.cookies.get("gcs", "")
    return any(hmac.compare_digest(t, k) for k in tokens) if t else False

# ---------- MAVLink ----------
def mav_loop():
    global mav, last_hb
    while True:
        try:
            m = mavutil.mavlink_connection(CONN, baud=BAUD)
            if not m.wait_heartbeat(timeout=10): say("No heartbeat, retrying"); continue
            m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            mav = m; say(f"Connected to system {m.target_system}")
            while True:
                x = m.recv_match(blocking=True, timeout=1)
                if time.time() - last_hb > 3: S["connected"] = False
                if not x: continue
                t = x.get_type()
                if t == "HEARTBEAT" and x.type != mavutil.mavlink.MAV_TYPE_GCS:
                    last_hb = time.time(); S.update(connected=True, armed=bool(m.motors_armed()), mode=m.flightmode)
                elif t == "GLOBAL_POSITION_INT": S.update(lat=x.lat/1e7, lon=x.lon/1e7, alt=x.relative_alt/1000, hdg=x.hdg/100)
                elif t == "VFR_HUD": S["gs"] = x.groundspeed
                elif t == "ATTITUDE": S.update(roll=x.roll*57.2958, pitch=x.pitch*57.2958)
                elif t == "SYS_STATUS": S.update(volt=x.voltage_battery/1000, batt=x.battery_remaining)
                elif t == "GPS_RAW_INT": S.update(sats=x.satellites_visible, fix=x.fix_type)
                elif t == "STATUSTEXT": say(x.text)
        except Exception as e:
            S["connected"] = False; say(f"Link error: {e}"); time.sleep(2)

def command(c):
    cmd = c.get("cmd")
    if cmd == "detect": S["detect"] = not S["detect"]; return
    if not mav or not S["connected"]: return say("Not connected")
    ml, ts, tc = mavutil.mavlink, mav.target_system, mav.target_component
    with lock:
        if cmd in ("arm", "disarm"):
            mav.mav.command_long_send(ts, tc, ml.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if cmd == "arm" else 0, 0,0,0,0,0,0)
        elif cmd == "mode": mav.set_mode(mav.mode_mapping()[c["mode"]])
        elif cmd == "takeoff": mav.mav.command_long_send(ts, tc, ml.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, float(c["alt"]))
        elif cmd == "goto":
            mav.mav.set_position_target_global_int_send(0, ts, tc, ml.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
                int(c["lat"]*1e7), int(c["lon"]*1e7), float(c["alt"]), 0,0,0,0,0,0,0,0)
        elif cmd in ("drop", "reset_drop"):
            mav.mav.command_long_send(ts, tc, ml.MAV_CMD_DO_SET_SERVO, 0, SERVO, PWM_OPEN if cmd == "drop" else PWM_CLOSED, 0,0,0,0,0)
            S["dropped"] = cmd == "drop"
    say(f"Sent: {cmd}")

# ---------- Camera + object detection ----------
def cam_loop():
    global frame_jpg
    try:
        from ultralytics import YOLO
        model = YOLO("yolov8n.pt")            # downloads once on first run
    except Exception:
        model = None; say("Detection off: pip install ultralytics")
    while True:
        cap = cv2.VideoCapture(CAM)
        if not cap.isOpened(): S["cam"] = False; time.sleep(3); continue
        S["cam"] = True; t0, n = time.time(), 0
        while True:
            ok, img = cap.read()
            if not ok: break
            if model and S["detect"]:
                r = model(img, imgsz=416, verbose=False)[0]
                img = r.plot()
                names = [model.names[int(k)] for k in r.boxes.cls]
                S["objects"] = {k: names.count(k) for k in set(names)}
            else: S["objects"] = {}
            n += 1
            if time.time() - t0 >= 1: S["fps"], n, t0 = n, 0, time.time()
            frame_jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        cap.release(); S["cam"] = False; say("Camera lost, retrying")

def frames():
    last = None
    while True:
        f = frame_jpg
        if f is not None and f is not last:
            last = f; yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
        else: time.sleep(0.03)

# ---------- Web ----------
@app.post("/api/login")
async def login(req: Request):
    b = await req.json()
    good = hmac.compare_digest(str(b.get("user", "")), USER) & hmac.compare_digest(str(b.get("password", "")), PASS)
    if not good: await asyncio.sleep(1); return JSONResponse({"ok": False}, 401)
    t = secrets.token_urlsafe(32); tokens.add(t)
    r = JSONResponse({"ok": True}); r.set_cookie("gcs", t, httponly=True, samesite="strict"); return r

@app.post("/api/logout")
def logout(req: Request):
    tokens.discard(req.cookies.get("gcs", "")); return JSONResponse({"ok": True})

@app.get("/login")
def login_page(): return FileResponse(os.path.join(HERE, "static", "login.html"))

@app.get("/")
def index(req: Request):
    if not authed(req): return RedirectResponse("/login")
    return FileResponse(os.path.join(HERE, "static", "index.html"))

@app.get("/video")
def video(req: Request):
    if not authed(req): return JSONResponse({}, 401)
    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=f")

@app.websocket("/ws")
async def ws(sock: WebSocket):
    if not authed(sock): return await sock.close(code=1008)
    await sock.accept()
    async def push():
        while True: await sock.send_text(json.dumps(S)); await asyncio.sleep(0.2)
    task = asyncio.create_task(push())
    try:
        while True: await asyncio.get_event_loop().run_in_executor(None, command, json.loads(await sock.receive_text()))
    except WebSocketDisconnect: pass
    finally: task.cancel()

if __name__ == "__main__":
    threading.Thread(target=mav_loop, daemon=True).start()
    threading.Thread(target=cam_loop, daemon=True).start()
    uvicorn.run(app, host="127.0.0.1", port=8000)
