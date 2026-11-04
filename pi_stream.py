"""Run this ON the Raspberry Pi: streams the Pi camera as MJPEG on port 8080."""
import io, socketserver
from http import server
from threading import Condition
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

class Out(io.BufferedIOBase):
    def __init__(s): s.frame, s.cond = None, Condition()
    def write(s, b):
        with s.cond: s.frame = b; s.cond.notify_all()

class H(server.BaseHTTPRequestHandler):
    def do_GET(s):
        if s.path != "/stream.mjpg": return s.send_error(404)
        s.send_response(200); s.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME"); s.end_headers()
        try:
            while True:
                with out.cond: out.cond.wait(); f = out.frame
                s.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(f) + f + b"\r\n")
        except Exception: pass
    def log_message(s, *a): pass

class Srv(socketserver.ThreadingMixIn, server.HTTPServer): daemon_threads = True

cam = Picamera2(); cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
out = Out(); cam.start_recording(MJPEGEncoder(), FileOutput(out))
print("Streaming on http://<pi-ip>:8080/stream.mjpg"); Srv(("", 8080), H).serve_forever()
