"""TeleopServer — aiohttp WSS server, run in a background thread.

The MuJoCo sim + viewer must own the main thread (mjpython/Cocoa), so the web
server runs in a daemon thread with its own asyncio loop (AppRunner/TCPSite, no
signal handlers). Two thread-safe bridges connect them:

  phone → sim:  every {type:"xr"} message updates SharedState.latest()
  sim → phone:  send_haptic() enqueues an event the loop pumps to the phone

No relay hop, no SLAM — the robot process *is* the server.
"""
import asyncio
import json
import pathlib
import queue
import ssl
import threading

from aiohttp import WSMsgType, web

from . import gen_cert

HERE = pathlib.Path(__file__).resolve().parent
STATIC = HERE / "static"


class SharedState:
    """Latest phone message + one-shot commands, readable from the sim thread."""
    def __init__(self):
        self._lock = threading.Lock()
        self._msg = None
        self._home = False
        self._setfwd = False
        self._instruction = ""

    def set_instruction(self, text):
        with self._lock:
            self._instruction = text

    def get_instruction(self):
        with self._lock:
            return self._instruction

    def update(self, msg):
        with self._lock:
            self._msg = msg

    def latest(self):
        with self._lock:
            return dict(self._msg) if self._msg else None

    def request_home(self):
        with self._lock:
            self._home = True

    def take_home_request(self):
        with self._lock:
            h, self._home = self._home, False
            return h

    def request_setfwd(self):
        with self._lock:
            self._setfwd = True

    def take_setfwd_request(self):
        with self._lock:
            s, self._setfwd = self._setfwd, False
            return s

    def request_discard(self):
        with self._lock:
            self._discard = True

    def take_discard_request(self):
        with self._lock:
            v, self._discard = getattr(self, "_discard", False), False
            return v


class TeleopServer:
    def __init__(self, port=8443):
        self.port = port
        self.state = SharedState()
        self._haptic_q = queue.Queue()
        self._clients = set()        # ws connections, touched only in loop thread
        self._loop = None
        self._thread = None
        self.ip = gen_cert.lan_ip()
        self._frame = None           # latest wrist-cam JPEG (bytes)
        self._frame_lock = threading.Lock()

    # ---- public API (sim thread) ----
    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def latest(self):
        return self.state.latest()

    def send_haptic(self, channel, **kw):
        """channel: 'vibrate' (phone) or 'sound' (phone beep). Thread-safe."""
        self._haptic_q.put({"type": "haptic", "channel": channel, **kw})

    def send_message(self, msg):
        """Enqueue an arbitrary dict for the phone(s). Thread-safe."""
        self._haptic_q.put(msg)

    def push_instruction(self, text):
        """Set + broadcast the current task instruction to the phone(s)."""
        self.state.set_instruction(text)
        self.send_message({"type": "instruction", "text": text})

    def push_frame(self, jpeg):
        """Publish the latest wrist-cam JPEG for the /wrist browser view."""
        with self._frame_lock:
            self._frame = jpeg

    def _get_frame(self):
        with self._frame_lock:
            return self._frame

    # ---- server internals (loop thread) ----
    def _run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop

        app = web.Application()
        app.add_routes([
            web.get("/", self._index),
            web.get("/ws", self._ws),
            web.get("/wrist", self._wrist_page),
            web.get("/wrist.mjpg", self._wrist_mjpg),
            web.get("/{asset:.+}", self._asset),
        ])
        runner = web.AppRunner(app)
        loop.run_until_complete(runner.setup())

        cert_path, key_path = gen_cert.generate()
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(cert_path, key_path)
        site = web.TCPSite(runner, "0.0.0.0", self.port, ssl_context=ssl_ctx)
        loop.run_until_complete(site.start())
        loop.create_task(self._haptic_pump())
        loop.run_forever()

    async def _index(self, request):
        return web.FileResponse(STATIC / "teleop.html")

    async def _asset(self, request):
        path = (STATIC / request.match_info["asset"]).resolve()
        if STATIC not in path.parents or not path.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={"Cache-Control": "no-cache"})

    async def _wrist_page(self, request):
        return web.Response(content_type="text/html", text=(
            "<!doctype html><html><head><meta charset=utf-8>"
            "<title>wrist cam</title></head>"
            "<body style='margin:0;background:#000;display:flex;"
            "align-items:center;justify-content:center;height:100vh'>"
            "<img src='/wrist.mjpg' style='max-width:100vw;max-height:100vh'>"
            "</body></html>"))

    async def _wrist_mjpg(self, request):
        resp = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-cache"})
        await resp.prepare(request)
        try:
            while True:
                frame = self._get_frame()
                if frame:
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                     + frame + b"\r\n")
                await asyncio.sleep(1 / 30)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def _ws(self, request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._clients.add(ws)
        peer = request.remote
        print(f"[teleop] phone connected ({peer})")
        instr = self.state.get_instruction()          # catch up a late joiner
        if instr:
            await ws.send_str(json.dumps({"type": "instruction", "text": instr}))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    kind = data.get("type")
                    if kind == "xr":
                        self.state.update(data)
                    elif kind == "home":
                        self.state.request_home()
                    elif kind == "setforward":
                        self.state.request_setfwd()
                    elif kind == "discard":
                        self.state.request_discard()
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self._clients.discard(ws)
            print(f"[teleop] phone disconnected ({peer})")
        return ws

    async def _haptic_pump(self):
        """Drain the sim thread's haptic queue and fan out to phones."""
        while True:
            await asyncio.sleep(0.01)
            while True:
                try:
                    event = self._haptic_q.get_nowait()
                except queue.Empty:
                    break
                payload = json.dumps(event)
                for ws in list(self._clients):
                    try:
                        await ws.send_str(payload)
                    except ConnectionError:
                        self._clients.discard(ws)
