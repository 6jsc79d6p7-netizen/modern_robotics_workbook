// pick_place phone teleop: stream ARKit/ARCore viewer pose + clutch + gripper
// over WSS to the robot, and play haptics (vibrate / beep) sent back.
// Adapted from simple_slam/web/xr.js.
const $ = (id) => document.getElementById(id);

let ws = null;
let xrSession = null;
let clutch = false;              // toggle (not press-hold) — the ratchet
let gripper = 255;               // 0 closed .. 255 open
let sentCount = 0;
let audioCtx = null;

function setState(on, text) {
  $("dot").classList.toggle("on", on);
  $("state").textContent = text;
}

function connect() {
  ws = new WebSocket(`wss://${location.host}/ws`);
  ws.onopen = () => {
    ws.send(JSON.stringify({ type: "hello", role: "xr" }));
    setState(true, "connected");
  };
  ws.onclose = () => { setState(false, "disconnected — retrying…"); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === "haptic") playHaptic(m);
    else if (m.type === "instruction") $("objective").textContent = m.text;
  };
}

// --- haptics received from the robot (collision feedback) ---
function playHaptic(m) {
  if (m.channel === "vibrate" && navigator.vibrate) {
    navigator.vibrate(m.pattern || m.ms || 40);   // pattern = distinct collision buzz
  } else if (m.channel === "sound") {
    beep(m.freq || 440, m.ms || 120);
  }
}
function beep(freq, ms) {
  try {
    audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
    const o = audioCtx.createOscillator(), g = audioCtx.createGain();
    o.frequency.value = freq; o.connect(g); g.connect(audioCtx.destination);
    g.gain.setValueAtTime(0.25, audioCtx.currentTime);
    o.start();
    o.stop(audioCtx.currentTime + ms / 1000);
  } catch (e) {}
}

// --- controls (dom-overlay buttons; pointer-events:auto so taps don't leak to XR select) ---
function setClutch(on) {
  clutch = on;
  const b = $("clutchBtn");
  b.textContent = on ? "CLUTCH ON" : "CLUTCH OFF";
  b.classList.toggle("on", on);
  b.classList.toggle("off", !on);
}
function wireControls() {
  $("clutchBtn").addEventListener("click", (e) => { e.stopPropagation(); setClutch(!clutch); });
  $("homeBtn").addEventListener("click", (e) => {
    e.stopPropagation();
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: "home" }));
    setClutch(false);                        // disengage so it doesn't re-yank
  });
  $("fwdBtn").addEventListener("click", (e) => {
    e.stopPropagation();
    // point the phone the way you want "into the screen", then tap
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: "setforward" }));
  });
  $("discardBtn").addEventListener("click", (e) => {
    e.stopPropagation();
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: "discard" }));
    setClutch(false);
  });
  $("openBtn").addEventListener("click", (e) => { e.stopPropagation(); gripper = 255; });
  $("closeBtn").addEventListener("click", (e) => { e.stopPropagation(); gripper = 0; });
}

async function startAR() {
  xrSession = await navigator.xr.requestSession("immersive-ar", {
    optionalFeatures: ["dom-overlay"],
    domOverlay: { root: $("overlay") },
  });
  $("overlay").style.display = "block";

  const glCanvas = document.createElement("canvas");
  const gl = glCanvas.getContext("webgl", { xrCompatible: true });
  await xrSession.updateRenderState({ baseLayer: new XRWebGLLayer(xrSession, gl) });

  // 'local' space: metres, y-up, origin + yaw fixed at session start = frame {ar}
  const refSpace = await xrSession.requestReferenceSpace("local");

  xrSession.addEventListener("end", () => {
    xrSession = null; setClutch(false);
    $("overlay").style.display = "none";
    $("support").textContent = "AR session ended.";
  });

  const onFrame = (t, frame) => {
    if (!xrSession) return;
    xrSession.requestAnimationFrame(onFrame);
    const pose = frame.getViewerPose(refSpace);
    if (!pose || !ws || ws.readyState !== 1) return;
    const p = pose.transform.position, q = pose.transform.orientation;
    ws.send(JSON.stringify({
      type: "xr", t,
      p: [p.x, p.y, p.z],
      q: [q.x, q.y, q.z, q.w],
      clutch, gripper,
      tracked: !pose.emulatedPosition,
    }));
    if (++sentCount % 15 === 0) {
      $("xrStats").textContent =
        `${p.x.toFixed(2)} ${p.y.toFixed(2)} ${p.z.toFixed(2)} m · ` +
        `${pose.emulatedPosition ? "3-DoF (no position!)" : "6-DoF"} · ` +
        `clutch ${clutch ? "ON" : "off"} · grip ${gripper}`;
    }
  };
  xrSession.requestAnimationFrame(onFrame);
}

(async function init() {
  connect();
  wireControls();
  if (!navigator.xr) { $("support").textContent = "WebXR not available in this browser."; return; }
  const ok = await navigator.xr.isSessionSupported("immersive-ar").catch(() => false);
  if (!ok) { $("support").textContent = "WebXR present but immersive-ar isn't supported/enabled."; return; }
  $("support").textContent = "immersive-ar supported — ready.";
  const btn = $("startBtn");
  btn.disabled = false;
  btn.addEventListener("click", () =>
    startAR().catch((e) => { $("support").textContent = `failed to start AR: ${e.message}`; }));
})();
