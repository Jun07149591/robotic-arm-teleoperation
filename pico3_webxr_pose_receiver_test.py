#!/usr/bin/env python3
"""
Receive PICO/WebXR controller poses from a headset browser.

This file is intentionally standalone and uses only the Python standard library.
It serves a small WebXR page and a WebSocket endpoint on the same port. Open the
page in the headset browser, enter VR, and the page streams controller poses
back to this Python process.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import datetime as dt
from dataclasses import dataclass, field
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import shutil
import socket
import ssl
import struct
import subprocess
import sys
import tempfile
import time
import threading
from typing import Any
from urllib.parse import urlparse


WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

@dataclass
class AppState:
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    server_running: bool = False
    server_error: str | None = None
    server_mode: str = "starting"
    server_urls: list[str] = field(default_factory=list)
    cert_path: str | None = None
    key_path: str | None = None
    latest_packet: dict[str, Any] | None = None
    latest_hello: dict[str, Any] | None = None
    client_connected: bool = False
    client_addr: tuple[str, int] | None = None
    packet_count: int = 0
    last_packet_at: float = 0.0
    last_event: str = "starting"


APP_STATE = AppState()

LATEST_PACKET: dict[str, Any] | None = None
LAST_PRINT_AT = 0.0
TOTAL_PACKETS = 0
SAVE_HANDLE = None


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PICO WebXR Pose Receiver</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101216;
      color: #f4f7fb;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 28px;
      background:
        radial-gradient(circle at 20% 20%, rgba(59, 130, 246, 0.20), transparent 32%),
        linear-gradient(135deg, #101216 0%, #182029 52%, #111827 100%);
    }
    main {
      width: min(900px, 100%);
      display: grid;
      gap: 18px;
    }
    h1 {
      margin: 0;
      font-size: clamp(28px, 5vw, 48px);
      letter-spacing: 0;
    }
    .panel {
      border: 1px solid rgba(255, 255, 255, 0.14);
      background: rgba(18, 24, 33, 0.78);
      border-radius: 8px;
      padding: 18px;
      box-shadow: 0 18px 60px rgba(0, 0, 0, 0.25);
    }
    .row {
      display: flex;
      align-items: center;
      flex-wrap: wrap;
      gap: 12px;
    }
    button {
      appearance: none;
      border: 0;
      border-radius: 8px;
      min-height: 44px;
      padding: 0 18px;
      background: #2dd4bf;
      color: #071311;
      font-weight: 800;
      cursor: pointer;
    }
    button:disabled {
      cursor: not-allowed;
      opacity: 0.45;
    }
    code, .mono {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 13px;
      overflow-wrap: anywhere;
    }
    .status {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(210px, 1fr));
      gap: 10px;
    }
    .kv {
      min-height: 54px;
      padding: 10px 12px;
      border-radius: 8px;
      background: rgba(255, 255, 255, 0.06);
    }
    .label {
      display: block;
      color: #9ca3af;
      font-size: 12px;
      margin-bottom: 4px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      table-layout: fixed;
    }
    th, td {
      padding: 10px 8px;
      border-bottom: 1px solid rgba(255, 255, 255, 0.10);
      text-align: left;
      vertical-align: top;
    }
    th {
      color: #9ca3af;
      font-size: 12px;
      font-weight: 700;
    }
    td {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .ok { color: #5eead4; }
    .bad { color: #fca5a5; }
    .warn { color: #fde68a; }
    canvas { display: none; }
  </style>
</head>
<body>
  <main>
    <section>
      <h1>PICO WebXR Pose Receiver</h1>
    </section>

    <section class="panel">
      <div class="row">
        <button id="enterVr" disabled>Enter VR</button>
        <span id="message" class="mono">Checking WebXR...</span>
      </div>
    </section>

    <section class="panel status">
      <div class="kv"><span class="label">Secure context</span><span id="secureState" class="mono"></span></div>
      <div class="kv"><span class="label">WebSocket</span><span id="wsState" class="mono"></span></div>
      <div class="kv"><span class="label">XR support</span><span id="xrState" class="mono"></span></div>
      <div class="kv"><span class="label">Reference space</span><span id="refState" class="mono">none</span></div>
    </section>

    <section class="panel">
      <table>
        <thead>
          <tr>
            <th>Hand</th>
            <th>Grip position</th>
            <th>Grip quaternion</th>
            <th>Target ray position</th>
            <th>Buttons</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>left</td>
            <td id="leftPos">--</td>
            <td id="leftQuat">--</td>
            <td id="leftRay">--</td>
            <td id="leftButtons">--</td>
          </tr>
          <tr>
            <td>right</td>
            <td id="rightPos">--</td>
            <td id="rightQuat">--</td>
            <td id="rightRay">--</td>
            <td id="rightButtons">--</td>
          </tr>
        </tbody>
      </table>
    </section>
  </main>

  <canvas id="xrCanvas"></canvas>

  <script>
    "use strict";

    const SEND_HZ = 30;
    const SEND_INTERVAL_MS = 1000 / SEND_HZ;

    const enterButton = document.getElementById("enterVr");
    const message = document.getElementById("message");
    const secureState = document.getElementById("secureState");
    const wsState = document.getElementById("wsState");
    const xrState = document.getElementById("xrState");
    const refState = document.getElementById("refState");

    let ws = null;
    let xrSession = null;
    let xrRefSpace = null;
    let xrRefSpaceType = "none";
    let gl = null;
    let seq = 0;
    let lastSendAt = 0;

    function setText(id, text) {
      document.getElementById(id).textContent = text;
    }

    function cls(el, name) {
      el.className = name;
    }

    function setMessage(text, kind) {
      message.textContent = text;
      message.className = "mono " + (kind || "");
    }

    function round6(v) {
      return Number.isFinite(v) ? Math.round(v * 1000000) / 1000000 : null;
    }

    function fmtVec(v) {
      if (!v) return "--";
      return `${v.x.toFixed(3)}, ${v.y.toFixed(3)}, ${v.z.toFixed(3)}`;
    }

    function fmtQuat(q) {
      if (!q) return "--";
      return `${q.x.toFixed(3)}, ${q.y.toFixed(3)}, ${q.z.toFixed(3)}, ${q.w.toFixed(3)}`;
    }

    function poseToJson(pose) {
      if (!pose) return null;
      const p = pose.transform.position;
      const q = pose.transform.orientation;
      return {
        position: { x: round6(p.x), y: round6(p.y), z: round6(p.z) },
        orientation: { x: round6(q.x), y: round6(q.y), z: round6(q.z), w: round6(q.w) },
        matrix: Array.from(pose.transform.matrix, round6),
        emulatedPosition: Boolean(pose.emulatedPosition)
      };
    }

    function gamepadToJson(gamepad) {
      if (!gamepad) return null;
      return {
        id: gamepad.id || "",
        mapping: gamepad.mapping || "",
        axes: Array.from(gamepad.axes || [], round6),
        buttons: Array.from(gamepad.buttons || [], (button) => ({
          pressed: Boolean(button.pressed),
          touched: Boolean(button.touched),
          value: round6(button.value)
        }))
      };
    }

    function sourceToJson(frame, source) {
      const gripPose = source.gripSpace ? frame.getPose(source.gripSpace, xrRefSpace) : null;
      const rayPose = source.targetRaySpace ? frame.getPose(source.targetRaySpace, xrRefSpace) : null;
      return {
        handedness: source.handedness || "none",
        targetRayMode: source.targetRayMode || "",
        profiles: Array.from(source.profiles || []),
        hasGripSpace: Boolean(source.gripSpace),
        grip: poseToJson(gripPose),
        targetRay: poseToJson(rayPose),
        gamepad: gamepadToJson(source.gamepad)
      };
    }

    function updateTable(sources) {
      const byHand = { left: null, right: null };
      for (const source of sources) {
        if (source.handedness === "left" || source.handedness === "right") {
          byHand[source.handedness] = source;
        }
      }
      for (const hand of ["left", "right"]) {
        const source = byHand[hand];
        const grip = source && source.grip;
        const ray = source && source.targetRay;
        setText(`${hand}Pos`, grip ? fmtVec(grip.position) : "--");
        setText(`${hand}Quat`, grip ? fmtQuat(grip.orientation) : "--");
        setText(`${hand}Ray`, ray ? fmtVec(ray.position) : "--");
        if (source && source.gamepad) {
          const buttons = source.gamepad.buttons.map((b, i) => `${i}:${b.value.toFixed(2)}${b.pressed ? "*" : ""}`);
          setText(`${hand}Buttons`, buttons.join(" "));
        } else {
          setText(`${hand}Buttons`, "--");
        }
      }
    }

    function connectWebSocket() {
      const scheme = location.protocol === "https:" ? "wss:" : "ws:";
      ws = new WebSocket(`${scheme}//${location.host}/ws`);

      ws.addEventListener("open", () => {
        wsState.textContent = "connected";
        cls(wsState, "mono ok");
        ws.send(JSON.stringify({
          type: "hello",
          userAgent: navigator.userAgent,
          secureContext: window.isSecureContext,
          location: location.href
        }));
      });

      ws.addEventListener("close", () => {
        wsState.textContent = "disconnected; retrying";
        cls(wsState, "mono warn");
        setTimeout(connectWebSocket, 1000);
      });

      ws.addEventListener("error", () => {
        wsState.textContent = "error";
        cls(wsState, "mono bad");
      });
    }

    async function initWebXR() {
      secureState.textContent = String(window.isSecureContext);
      cls(secureState, window.isSecureContext ? "mono ok" : "mono bad");

      if (!("xr" in navigator)) {
        xrState.textContent = "navigator.xr unavailable";
        cls(xrState, "mono bad");
        setMessage("WebXR is not available in this browser.", "bad");
        return;
      }

      try {
        const supported = await navigator.xr.isSessionSupported("immersive-vr");
        xrState.textContent = supported ? "immersive-vr supported" : "immersive-vr not supported";
        cls(xrState, supported ? "mono ok" : "mono bad");
        enterButton.disabled = !supported;
        setMessage(supported ? "Ready. Put on the headset and press Enter VR." : "This browser cannot start immersive VR.", supported ? "ok" : "bad");
      } catch (err) {
        xrState.textContent = String(err && err.message ? err.message : err);
        cls(xrState, "mono bad");
        setMessage("WebXR support check failed.", "bad");
      }
    }

    async function startSession() {
      try {
        enterButton.disabled = true;
        setMessage("Starting immersive VR session...", "warn");

        const canvas = document.getElementById("xrCanvas");
        gl = canvas.getContext("webgl", { xrCompatible: true, alpha: false, antialias: false });
        if (!gl) throw new Error("WebGL is unavailable.");
        if (gl.makeXRCompatible) await gl.makeXRCompatible();

        xrSession = await navigator.xr.requestSession("immersive-vr", {
          optionalFeatures: ["local-floor", "bounded-floor"]
        });
        xrSession.addEventListener("end", onSessionEnd);
        xrSession.addEventListener("inputsourceschange", () => {
          setMessage(`Input sources: ${xrSession.inputSources.length}`, "ok");
        });

        xrSession.updateRenderState({ baseLayer: new XRWebGLLayer(xrSession, gl) });

        try {
          xrRefSpace = await xrSession.requestReferenceSpace("local-floor");
          xrRefSpaceType = "local-floor";
        } catch (err) {
          xrRefSpace = await xrSession.requestReferenceSpace("local");
          xrRefSpaceType = "local";
        }
        refState.textContent = xrRefSpaceType;
        cls(refState, "mono ok");

        setMessage("Streaming poses to Python.", "ok");
        xrSession.requestAnimationFrame(onXRFrame);
      } catch (err) {
        enterButton.disabled = false;
        setMessage(String(err && err.message ? err.message : err), "bad");
      }
    }

    function onSessionEnd() {
      xrSession = null;
      xrRefSpace = null;
      xrRefSpaceType = "none";
      refState.textContent = "none";
      cls(refState, "mono");
      enterButton.disabled = false;
      setMessage("VR session ended.", "warn");
    }

    function drawXRFrame(frame, viewerPose) {
      const layer = xrSession.renderState.baseLayer;
      if (!layer || !viewerPose) return;
      gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
      for (const view of viewerPose.views) {
        const viewport = layer.getViewport(view);
        gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
        gl.clearColor(0.01, 0.02, 0.04, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      }
    }

    function onXRFrame(timestamp, frame) {
      if (!xrSession || !xrRefSpace) return;
      xrSession.requestAnimationFrame(onXRFrame);

      const viewerPose = frame.getViewerPose(xrRefSpace);
      drawXRFrame(frame, viewerPose);

      const now = performance.now();
      if (now - lastSendAt < SEND_INTERVAL_MS) return;
      lastSendAt = now;

      const sources = Array.from(xrSession.inputSources || [], (source) => sourceToJson(frame, source));
      updateTable(sources);

      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
          type: "pose",
          seq: seq++,
          timestampMs: round6(timestamp),
          performanceNowMs: round6(now),
          referenceSpace: xrRefSpaceType,
          secureContext: window.isSecureContext,
          viewer: poseToJson(viewerPose),
          inputSources: sources
        }));
      }
    }

    enterButton.addEventListener("click", startSession);
    connectWebSocket();
    initWebXR();
  </script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve a WebXR page and receive PICO controller poses over WebSocket."
    )
    parser.add_argument("--host", default="0.0.0.0", help="Bind address. Use 127.0.0.1 with adb reverse.")
    parser.add_argument("--port", type=int, default=8443, help="TCP port to listen on.")
    parser.add_argument("--http", action="store_true", help="Use plain HTTP/WS instead of HTTPS/WSS.")
    parser.add_argument(
        "--wifi-https",
        action="store_true",
        help="Explicit Wi-Fi mode: bind to 0.0.0.0 and serve HTTPS/WSS for the headset.",
    )
    parser.add_argument("--cert", type=Path, help="TLS certificate path for HTTPS mode.")
    parser.add_argument("--key", type=Path, help="TLS private key path for HTTPS mode.")
    parser.add_argument(
        "--cert-dir",
        type=Path,
        default=Path(".webxr_certs"),
        help="Directory for the auto-generated self-signed certificate.",
    )
    parser.add_argument("--regen-cert", action="store_true", help="Regenerate the auto certificate.")
    parser.add_argument("--print-rate", type=float, default=10.0, help="Maximum terminal print rate in Hz.")
    parser.add_argument("--raw", action="store_true", help="Print each raw JSON pose packet.")
    parser.add_argument("--save", type=Path, help="Save all pose packets as newline-delimited JSON.")
    parser.add_argument("--no-gui", action="store_true", help="Run in terminal-only mode.")
    args = parser.parse_args()
    if args.wifi_https:
        args.http = False
        if args.host in ("127.0.0.1", "localhost"):
            args.host = "0.0.0.0"
    return args


def get_ip_candidates() -> list[str]:
    ips: set[str] = set()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 80))
            ips.add(sock.getsockname()[0])
        finally:
            sock.close()
    except OSError:
        pass

    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            ips.add(ip)
    except OSError:
        pass

    good: list[str] = []
    for ip in sorted(ips):
        try:
            parsed = ipaddress.ip_address(ip)
        except ValueError:
            continue
        if parsed.version == 4 and not parsed.is_loopback and not parsed.is_link_local:
            good.append(ip)
    return good or ["127.0.0.1"]


def build_cert_config(ips: list[str]) -> str:
    alt_lines = ["DNS.1 = localhost", "IP.1 = 127.0.0.1"]
    idx = 2
    for ip in ips:
        if ip != "127.0.0.1":
            alt_lines.append(f"IP.{idx} = {ip}")
            idx += 1

    return "\n".join(
        [
            "[req]",
            "default_bits = 2048",
            "prompt = no",
            "default_md = sha256",
            "distinguished_name = dn",
            "x509_extensions = v3_req",
            "",
            "[dn]",
            "CN = localhost",
            "",
            "[v3_req]",
            "subjectAltName = @alt_names",
            "",
            "[alt_names]",
            *alt_lines,
            "",
        ]
    )


def ensure_auto_cert(args: argparse.Namespace, ips: list[str]) -> tuple[Path, Path]:
    cert_dir: Path = args.cert_dir
    cert_path = cert_dir / "webxr_pose_cert.pem"
    key_path = cert_dir / "webxr_pose_key.pem"

    if args.regen_cert:
        cert_path.unlink(missing_ok=True)
        key_path.unlink(missing_ok=True)

    if cert_path.exists() and key_path.exists():
        return cert_path, key_path

    openssl = shutil.which("openssl")
    if not openssl:
        raise RuntimeError(
            "openssl was not found. Install openssl, pass --cert/--key, or run with --http for adb-reverse localhost mode."
        )

    cert_dir.mkdir(parents=True, exist_ok=True)
    config_text = build_cert_config(ips)
    with tempfile.NamedTemporaryFile("w", suffix=".cnf", delete=False) as cfg:
        cfg.write(config_text)
        cfg_path = cfg.name

    try:
        subprocess.run(
            [
                openssl,
                "req",
                "-x509",
                "-nodes",
                "-days",
                "3650",
                "-newkey",
                "rsa:2048",
                "-keyout",
                str(key_path),
                "-out",
                str(cert_path),
                "-config",
                cfg_path,
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    finally:
        Path(cfg_path).unlink(missing_ok=True)

    try:
        key_path.chmod(0o600)
    except OSError:
        pass
    return cert_path, key_path


def make_ssl_context(args: argparse.Namespace, ips: list[str]) -> ssl.SSLContext | None:
    if args.http:
        return None

    if bool(args.cert) != bool(args.key):
        raise RuntimeError("Pass both --cert and --key, or pass neither to auto-generate a certificate.")

    cert_path, key_path = (args.cert, args.key) if args.cert and args.key else ensure_auto_cert(args, ips)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=cert_path, keyfile=key_path)
    return context


async def send_http_response(
    writer: asyncio.StreamWriter,
    status: str,
    body: bytes | str,
    content_type: str = "text/plain; charset=utf-8",
    extra_headers: dict[str, str] | None = None,
) -> None:
    body_bytes = body.encode("utf-8") if isinstance(body, str) else body
    headers = {
        "Content-Type": content_type,
        "Content-Length": str(len(body_bytes)),
        "Cache-Control": "no-store",
        "Connection": "close",
        "Permissions-Policy": "xr-spatial-tracking=(self)",
    }
    if extra_headers:
        headers.update(extra_headers)
    header_text = "".join(f"{name}: {value}\r\n" for name, value in headers.items())
    writer.write(f"HTTP/1.1 {status}\r\n{header_text}\r\n".encode("utf-8") + body_bytes)
    await writer.drain()
    writer.close()
    await writer.wait_closed()


async def send_ws_frame(writer: asyncio.StreamWriter, opcode: int, payload: bytes = b"") -> None:
    first = 0x80 | (opcode & 0x0F)
    length = len(payload)
    if length < 126:
        header = struct.pack("!BB", first, length)
    elif length <= 0xFFFF:
        header = struct.pack("!BBH", first, 126, length)
    else:
        header = struct.pack("!BBQ", first, 127, length)
    writer.write(header + payload)
    await writer.drain()


async def read_ws_frame(reader: asyncio.StreamReader) -> tuple[bool, int, bytes] | None:
    try:
        b1, b2 = await reader.readexactly(2)
    except asyncio.IncompleteReadError:
        return None

    fin = bool(b1 & 0x80)
    opcode = b1 & 0x0F
    masked = bool(b2 & 0x80)
    length = b2 & 0x7F

    if length == 126:
        length = struct.unpack("!H", await reader.readexactly(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", await reader.readexactly(8))[0]

    mask = await reader.readexactly(4) if masked else b""
    payload = await reader.readexactly(length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
    return fin, opcode, payload


def format_pose(source: dict[str, Any]) -> str:
    hand = source.get("handedness", "none")
    pose = source.get("grip") or source.get("targetRay")
    pose_name = "grip" if source.get("grip") else "ray"
    if not pose:
        return f"{hand}=no-pose"

    p = pose.get("position") or {}
    q = pose.get("orientation") or {}
    emu = " emulated" if pose.get("emulatedPosition") else ""
    return (
        f"{hand} {pose_name} "
        f"p=({p.get('x', 0):+.3f},{p.get('y', 0):+.3f},{p.get('z', 0):+.3f}) "
        f"q=({q.get('x', 0):+.3f},{q.get('y', 0):+.3f},{q.get('z', 0):+.3f},{q.get('w', 0):+.3f})"
        f"{emu}"
    )


def handle_text_message(text: str, addr: tuple[str, int], args: argparse.Namespace) -> None:
    global LAST_PRINT_AT, LATEST_PACKET, TOTAL_PACKETS, SAVE_HANDLE

    try:
        packet = json.loads(text)
    except json.JSONDecodeError:
        print(f"[{addr[0]}:{addr[1]}] non-json message: {text[:120]}", flush=True)
        return

    packet_type = packet.get("type")
    if packet_type == "hello":
        user_agent = packet.get("userAgent", "unknown")
        secure = packet.get("secureContext")
        with APP_STATE.lock:
            APP_STATE.latest_hello = packet
            APP_STATE.client_connected = True
            APP_STATE.client_addr = addr
            APP_STATE.last_event = "hello"
        print(f"[client] connected from {addr[0]} secureContext={secure} ua={user_agent}", flush=True)
        return

    if packet_type != "pose":
        return

    TOTAL_PACKETS += 1
    packet["receivedAt"] = dt.datetime.now(dt.timezone.utc).isoformat()
    LATEST_PACKET = packet
    # Write latest to file for vr_teleop (fast, no HTTP polling needed).
    # Use atomic replace so the reader never sees a half-written JSON file.
    try:
        latest_path = "/tmp/pico_latest_pose.json"
        tmp_path = latest_path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(packet, f, separators=(",", ":"))
        os.replace(tmp_path, latest_path)
    except Exception:
        pass
    with APP_STATE.lock:
        APP_STATE.latest_packet = packet
        APP_STATE.packet_count += 1
        APP_STATE.last_packet_at = time.time()
        APP_STATE.client_connected = True
        APP_STATE.client_addr = addr
        APP_STATE.last_event = "pose"

    if SAVE_HANDLE:
        SAVE_HANDLE.write(json.dumps(packet, separators=(",", ":")) + "\n")
        SAVE_HANDLE.flush()

    if args.raw:
        print(json.dumps(packet, separators=(",", ":")), flush=True)
        return

    now = time.monotonic()
    interval = 0 if args.print_rate <= 0 else 1.0 / args.print_rate
    if now - LAST_PRINT_AT < interval:
        return
    LAST_PRINT_AT = now

    sources = packet.get("inputSources") or []
    ordered = sorted(sources, key=lambda s: {"left": 0, "right": 1}.get(s.get("handedness"), 2))
    pose_text = " | ".join(format_pose(source) for source in ordered) or "no input sources"
    clock = dt.datetime.now().strftime("%H:%M:%S")
    seq = packet.get("seq", "?")
    ref = packet.get("referenceSpace", "?")
    print(f"{clock} seq={seq} ref={ref} sources={len(sources)} | {pose_text}", flush=True)


async def handle_websocket(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    headers: dict[str, str],
    addr: tuple[str, int],
    args: argparse.Namespace,
) -> None:
    key = headers.get("sec-websocket-key")
    if not key:
        await send_http_response(writer, "400 Bad Request", "Missing Sec-WebSocket-Key\n")
        return

    accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")
    writer.write(
        (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n"
            "\r\n"
        ).encode("ascii")
    )
    await writer.drain()

    while True:
        try:
            frame = await read_ws_frame(reader)
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            break
        if frame is None:
            break

        fin, opcode, payload = frame
        if opcode == 0x8:
            await send_ws_frame(writer, 0x8, payload[:125])
            break
        if opcode == 0x9:
            await send_ws_frame(writer, 0xA, payload)
            continue
        if opcode == 0x1 and fin:
            try:
                handle_text_message(payload.decode("utf-8"), addr, args)
            except UnicodeDecodeError:
                pass

    writer.close()
    try:
        await writer.wait_closed()
    except OSError:
        pass


async def handle_http_client(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    args: argparse.Namespace,
) -> None:
    peer = writer.get_extra_info("peername")
    addr = peer if isinstance(peer, tuple) else ("unknown", 0)

    try:
        request = await reader.readuntil(b"\r\n\r\n")
    except asyncio.LimitOverrunError:
        await send_http_response(writer, "431 Request Header Fields Too Large", "Headers too large\n")
        return
    except asyncio.IncompleteReadError:
        writer.close()
        await writer.wait_closed()
        return

    try:
        text = request.decode("iso-8859-1")
        lines = text.split("\r\n")
        method, raw_path, _version = lines[0].split(" ", 2)
    except ValueError:
        await send_http_response(writer, "400 Bad Request", "Bad request\n")
        return

    headers: dict[str, str] = {}
    for line in lines[1:]:
        if not line or ":" not in line:
            continue
        name, value = line.split(":", 1)
        headers[name.strip().lower()] = value.strip()

    path = urlparse(raw_path).path
    if headers.get("upgrade", "").lower() == "websocket" and path == "/ws":
        await handle_websocket(reader, writer, headers, addr, args)
        return

    if method != "GET":
        await send_http_response(writer, "405 Method Not Allowed", "Only GET is supported\n")
        return

    if path == "/" or path == "/index.html":
        await send_http_response(writer, "200 OK", HTML, "text/html; charset=utf-8")
    elif path == "/health":
        await send_http_response(writer, "200 OK", "ok\n")
    elif path == "/pose/latest":
        body = json.dumps(LATEST_PACKET or {}, indent=2)
        await send_http_response(writer, "200 OK", body, "application/json; charset=utf-8")
    elif path == "/favicon.ico":
        await send_http_response(writer, "204 No Content", b"")
    else:
        await send_http_response(writer, "404 Not Found", "Not found\n")


def print_startup(args: argparse.Namespace, ips: list[str], https: bool) -> None:
    scheme = "http" if args.http else "https"
    bind = args.host
    print()
    print("PICO WebXR pose receiver is running.")
    print(f"  bind:   {bind}:{args.port}")
    print(f"  mode:   {'HTTP/WS' if args.http else 'HTTPS/WSS'}")
    if args.save:
        print(f"  saving: {args.save}")
    print()

    if bind in ("127.0.0.1", "localhost"):
        print("Open in the headset browser after adb reverse:")
        print(f"  {scheme}://localhost:{args.port}/")
    else:
        print("Open one of these URLs in the headset browser:")
        for ip in ips:
            print(f"  {scheme}://{ip}:{args.port}/")
    if https:
        if args.cert and args.key:
            print(f"TLS cert: {args.cert}")
        else:
            print(f"TLS cert: {args.cert_dir / 'webxr_pose_cert.pem'}")

    print()
    print("After the page opens, press Enter VR in the headset.")
    print("Coordinate space: +X right, +Y up, -Z forward, relative to the selected WebXR reference space.")
    if https:
        print("If the page reports secureContext=false, the headset browser does not trust this certificate.")
    print("Press Ctrl+C here to stop.")
    print()


def build_open_urls(args: argparse.Namespace, ips: list[str]) -> list[str]:
    scheme = "http" if args.http else "https"
    if args.host in ("127.0.0.1", "localhost"):
        return [f"{scheme}://localhost:{args.port}/"]
    return [f"{scheme}://{ip}:{args.port}/" for ip in ips]


async def run_server(args: argparse.Namespace) -> None:
    global SAVE_HANDLE

    ips = get_ip_candidates()
    ssl_context = make_ssl_context(args, ips)
    mode = "HTTP/WS" if args.http else "HTTPS/WSS"
    cert_path = None
    key_path = None
    if not args.http:
        if args.cert and args.key:
            cert_path = str(args.cert)
            key_path = str(args.key)
        else:
            cert_path = str(args.cert_dir / "webxr_pose_cert.pem")
            key_path = str(args.cert_dir / "webxr_pose_key.pem")

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        SAVE_HANDLE = args.save.open("a", encoding="utf-8")

    urls = build_open_urls(args, ips)
    with APP_STATE.lock:
        APP_STATE.server_urls = urls
        APP_STATE.server_mode = mode
        APP_STATE.cert_path = cert_path
        APP_STATE.key_path = key_path
        APP_STATE.last_event = "binding server"

    print_startup(args, ips, ssl_context is not None)

    server = await asyncio.start_server(
        lambda r, w: handle_http_client(r, w, args),
        host=args.host,
        port=args.port,
        ssl=ssl_context,
    )

    with APP_STATE.lock:
        APP_STATE.server_running = True
        APP_STATE.server_error = None
        APP_STATE.server_mode = mode
        APP_STATE.cert_path = cert_path
        APP_STATE.key_path = key_path
        APP_STATE.server_urls = urls
        APP_STATE.last_event = "server running"

    async with server:
        await server.serve_forever()


def run_server_thread(args: argparse.Namespace) -> None:
    try:
        asyncio.run(run_server(args))
    except Exception as exc:
        with APP_STATE.lock:
            APP_STATE.server_running = False
            APP_STATE.server_error = str(exc)
            APP_STATE.last_event = "server error"
        print(f"error: {exc}", file=sys.stderr, flush=True)
    finally:
        if SAVE_HANDLE:
            SAVE_HANDLE.close()


def load_tkinter():
    try:
        import tkinter as tk
        from tkinter import ttk
        return tk, ttk
    except Exception:
        return None, None


def vec_from_pose(pose: dict[str, Any] | None) -> tuple[float, float, float] | None:
    if not pose:
        return None
    pos = pose.get("position") or {}
    try:
        return (float(pos["x"]), float(pos["y"]), float(pos["z"]))
    except Exception:
        return None


def quat_from_pose(pose: dict[str, Any] | None) -> tuple[float, float, float, float] | None:
    if not pose:
        return None
    ori = pose.get("orientation") or {}
    try:
        return (float(ori["x"]), float(ori["y"]), float(ori["z"]), float(ori["w"]))
    except Exception:
        return None


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def quaternion_to_yaw_pitch_roll(q: tuple[float, float, float, float] | None) -> tuple[float, float, float] | None:
    if q is None:
        return None
    x, y, z, w = q

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw, pitch, roll


def format_vec(vec: tuple[float, float, float] | None) -> str:
    if vec is None:
        return "--"
    return f"{vec[0]:+.3f}  {vec[1]:+.3f}  {vec[2]:+.3f}"


def format_quat(quat: tuple[float, float, float, float] | None) -> str:
    if quat is None:
        return "--"
    return f"{quat[0]:+.3f}  {quat[1]:+.3f}  {quat[2]:+.3f}  {quat[3]:+.3f}"


def run_gui(args: argparse.Namespace) -> None:
    tk, ttk = load_tkinter()
    if tk is None or ttk is None:
        raise RuntimeError("Tkinter is not available in this Python installation.")

    root = tk.Tk()
    root.title("PICO WebXR Pose Monitor")
    root.geometry("1220x820")
    root.minsize(1120, 760)

    default_font = ("Helvetica", 12)
    mono_font = ("Menlo", 12)
    big_mono = ("Menlo", 16, "bold")

    root.configure(bg="#0f1116")
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:
        pass

    style.configure("TFrame", background="#0f1116")
    style.configure("Panel.TFrame", background="#151922", relief="flat")
    style.configure("TLabel", background="#0f1116", foreground="#e5e7eb", font=default_font)
    style.configure("Heading.TLabel", background="#0f1116", foreground="#f8fafc", font=("Helvetica", 20, "bold"))
    style.configure("Muted.TLabel", background="#0f1116", foreground="#9ca3af", font=default_font)
    style.configure("Panel.TLabel", background="#151922", foreground="#e5e7eb", font=default_font)
    style.configure("Status.TLabel", background="#151922", foreground="#e5e7eb", font=mono_font)
    style.configure("Value.TLabel", background="#151922", foreground="#f8fafc", font=big_mono)
    style.configure("TButton", font=("Helvetica", 12, "bold"))

    root.columnconfigure(0, weight=1)
    root.rowconfigure(1, weight=1)

    top = ttk.Frame(root, style="TFrame", padding=(18, 16))
    top.grid(row=0, column=0, sticky="ew")
    top.columnconfigure(0, weight=1)

    ttk.Label(top, text="PICO WebXR Pose Monitor", style="Heading.TLabel").grid(row=0, column=0, sticky="w")
    status_text = ttk.Label(top, text="starting...", style="Muted.TLabel")
    status_text.grid(row=1, column=0, sticky="w", pady=(6, 0))

    actions = ttk.Frame(top, style="TFrame")
    actions.grid(row=0, column=1, rowspan=2, sticky="e")

    def copy_urls():
        urls = "\n".join(APP_STATE.server_urls or [])
        root.clipboard_clear()
        root.clipboard_append(urls)
        root.update()
        status_text.configure(text="URL copied to clipboard")

    ttk.Button(actions, text="Copy URL", command=copy_urls).grid(row=0, column=0, padx=(0, 8))
    ttk.Button(actions, text="Quit", command=root.destroy).grid(row=0, column=1)

    body = ttk.Frame(root, padding=(18, 0, 18, 18))
    body.grid(row=1, column=0, sticky="nsew")
    body.columnconfigure(0, weight=1)
    body.columnconfigure(1, weight=1)
    body.rowconfigure(0, weight=1)

    left_panel = ttk.Frame(body, style="Panel.TFrame", padding=16)
    left_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
    left_panel.columnconfigure(0, weight=1)
    left_panel.rowconfigure(3, weight=1)

    right_panel = ttk.Frame(body, style="Panel.TFrame", padding=16)
    right_panel.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
    right_panel.columnconfigure(0, weight=1)
    right_panel.rowconfigure(0, weight=1)

    # Connection summary
    summary = ttk.Frame(left_panel, style="Panel.TFrame")
    summary.grid(row=0, column=0, sticky="ew")
    summary.columnconfigure(1, weight=1)

    def add_summary_row(row: int, label: str):
        ttk.Label(summary, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        val = ttk.Label(summary, text="--", style="Value.TLabel")
        val.grid(row=row, column=1, sticky="w", pady=4, padx=(12, 0))
        return val

    srv_val = add_summary_row(0, "Server")
    cli_val = add_summary_row(1, "Client")
    evt_val = add_summary_row(2, "Event")
    pkt_val = add_summary_row(3, "Packets")

    connection = ttk.Frame(left_panel, style="Panel.TFrame")
    connection.grid(row=1, column=0, sticky="ew", pady=(16, 0))
    connection.columnconfigure(1, weight=1)

    def add_connection_row(row: int, label: str):
        ttk.Label(connection, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        val = ttk.Label(connection, text="--", style="Status.TLabel")
        val.grid(row=row, column=1, sticky="w", pady=4, padx=(14, 0))
        return val

    mode_val = add_connection_row(0, "Mode")
    url_val = add_connection_row(1, "Headset URL")
    cert_val = add_connection_row(2, "TLS cert")

    # Controller values
    values = ttk.Frame(left_panel, style="Panel.TFrame")
    values.grid(row=2, column=0, sticky="ew", pady=(16, 0))
    values.columnconfigure(1, weight=1)

    def add_value_row(row: int, label: str):
        ttk.Label(values, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=4)
        val = ttk.Label(values, text="--", style="Status.TLabel")
        val.grid(row=row, column=1, sticky="w", pady=4, padx=(14, 0))
        return val

    left_pos = add_value_row(0, "Left grip position")
    left_quat = add_value_row(1, "Left grip quaternion")
    left_ray = add_value_row(2, "Left target ray position")
    right_pos = add_value_row(3, "Right grip position")
    right_quat = add_value_row(4, "Right grip quaternion")
    right_ray = add_value_row(5, "Right target ray position")

    # Logs
    log_frame = ttk.Frame(left_panel, style="Panel.TFrame")
    log_frame.grid(row=3, column=0, sticky="nsew", pady=(16, 0))
    left_panel.rowconfigure(3, weight=1)
    log_frame.columnconfigure(0, weight=1)
    log_frame.rowconfigure(1, weight=1)

    ttk.Label(log_frame, text="Latest packet", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
    log_text = tk.Text(
        log_frame,
        height=10,
        wrap="word",
        font=("Menlo", 11),
        bg="#0b0f14",
        fg="#d1d5db",
        insertbackground="#d1d5db",
        relief="flat",
        highlightthickness=1,
        highlightbackground="#273041",
        highlightcolor="#273041",
    )
    log_text.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
    log_text.configure(state="disabled")

    # Visual canvas
    canvas = tk.Canvas(
        right_panel,
        bg="#081019",
        highlightthickness=1,
        highlightbackground="#273041",
        width=520,
        height=640,
    )
    canvas.grid(row=0, column=0, sticky="nsew")

    help_box = ttk.Label(
        right_panel,
        text="Top view: X = left/right, Z = forward/back. Bright circle = grip. Arrow = facing direction.",
        style="Muted.TLabel",
        wraplength=520,
        justify="left",
    )
    help_box.grid(row=1, column=0, sticky="ew", pady=(12, 0))

    def draw_arrow(cx: float, cy: float, angle: float, color: str, length: float = 56.0):
        ex = cx + math.sin(angle) * length
        ey = cy - math.cos(angle) * length
        canvas.create_line(cx, cy, ex, ey, fill=color, width=4, arrow=tk.LAST, arrowshape=(12, 16, 6))

    def latest_snapshot():
        with APP_STATE.lock:
            packet = APP_STATE.latest_packet.copy() if APP_STATE.latest_packet else None
            hello = APP_STATE.latest_hello.copy() if APP_STATE.latest_hello else None
            connected = APP_STATE.client_connected
            addr = APP_STATE.client_addr
            count = APP_STATE.packet_count
            last_packet_at = APP_STATE.last_packet_at
            last_event = APP_STATE.last_event
            server_running = APP_STATE.server_running
            server_error = APP_STATE.server_error
            urls = list(APP_STATE.server_urls)
        return packet, hello, connected, addr, count, last_packet_at, last_event, server_running, server_error, urls

    def draw_scene(packet: dict[str, Any] | None):
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        cx = w / 2
        cy = h / 2

        canvas.create_rectangle(20, 20, w - 20, h - 20, outline="#203040", width=2)
        canvas.create_line(40, cy, w - 40, cy, fill="#2b394d", dash=(4, 4))
        canvas.create_line(cx, 40, cx, h - 40, fill="#2b394d", dash=(4, 4))
        canvas.create_text(50, 30, text="-Z forward", fill="#94a3b8", anchor="w", font=("Helvetica", 10))
        canvas.create_text(w - 50, cy + 6, text="+X right", fill="#94a3b8", anchor="e", font=("Helvetica", 10))
        canvas.create_text(cx + 6, 50, text="+Y up", fill="#94a3b8", anchor="w", font=("Helvetica", 10))

        if not packet:
            canvas.create_text(cx, cy, text="Waiting for pose data...", fill="#cbd5e1", font=("Helvetica", 18, "bold"))
            return

        sources = packet.get("inputSources") or []
        max_span = 1.25
        scale = min(w, h) * 0.30 / max_span

        for source in sources:
            hand = source.get("handedness", "none")
            grip = vec_from_pose(source.get("grip"))
            ray = vec_from_pose(source.get("targetRay"))
            grip_q = quat_from_pose(source.get("grip"))
            ray_q = quat_from_pose(source.get("targetRay"))
            if hand == "left":
                grip_color = "#60a5fa"
                ray_color = "#3b82f6"
            elif hand == "right":
                grip_color = "#f59e0b"
                ray_color = "#f97316"
            else:
                grip_color = "#a78bfa"
                ray_color = "#8b5cf6"

            def project(vec: tuple[float, float, float] | None) -> tuple[float, float]:
                if vec is None:
                    return cx, cy
                x, y, z = vec
                px = cx + clamp(x, -max_span, max_span) * scale
                pz = cy + clamp(z, -max_span, max_span) * scale
                return px, pz

            gx, gy = project(grip)
            rx, ry = project(ray)

            canvas.create_oval(gx - 10, gy - 10, gx + 10, gy + 10, fill=grip_color, outline="")
            canvas.create_oval(rx - 6, ry - 6, rx + 6, ry + 6, fill=ray_color, outline="")
            canvas.create_line(gx, gy, rx, ry, fill=ray_color, width=2, dash=(3, 3))
            canvas.create_text(gx + 14, gy - 14, text=f"{hand} grip", fill="#e5e7eb", anchor="w", font=("Helvetica", 10, "bold"))
            canvas.create_text(rx + 10, ry - 10, text=f"{hand} ray", fill="#e5e7eb", anchor="w", font=("Helvetica", 9))

            yaw_pitch_roll = quaternion_to_yaw_pitch_roll(grip_q)
            if yaw_pitch_roll is not None:
                yaw, pitch, roll = yaw_pitch_roll
                draw_arrow(gx, gy, yaw, grip_color, 52)
                canvas.create_text(
                    gx + 14,
                    gy + 16,
                    text=f"yaw {math.degrees(yaw):+.0f}°  pitch {math.degrees(pitch):+.0f}°  roll {math.degrees(roll):+.0f}°",
                    fill="#cbd5e1",
                    anchor="w",
                    font=("Helvetica", 9),
                )

        canvas.create_text(
            28,
            h - 28,
            text="Position is shown in the selected WebXR reference space. X and Z are visualized; Y is listed in the value fields.",
            fill="#94a3b8",
            anchor="sw",
            font=("Helvetica", 9),
        )

    def update_ui():
        packet, hello, connected, addr, count, last_packet_at, last_event, server_running, server_error, urls = latest_snapshot()
        with APP_STATE.lock:
            server_mode = APP_STATE.server_mode
            cert_path = APP_STATE.cert_path or "--"

        srv_val.configure(text="running" if server_running else "starting...")
        if server_error:
            srv_val.configure(text=f"error: {server_error}")
        mode_val.configure(text=server_mode)
        url_val.configure(text=urls[0] if urls else "--")
        cert_val.configure(text=cert_path)

        if connected and addr:
            cli_val.configure(text=f"{addr[0]}:{addr[1]}")
        else:
            cli_val.configure(text="waiting for headset")

        evt_val.configure(text=last_event)
        pkt_val.configure(text=str(count))

        if packet:
            sources = packet.get("inputSources") or []
            left = next((s for s in sources if s.get("handedness") == "left"), None)
            right = next((s for s in sources if s.get("handedness") == "right"), None)
            left_pos.configure(text=format_vec(vec_from_pose(left.get("grip") if left else None)))
            left_quat.configure(text=format_quat(quat_from_pose(left.get("grip") if left else None)))
            left_ray.configure(text=format_vec(vec_from_pose(left.get("targetRay") if left else None)))
            right_pos.configure(text=format_vec(vec_from_pose(right.get("grip") if right else None)))
            right_quat.configure(text=format_quat(quat_from_pose(right.get("grip") if right else None)))
            right_ray.configure(text=format_vec(vec_from_pose(right.get("targetRay") if right else None)))

            viewer = packet.get("viewer") or {}
            ref = packet.get("referenceSpace", "?")
            timestamp_ms = packet.get("performanceNowMs", "?")
            status_text.configure(
                text=(
                    f"referenceSpace={ref}  |  packet={packet.get('seq', '?')}  |  "
                    f"sources={len(sources)}  |  perfNow={timestamp_ms}ms"
                )
            )
        else:
            left_pos.configure(text="--")
            left_quat.configure(text="--")
            left_ray.configure(text="--")
            right_pos.configure(text="--")
            right_quat.configure(text="--")
            right_ray.configure(text="--")
            status_text.configure(text="Open the page in the headset browser, then click Enter VR.")

        if packet:
            pretty = json.dumps(packet, ensure_ascii=False, indent=2)
            log_text.configure(state="normal")
            log_text.delete("1.0", tk.END)
            log_text.insert(tk.END, pretty)
            log_text.configure(state="disabled")

        draw_scene(packet)

        if urls:
            root.title(f"PICO WebXR Pose Monitor - {urls[0]}")
        root.after(33, update_ui)

    update_ui()
    root.mainloop()


def main() -> int:
    args = parse_args()
    if not args.no_gui:
        server_thread = threading.Thread(target=run_server_thread, args=(args,), daemon=True)
        server_thread.start()
        try:
            run_gui(args)
        except KeyboardInterrupt:
            print("\nStopped.")
        except Exception as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        return 0

    try:
        asyncio.run(run_server(args))
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if SAVE_HANDLE:
            SAVE_HANDLE.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
