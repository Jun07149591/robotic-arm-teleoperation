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
import hashlib
import ipaddress
import json
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
from typing import Any
from urllib.parse import urlparse


WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

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

    function sendLog(text, kind) {
      try {
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({
            type: "log",
            text: String(text),
            kind: kind || "",
            time: new Date().toISOString()
          }));
        }
      } catch (_) {}
    }

    function sendError(message, stack) {
      try {
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({
            type: "error",
            message: String(message || ""),
            stack: String(stack || ""),
            time: new Date().toISOString()
          }));
        }
      } catch (_) {}
    }

    function setMessage(text, kind) {
      message.textContent = text;
      message.className = "mono " + (kind || "");
      sendLog(text, kind);
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
    window.addEventListener("error", (event) => {
      sendError(
        event.message || "window error",
        event.error && event.error.stack ? event.error.stack : `${event.filename || ""}:${event.lineno || 0}:${event.colno || 0}`
      );
    });

    window.addEventListener("unhandledrejection", (event) => {
      const reason = event.reason;
      sendError(
        reason && reason.message ? reason.message : String(reason),
        reason && reason.stack ? reason.stack : ""
      );
    });

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
    return parser.parse_args()


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
        location = packet.get("location", "")
        print(
            f"[client] connected from {addr[0]} secureContext={secure} "
            f"location={location} ua={user_agent}",
            flush=True,
        )
        return

    if packet_type == "log":
        kind = packet.get("kind", "")
        msg = packet.get("text", "")
        print(f"[client-log] {kind}: {msg}", flush=True)
        return

    if packet_type == "error":
        msg = packet.get("message", "")
        stack = packet.get("stack", "")
        print(f"[client-error] {msg}", flush=True)
        if stack:
            print(stack, flush=True)
        return

    if packet_type != "pose":
        print(f"[client-msg] ignored type={packet_type}: {packet}", flush=True)
        return

    TOTAL_PACKETS += 1
    packet["receivedAt"] = dt.datetime.now(dt.UTC).isoformat()
    LATEST_PACKET = packet

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

    print()
    print("After the page opens, press Enter VR in the headset.")
    print("Coordinate space: +X right, +Y up, -Z forward, relative to the selected WebXR reference space.")
    if https:
        print("If the page reports secureContext=false, the headset browser does not trust this certificate.")
    print("Press Ctrl+C here to stop.")
    print()


async def run_server(args: argparse.Namespace) -> None:
    global SAVE_HANDLE

    ips = get_ip_candidates()
    ssl_context = make_ssl_context(args, ips)

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        SAVE_HANDLE = args.save.open("a", encoding="utf-8")

    print_startup(args, ips, ssl_context is not None)

    server = await asyncio.start_server(
        lambda r, w: handle_http_client(r, w, args),
        host=args.host,
        port=args.port,
        ssl=ssl_context,
    )

    async with server:
        await server.serve_forever()


def main() -> int:
    args = parse_args()
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
