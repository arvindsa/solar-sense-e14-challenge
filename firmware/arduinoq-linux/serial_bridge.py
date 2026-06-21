#!/usr/bin/env python3
"""
SolarSense serial → TCP bridge for LabVIEW.

Reads TLM lines from ArduinoQ LPUSART (/dev/ttyHS1 at 115200),
computes soiling ratios / alert status, and pushes every update
to LabVIEW clients as newline-delimited JSON over TCP port 9000.

The JSON format and field names are unchanged from the MQTT bridge
(mqtt_bridge.py), so the existing LabVIEW VI and WIRING_SPEC.md
remain fully compatible.

Also ingests data windows to Edge Impulse for training, and can
classify via the EI REST API when EI_CLASSIFY_URL is set.

Run on ArduinoQ Linux:
    pip install pyserial requests
    export EI_API_KEY=ei_xxxxxxxxxxxx
    python serial_bridge.py

LabVIEW: set Bridge IP to ArduinoQ's LAN address, port 9000.
"""

import json
import logging
import os
import socket
import threading
import time
from collections import deque

import requests
import serial

import config

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("solarsense-serial-bridge")

# ---------------------------------------------------------------------------
# Shared state (same fields as mqtt_bridge.py for LabVIEW compatibility)
# ---------------------------------------------------------------------------
state = {
    "panel1_ratio":    None,
    "panel2_ratio":    None,
    "uv_index":        None,
    "aqi":             None,
    "humidity":        None,
    "temp":            None,
    "battery_soc":     None,
    "cleaning_status": "unknown",
    "ts":              None,
}
state_lock = threading.Lock()

ei_window = deque()
ei_window_lock = threading.Lock()


# ---------------------------------------------------------------------------
# TLM parsing
# ---------------------------------------------------------------------------
def parse_tlm(line: str) -> dict | None:
    """Parse 'TLM key=val ...' into a dict.  Hex values (0x..) are accepted."""
    if not line.startswith("TLM "):
        return None
    out: dict = {}
    for token in line[4:].split():
        k, _, v = token.partition("=")
        if not k:
            continue
        try:
            out[k] = int(v, 0)
        except ValueError:
            out[k] = v
    return out


def uv_mv_to_index(uv_mv: int) -> float:
    """Convert GUVA-S12S millivolts to approximate UV index.

    Linear fit: UV index ≈ uv_mv / 110.
    Tune the divisor in config.py (UV_MV_PER_INDEX) once you have outdoor
    readings against a reference instrument.
    """
    return round(max(0.0, uv_mv / config.UV_MV_PER_INDEX), 1)


def _clamp_ratio(r: float) -> float:
    return min(max(r, 0.0), 2.0)


def compute_panel_ratios(tlm: dict):
    """Return (p1_ratio, p2_ratio) vs the reference panel (config.REFERENCE_PANEL).

    P3 (index 2) is assumed to be the always-clean reference.
    Returns (None, None) when the reference panel has no valid reading.
    """
    ref_idx = config.REFERENCE_PANEL       # 0=P1, 1=P2, 2=P3
    ref_ok_key  = f"p{ref_idx+1}_ok"
    ref_mw_key  = f"p{ref_idx+1}_mw"
    ref_ok  = tlm.get(ref_ok_key, 0)
    ref_mw  = tlm.get(ref_mw_key, 0)

    if not ref_ok or ref_mw <= 0:
        return None, None

    test_indices = [i for i in range(3) if i != ref_idx]
    ratios = []
    for i in test_indices:
        ok  = tlm.get(f"p{i+1}_ok", 0)
        mw  = tlm.get(f"p{i+1}_mw", 0)
        ratios.append(_clamp_ratio(mw / ref_mw) if ok else None)

    p1 = ratios[0] if len(ratios) > 0 else None
    p2 = ratios[1] if len(ratios) > 1 else None
    return p1, p2


def compute_alert(p1_ratio, p2_ratio) -> str:
    worst = min(
        p1_ratio if p1_ratio is not None else 1.0,
        p2_ratio if p2_ratio is not None else 1.0,
    )
    if worst < config.THRESHOLD_URGENT:
        return "urgent"
    if worst < config.THRESHOLD_SCHEDULE:
        return "schedule"
    return "clean"


def tlm_to_state(tlm: dict) -> dict:
    """Map raw TLM integers to the dashboard state dict."""
    p1_ratio, p2_ratio = compute_panel_ratios(tlm)

    hx_ok    = tlm.get("hx_ok", 0)
    bmp_ok   = tlm.get("bmp_ok", 0)
    humidity = tlm.get("hx_rh", 0) / 10.0  if hx_ok  else None
    temp     = tlm.get("hx_t",  0) / 100.0 if hx_ok  else \
               tlm.get("bmp_c", 0) / 100.0 if bmp_ok else None
    aqi      = tlm.get("aq_pm25", 0) / 10.0 if tlm.get("aq_ok", 0) else None
    uv       = uv_mv_to_index(tlm.get("uv_mv", 0))
    soc      = tlm.get("bat_soc", 0) / 10.0

    return {
        "panel1_ratio":    p1_ratio,
        "panel2_ratio":    p2_ratio,
        "uv_index":        uv,
        "aqi":             aqi,
        "humidity":        humidity,
        "temp":            temp,
        "battery_soc":     soc,
        "cleaning_status": compute_alert(p1_ratio, p2_ratio),
        "ts":              time.time(),
    }


# ---------------------------------------------------------------------------
# TCP server (same protocol as mqtt_bridge.py — JSON line per update)
# ---------------------------------------------------------------------------
tcp_clients: list = []
tcp_clients_lock = threading.Lock()


def tcp_broadcast(snapshot: dict) -> None:
    data = (json.dumps(snapshot) + "\n").encode("utf-8")
    with tcp_clients_lock:
        dead = []
        for conn in tcp_clients:
            try:
                conn.sendall(data)
            except OSError:
                dead.append(conn)
        for conn in dead:
            tcp_clients.remove(conn)
            log.info("LabVIEW client disconnected")


def tcp_server_thread() -> None:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((config.TCP_HOST, config.TCP_PORT))
    srv.listen(5)
    log.info("TCP server on %s:%d", config.TCP_HOST, config.TCP_PORT)
    while True:
        conn, addr = srv.accept()
        log.info("LabVIEW connected from %s", addr)
        with tcp_clients_lock:
            tcp_clients.append(conn)
        with state_lock:
            snap = dict(state)
        try:
            conn.sendall((json.dumps(snap) + "\n").encode("utf-8"))
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Serial reader
# ---------------------------------------------------------------------------
def serial_reader_thread() -> None:
    while True:
        try:
            with serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=2) as ser:
                log.info("Serial open: %s @ %d baud", config.SERIAL_PORT, config.SERIAL_BAUD)
                while True:
                    raw = ser.readline().decode("ascii", errors="replace").strip()
                    if not raw.startswith("TLM "):
                        continue
                    tlm = parse_tlm(raw)
                    if tlm is None:
                        continue

                    snap = tlm_to_state(tlm)
                    with state_lock:
                        state.update(snap)

                    tcp_broadcast(snap)
                    _ei_queue(snap)

                    result = ei_classify()
                    if result:
                        log.info("EI classify: %s", result)
        except Exception as exc:
            log.error("Serial error: %s — retrying in 3 s", exc)
            time.sleep(3)


# ---------------------------------------------------------------------------
# Edge Impulse — data ingestion
# ---------------------------------------------------------------------------
def _ei_queue(snap: dict) -> None:
    with ei_window_lock:
        ei_window.append({
            "ts":           snap["ts"],
            "panel1_ratio": snap["panel1_ratio"],
            "panel2_ratio": snap["panel2_ratio"],
            "uv_index":     snap["uv_index"],
            "aqi":          snap["aqi"],
            "humidity":     snap["humidity"],
            "temp":         snap["temp"],
            "label":        snap["cleaning_status"],
        })


def ei_ingest_thread() -> None:
    """Drain the rolling window and POST labeled samples to Edge Impulse."""
    while True:
        time.sleep(config.EI_WINDOW_SECONDS)

        with ei_window_lock:
            if not ei_window:
                continue
            rows  = list(ei_window)
            label = rows[-1]["label"]
            ei_window.clear()

        if not config.EI_API_KEY or config.EI_API_KEY == "ei_REPLACE_ME":
            log.debug("EI_API_KEY not set — skipping ingestion")
            continue

        complete = [
            r for r in rows
            if all(r[k] is not None
                   for k in ("panel1_ratio", "panel2_ratio",
                              "uv_index", "aqi", "humidity", "temp"))
        ]
        if not complete:
            log.info("EI window skipped — incomplete sensor data")
            continue

        payload = {
            "protected": {"ver": "v1", "alg": "none", "iat": 0},
            "signature": "0000",
            "payload": {
                "device_name": config.EI_DEVICE_NAME,
                "device_type": "SOLARSENSE_ARDUINOQ",
                "interval_ms": int(
                    config.EI_WINDOW_SECONDS * 1000 / max(len(complete), 1)
                ),
                "sensors": [
                    {"name": "panel1_ratio", "units": "ratio"},
                    {"name": "panel2_ratio", "units": "ratio"},
                    {"name": "uv_index",     "units": "uW/cm2"},
                    {"name": "aqi",          "units": "ug/m3"},
                    {"name": "humidity",     "units": "%RH"},
                    {"name": "temp",         "units": "C"},
                ],
                "values": [
                    [r["panel1_ratio"], r["panel2_ratio"],
                     r["uv_index"],     r["aqi"],
                     r["humidity"],     r["temp"]]
                    for r in complete
                ],
            },
        }
        try:
            resp = requests.post(
                config.EI_INGEST_URL,
                headers={
                    "x-api-key":    config.EI_API_KEY,
                    "x-label":      label,
                    "Content-Type": "application/json",
                },
                data=json.dumps(payload),
                timeout=10,
            )
            log.info("EI ingest: label=%s rows=%d status=%d",
                     label, len(complete), resp.status_code)
        except requests.RequestException as exc:
            log.error("EI ingest failed: %s", exc)


# ---------------------------------------------------------------------------
# Edge Impulse — model inference via REST classify endpoint
# ---------------------------------------------------------------------------
def ei_classify() -> dict | None:
    """Run the deployed EI impulse on the current state snapshot.

    Requires EI_CLASSIFY_URL to be set in config.py or the environment
    (set it to the URL shown in your EI project's Deployment → REST API page).
    Returns the parsed JSON response or None on error/not configured.
    """
    if not config.EI_CLASSIFY_URL or not config.EI_API_KEY:
        return None
    with state_lock:
        snap = dict(state)
    features = [
        snap.get("panel1_ratio") or 0.0,
        snap.get("panel2_ratio") or 0.0,
        snap.get("uv_index")     or 0.0,
        snap.get("aqi")          or 0.0,
        snap.get("humidity")     or 0.0,
        snap.get("temp")         or 0.0,
    ]
    try:
        resp = requests.post(
            config.EI_CLASSIFY_URL,
            headers={
                "x-api-key":    config.EI_API_KEY,
                "Content-Type": "application/json",
            },
            json={"features": features},
            timeout=5,
        )
        if resp.status_code == 200:
            return resp.json()
        log.warning("EI classify HTTP %d", resp.status_code)
    except requests.RequestException as exc:
        log.warning("EI classify error: %s", exc)
    return None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    threading.Thread(target=tcp_server_thread, daemon=True).start()
    threading.Thread(target=ei_ingest_thread,  daemon=True).start()
    serial_reader_thread()   # blocks; reconnects automatically on serial error


if __name__ == "__main__":
    main()
