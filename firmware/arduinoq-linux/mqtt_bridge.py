"""
SolarSense MQTT → TCP bridge for LabVIEW.

Subscribes to all solarsense/# topics on the MPU's mosquitto broker,
computes alert status, and forwards every update to LabVIEW as
newline-delimited JSON over a TCP socket.

Run on the Windows PC:
    pip install paho-mqtt requests
    python mqtt_bridge.py

LabVIEW connects to TCP 0.0.0.0:9000 (see WIRING_SPEC.md).
"""

import json
import logging
import socket
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt
import requests

import config

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("solarsense-bridge")


# ---------------------------------------------------------------------------
# Shared state — updated by MQTT thread, read by TCP and EI threads
# ---------------------------------------------------------------------------

state = {
    "panel1_ratio":    None,   # test panel 1 current / reference current
    "panel2_ratio":    None,   # test panel 2 current / reference current
    "uv_index":        None,
    "aqi":             None,
    "humidity":        None,
    "temp":            None,
    "battery_soc":     None,
    "cleaning_status": "unknown",
    "ts":              None,   # epoch of last update
}

state_lock = threading.Lock()

# Rolling window for Edge Impulse ingestion
ei_window = deque()
ei_window_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Alert logic
# ---------------------------------------------------------------------------

def compute_alert(p1_ratio, p2_ratio):
    """Return worst-case alert across both test panels."""
    worst = min(
        p1_ratio if p1_ratio is not None else 1.0,
        p2_ratio if p2_ratio is not None else 1.0,
    )
    if worst < config.THRESHOLD_URGENT:
        return "urgent"
    if worst < config.THRESHOLD_SCHEDULE:
        return "schedule"
    return "clean"


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

TOPIC_MAP = {
    "solarsense/panel1/current_ratio": "panel1_ratio",
    "solarsense/panel2/current_ratio": "panel2_ratio",
    "solarsense/env/uv_index":         "uv_index",
    "solarsense/env/aqi":              "aqi",
    "solarsense/env/humidity":         "humidity",
    "solarsense/env/temp":             "temp",
    "solarsense/battery/soc":          "battery_soc",
}


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log.info("Connected to MQTT broker at %s:%d", config.MQTT_HOST, config.MQTT_PORT)
        client.subscribe(config.MQTT_TOPIC_ROOT)
    else:
        log.error("MQTT connection failed, rc=%d", rc)


def on_disconnect(client, userdata, rc):
    log.warning("MQTT disconnected (rc=%d), will auto-reconnect", rc)


def on_message(client, userdata, msg):
    topic = msg.topic
    raw   = msg.payload.decode("utf-8", errors="replace").strip()

    key = TOPIC_MAP.get(topic)
    if key is None:
        return

    try:
        value = float(raw)
    except ValueError:
        log.warning("Non-numeric payload on %s: %r", topic, raw)
        return

    with state_lock:
        state[key]  = value
        state["ts"] = time.time()
        state["cleaning_status"] = compute_alert(
            state["panel1_ratio"], state["panel2_ratio"]
        )
        snapshot = dict(state)

    # Push to TCP clients
    tcp_broadcast(snapshot)

    # Accumulate for Edge Impulse window
    with ei_window_lock:
        ei_window.append({
            "ts":           snapshot["ts"],
            "panel1_ratio": snapshot["panel1_ratio"],
            "panel2_ratio": snapshot["panel2_ratio"],
            "uv_index":     snapshot["uv_index"],
            "aqi":          snapshot["aqi"],
            "humidity":     snapshot["humidity"],
            "temp":         snapshot["temp"],
            "label":        snapshot["cleaning_status"],
        })


# ---------------------------------------------------------------------------
# TCP server — LabVIEW connects here
# ---------------------------------------------------------------------------

tcp_clients = []
tcp_clients_lock = threading.Lock()


def tcp_broadcast(snapshot):
    """Send snapshot as newline-delimited JSON to all connected LabVIEW clients."""
    line = json.dumps(snapshot) + "\n"
    data = line.encode("utf-8")
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


def tcp_server_thread():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((config.TCP_HOST, config.TCP_PORT))
    srv.listen(5)
    log.info("TCP server listening on %s:%d", config.TCP_HOST, config.TCP_PORT)

    while True:
        conn, addr = srv.accept()
        log.info("LabVIEW client connected from %s", addr)
        with tcp_clients_lock:
            tcp_clients.append(conn)

        # Send current state immediately so LabVIEW populates on connect
        with state_lock:
            snapshot = dict(state)
        try:
            conn.sendall((json.dumps(snapshot) + "\n").encode())
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Edge Impulse ingestion — fires every EI_WINDOW_SECONDS
# ---------------------------------------------------------------------------

def ei_ingest_thread():
    """Drain the rolling window and POST labeled samples to Edge Impulse."""
    while True:
        time.sleep(config.EI_WINDOW_SECONDS)

        with ei_window_lock:
            if not ei_window:
                continue
            rows  = list(ei_window)
            label = rows[-1]["label"]   # label of window = state at end
            ei_window.clear()

        # Drop rows with any None sensor value
        complete = [
            r for r in rows
            if all(r[k] is not None for k in
                   ("panel1_ratio", "panel2_ratio", "uv_index",
                    "aqi", "humidity", "temp"))
        ]
        if not complete:
            log.info("EI window skipped — incomplete sensor data")
            continue

        payload = {
            "protected": {"ver": "v1", "alg": "none", "iat": 0},
            "signature": "0000",
            "payload": {
                "device_name": config.EI_DEVICE_NAME,
                "device_type": "SOLARSENSE_MPU",
                "interval_ms": int(config.EI_WINDOW_SECONDS * 1000 / len(complete)),
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
                     r["uv_index"], r["aqi"], r["humidity"], r["temp"]]
                    for r in complete
                ],
            },
        }

        try:
            resp = requests.post(
                config.EI_INGEST_URL,
                headers={
                    "x-api-key": config.EI_API_KEY,
                    "x-label":   label,
                    "Content-Type": "application/json",
                },
                data=json.dumps(payload),
                timeout=10,
            )
            log.info("EI ingestion: label=%s rows=%d status=%d",
                     label, len(complete), resp.status_code)
        except requests.RequestException as e:
            log.error("EI ingestion failed: %s", e)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    # TCP server in background thread
    t_tcp = threading.Thread(target=tcp_server_thread, daemon=True)
    t_tcp.start()

    # Edge Impulse ingestion in background thread
    t_ei = threading.Thread(target=ei_ingest_thread, daemon=True)
    t_ei.start()

    # MQTT client — runs forever, auto-reconnects
    mq = mqtt.Client(client_id="solarsense-bridge")
    mq.on_connect    = on_connect
    mq.on_disconnect = on_disconnect
    mq.on_message    = on_message
    mq.reconnect_delay_set(min_delay=2, max_delay=30)

    while True:
        try:
            mq.connect(config.MQTT_HOST, config.MQTT_PORT, config.MQTT_KEEPALIVE)
            mq.loop_forever()
        except (OSError, ConnectionRefusedError) as e:
            log.error("Cannot reach broker: %s — retrying in 5s", e)
            time.sleep(5)


if __name__ == "__main__":
    main()
