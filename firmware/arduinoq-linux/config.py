import os

# ---------------------------------------------------------------------------
# Serial port — ArduinoQ LPUSART → Linux
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyHS1"
SERIAL_BAUD = 115200

# ---------------------------------------------------------------------------
# TCP server — LabVIEW connects here (over LAN from Windows PC)
# ---------------------------------------------------------------------------
TCP_HOST = "0.0.0.0"
TCP_PORT = 9000

# ---------------------------------------------------------------------------
# Soiling alert thresholds (power ratio vs reference panel)
# ---------------------------------------------------------------------------
THRESHOLD_SCHEDULE = 0.90   # below 90 % → schedule a clean
THRESHOLD_URGENT   = 0.75   # below 75 % → clean immediately

# Which INA219 panel is the always-clean reference (0=P1, 1=P2, 2=P3)
REFERENCE_PANEL = 2          # P3 = shaded/covered reference

# ---------------------------------------------------------------------------
# UV sensor calibration
# ---------------------------------------------------------------------------
# GUVA-S12S: UV index ≈ uv_mv / UV_MV_PER_INDEX.  Tune after outdoor calibration.
UV_MV_PER_INDEX = 110

# ---------------------------------------------------------------------------
# Edge Impulse
# ---------------------------------------------------------------------------
EI_WINDOW_SECONDS = 30
EI_API_KEY        = os.environ.get("EI_API_KEY", "ei_REPLACE_ME")
EI_INGEST_URL     = "https://ingestion.edgeimpulse.com/api/training/data"
EI_CLASSIFY_URL   = os.environ.get("EI_CLASSIFY_URL", "")   # REST classify endpoint
EI_DEVICE_NAME    = "solarsense-node-01"

# ---------------------------------------------------------------------------
# Legacy MQTT settings (mqtt_bridge.py only — not used by serial_bridge.py)
# ---------------------------------------------------------------------------
MQTT_HOST      = "192.168.1.100"
MQTT_PORT      = 1883
MQTT_KEEPALIVE = 60
MQTT_TOPIC_ROOT = "solarsense/#"
