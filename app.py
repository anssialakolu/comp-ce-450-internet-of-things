import threading
import datetime
import json

from flask import Flask, render_template, request, jsonify, abort
from paho.mqtt import publish 

app = Flask(__name__)

ROOMS = {
    "1": "Living Room",
    "2": "Bedroom",
}

MAX_POINTS = 500  # limit stored data points

# { room_id: [ {timestamp, temperature, humidity, heater_on}, ... ] }
DATA = {room_id: [] for room_id in ROOMS}
LOCK = threading.Lock()

# UI commands for heater control (requested, not confirmed)
HEATER_COMMANDS = {"2": {"override": False, "setpoint": None}}

# MQTT command publishing configuration
MQTT_BROKER = "broker.mqttdashboard.com"
MQTT_PORT = 1883
MQTT_COMMANDS_TOPIC = "group14/commands/arduino"
HEATER_PIN = 12

# Track last command we sent to avoid spamming MQTT
LAST_HEATER_CMD_SENT = None


def utc_now_str() -> str:
    return datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")


def publish_heater_command(cmd: int) -> None:
    """Publish {"pin":13,"cmd":0/1} to MQTT."""
    global LAST_HEATER_CMD_SENT

    if cmd not in (0, 1):
        return

    # Only publish when the desired cmd changes
    if LAST_HEATER_CMD_SENT == cmd:
        return

    payload = json.dumps({"pin": HEATER_PIN, "cmd": cmd})

    try:
        publish.single(
            topic=MQTT_COMMANDS_TOPIC,
            payload=payload,
            hostname=MQTT_BROKER,
            port=MQTT_PORT,
        )
        LAST_HEATER_CMD_SENT = cmd
        print(f"[MQTT] Published heater command: {payload}")
    except Exception as e:
        print(f"[MQTT] Publish failed: {e}")


def apply_heater_control(current_temp: float | None) -> None:
    """
    Decide what we want the heater to do based on UI settings,
    then publish to MQTT if needed.
    """
    with LOCK:
        cfg = HEATER_COMMANDS.get("2", {"override": False, "setpoint": None})
        override = bool(cfg.get("override", False))
        setpoint = cfg.get("setpoint", None)

    # Override forces ON
    if override:
        publish_heater_command(1)
        return

    # "Off" forces OFF
    if setpoint is None:
        publish_heater_command(0)
        return

    # If no temperature, do nothing
    if current_temp is None:
        return

    # thermostat logic: ON when temp < setpoint
    desired = 1 if current_temp < float(setpoint) else 0
    publish_heater_command(desired)


def add_measurement(room_id: str, temperature: float, humidity: float, heater_on: int | None = None) -> None:
    """Store a measurement point for a room in memory, keeping only the latest MAX_POINTS."""
    if room_id not in ROOMS:
        return

    heater_val = None
    if heater_on is not None:
        try:
            heater_val = int(heater_on)
        except (TypeError, ValueError):
            heater_val = None
        if heater_val not in (0, 1):
            heater_val = None

    point = {
        "timestamp": utc_now_str(),
        "temperature": float(temperature),
        "humidity": float(humidity),
        "heater_on": heater_val,  # confirmed status (from ACK via forwarder), 0/1/None
    }

    with LOCK:
        DATA[room_id].append(point)
        if len(DATA[room_id]) > MAX_POINTS:
            DATA[room_id] = DATA[room_id][-MAX_POINTS:]


@app.route("/")
def dashboard():
    return render_template("dashboard.html", rooms=ROOMS)


@app.post("/api/rooms/<room_id>/measurements")
def ingest_measurement(room_id):
    """
    Expected JSON:
      {
        "temperature": 22.5,
        "humidity": 45.1,
        "heater_on": 0 or 1 or None   # optional confirmation (room 2)
      }
    """
    if room_id not in ROOMS:
        abort(404)

    payload = request.get_json(silent=True) or {}

    if "temperature" not in payload or "humidity" not in payload:
        return jsonify({"message": "missing temperature/humidity"}), 400

    try:
        temperature = float(payload["temperature"])
        humidity = float(payload["humidity"])
    except (TypeError, ValueError):
        return jsonify({"message": "temperature/humidity must be numbers"}), 400

    heater_on = payload.get("heater_on")  # expect 0, 1 or None
    add_measurement(room_id, temperature, humidity, heater_on)

    # Apply heater control only for Bedroom (room 2)
    if room_id == "2":
        apply_heater_control(current_temp=temperature)

    return jsonify({"message": "ok"})


@app.post("/api/rooms/<room_id>/heater/command")
def set_heater_command(room_id):
    """
    UI -> backend "request"
    Payload:
      { "override": true/false, "setpoint": "off" or 15..22 }
    """
    if room_id != "2":
        abort(404)  # Heater only in room 2

    payload = request.get_json(silent=True) or {}

    override = bool(payload.get("override") or False)

    sp_raw = payload.get("setpoint", "off")
    try:
        setpoint = None if sp_raw == "off" else float(sp_raw)
    except (TypeError, ValueError):
        return jsonify({"message": "bad setpoint"}), 400

    with LOCK:
        HEATER_COMMANDS["2"] = {"override": override, "setpoint": setpoint}

    latest_temp = None
    with LOCK:
        if DATA["2"]:
            latest_temp = DATA["2"][-1]["temperature"]

    apply_heater_control(current_temp=latest_temp)

    return ("", 204)


@app.get("/api/rooms/latest")
def latest_all_rooms():
    out = []
    with LOCK:
        for room_id, name in ROOMS.items():
            latest_point = DATA[room_id][-1] if len(DATA[room_id]) else None
            out.append({
                "room_id": room_id,
                "name": name,
                "timestamp": latest_point["timestamp"] if latest_point else None,
                "temperature": latest_point["temperature"] if latest_point else None,
                "humidity": latest_point["humidity"] if latest_point else None,
                "heater_on": latest_point.get("heater_on") if latest_point else None,
            })
    return jsonify(out)


@app.get("/api/rooms/<room_id>/history")
def history(room_id):
    if room_id not in ROOMS:
        abort(404)

    limit = request.args.get("limit", default="200")
    try:
        limit_i = max(1, min(int(limit), MAX_POINTS))
    except ValueError:
        limit_i = 200

    with LOCK:
        points = DATA[room_id][-limit_i:]

    return jsonify(points)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8000, debug=True)
