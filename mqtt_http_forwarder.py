import json
import requests
import paho.mqtt.client as mqtt

BROKER = "broker.mqttdashboard.com"
PORT = 1883

# MQTT topics
TOPIC_ARDUINO_TEMP = "group14/arduino/temperature"
TOPIC_ARDUINO_HUM  = "group14/arduino/humidity"
TOPIC_RUUVI        = "group14/ruuvitag/measurements"
TOPIC_ACK          = "group14/commands/arduino/ack"

# Flask endpoint
API_URL = "http://127.0.0.1:8000/api/rooms/{}/measurements"

# Rooms
ROOM_LIVING = "1"
ROOM_BEDROOM = "2"

# Heater pin on arduino
HEATER_PIN = 12

# Keep latest values
latest_bed_temp = None
latest_bed_hum = None
latest_bed_heater_on = None  # 0/1 confirmed via ACK

session = requests.Session()


def post_measurement(room_id: str, temperature, humidity, heater_on=None):
    """Send combined measurement to Flask. heater_on is optional (0/1)."""
    out = {
        "temperature": temperature,
        "humidity": humidity,
    }
    if heater_on is not None:
        out["heater_on"] = heater_on

    try:
        session.post(API_URL.format(room_id), json=out, timeout=2)
    except requests.RequestException:
        pass


def try_post_bedroom():
    """Post bedroom measurement only when we have both temp and humidity."""
    if latest_bed_temp is None or latest_bed_hum is None:
        return
    post_measurement(ROOM_BEDROOM, latest_bed_temp, latest_bed_hum, latest_bed_heater_on)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe([
            (TOPIC_ARDUINO_TEMP, 0),
            (TOPIC_ARDUINO_HUM, 0),
            (TOPIC_RUUVI, 0),
            (TOPIC_ACK, 0),
        ])
    else:
        print("MQTT connect failed rc=", rc)


def on_message(client, userdata, msg):
    global latest_bed_temp, latest_bed_hum, latest_bed_heater_on

    try:
        payload = json.loads(msg.payload.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return

    topic = msg.topic

    # Living room from RuuviTag
    if topic == TOPIC_RUUVI:
        t = payload.get("temperature")
        h = payload.get("humidity")
        if t is None or h is None:
            return
        post_measurement(ROOM_LIVING, t, h)
        return

    # Bedroom temperature from Arduino
    if topic == TOPIC_ARDUINO_TEMP:
        t = payload.get("temperature")
        if t is None:
            return
        latest_bed_temp = t
        try_post_bedroom()
        return

    # Bedroom humidity from Arduino (friend code publishes "humibity" typo)
    if topic == TOPIC_ARDUINO_HUM:
        h = payload.get("humidity")
        if h is None:
            h = payload.get("humibity")
        if h is None:
            return
        latest_bed_hum = h
        try_post_bedroom()
        return

    # ACK used as "confirmation" for heater state
    if topic == TOPIC_ACK:
        status = payload.get("status")
        pin = payload.get("pin")
        cmd = payload.get("cmd")

        if status != "ok":
            return

        try:
            pin_i = int(pin)
            cmd_i = int(cmd)
        except (TypeError, ValueError):
            return

        if pin_i != HEATER_PIN:
            return

        if cmd_i not in (0, 1):
            return

        latest_bed_heater_on = cmd_i
        try_post_bedroom()
        return


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER, PORT, keepalive=60)
    client.loop_forever()


if __name__ == "__main__":
    main()
