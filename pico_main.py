import bluetooth
import struct
import time
import network
from umqtt.simple import MQTTClient
import ujson
import math

# WIFI configuration
WIFI_SSID = "XXXXXX"
WIFI_PASSWORD = "XXXXXX"

# MQTT configuration
MQTT_BROKER = "broker.mqttdashboard.com"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "pico2w_gateway"

# MQTT topics
MQTT_ARDUINO_TEMP_TOPIC = b"group14/arduino/temperature"
MQTT_ARDUINO_HUM_TOPIC = b"group14/arduino/humidity"
MQTT_RUUVI_TOPIC = b"group14/ruuvitag/measurements"
MQTT_COMMANDS_TOPIC = b"group14/commands/arduino"
MQTT_ACK_TOPIC = b"group14/commands/arduino/ack"

# Activate BLE
ble = bluetooth.BLE()
ble.active(True)

# UUID's of the Nano
ENV_SERVICE_UUID = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef0")
TEMP_CHAR_UUID  = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef1")
HUM_CHAR_UUID   = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef2")
CMD_CHAR_UUID = bluetooth.UUID("12345678-1234-5678-1234-56789abcdef3")

# BLE IRQ event codes
_IRQ_SCAN_RESULT = 5
_IRQ_PERIPHERAL_CONNECT = 7
_IRQ_GATTC_SERVICE_RESULT = 9
_IRQ_GATTC_SERVICE_DONE = 10
_IRQ_GATTC_CHARACTERISTIC_RESULT = 11
_IRQ_GATTC_CHARACTERISTIC_DONE = 12
_IRQ_GATTC_NOTIFY = 18

# Nano 33 Sense BLE MAC address
NANO_MAC = b'\xA2\x1D\xD6\x26\xB8\x4D'
ADDR_TYPE = 0 # Public address

# BLE handles and flags
conn_handle = None
start_handle = None
end_handle = None
temp_handle = None
hum_handle = None
cmd_handle = None

# BLE state machine
state = "IDLE"
enable_temp_notify = False
enable_hum_notify = False

# Ruuvitag MAC address
RUUVI_MAC = b'\xF2\x66\x76\x2E\x02\x21'
latest_ruuvi_data = None

# Latest mqtt publication time and publishing interval
last_ruuvi_print = 0
PUBLISH_INTERVAL = 10000


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting to WiFi")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
    print("WiFi connected")
    return wlan


def connect_mqtt():
    print("Connecting to MQTT broker")
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
    client.connect()
    print("Connected to MQTT broker")
    return client


def send_mqtt(key, value, topic):
    """
    Sends message to MQTT broker
    If key is provided, wraps value in a dict with that key.
    Otherwise, sends value directly (expects json format).
    """
    
    if key:
        msg_dict = {key: value}
        msg = ujson.dumps(msg_dict)
    else:
        msg = ujson.dumps(value)

    client.publish(topic, msg.encode("utf-8"))
    # print("Published JSON:", msg)
        
        
def decode_ruuvi(payload: bytes) -> dict | None:
    """
    Decodes RuuviTag Data format 3 (RAWv1).
    https://docs.ruuvi.com/communication/bluetooth-advertisements/data-format-3-rawv1
    Returns temperature, humidity, pressure, acceleration, battery info.
    """
    
    # Make sure payload is valid and in DF3
    if len(payload) < 14:
        return None
    if payload[0] != 0x03:
        return None

    humidity = payload[1] * 0.5

    # Temperature decoding
    temp_int = payload[2]
    temp_frac = payload[3] / 100
    if temp_int < 0:
        temperature = -(temp_int + 128 + temp_frac)
    else:
        temperature = temp_int + temp_frac

    # Pressure (hPa)
    pressure = (payload[4] << 8 | payload[5]) + 50000
    pressure /= 100

    # Acceleration
    acc_x = struct.unpack(">h", payload[6:8])[0]
    acc_y = struct.unpack(">h", payload[8:10])[0]
    acc_z = struct.unpack(">h", payload[10:12])[0]
    acceleration = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)

    # Battery (mV)
    battery = (payload[12] << 8 | payload[13])

    return {
        "temperature": temperature,
        "humidity": humidity,
        "pressure": pressure,
        "acceleration": acceleration,
        "acceleration_x": acc_x,
        "acceleration_y": acc_y,
        "acceleration_z": acc_z,
        "battery": battery
    }


# Allowed arduino pins for commands (only digital pins)
ALLOWED_PINS = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]

def mqtt_callback(topic, msg):
    """
    MQTT message callback for sending commands to arduino.
    Checks command validity and forwards to send_command function.
    Responds to MQTT ACK topic if command is not valid.
    
    """
    
    if topic == MQTT_COMMANDS_TOPIC:
        try:
            cmd = ujson.loads(msg)
            print("Received command:", cmd)

            pin = cmd["pin"]
            value = cmd["cmd"]
            
            # Check that pin is allowed
            if pin not in ALLOWED_PINS:
                print("Pin not allowed")
                nack = ujson.dumps({
                    "status": "Pin not allowed"
                })
                client.publish(MQTT_ACK_TOPIC, nack)
                return
            
            # Check that command/value is valid
            if value not in (0,1):
                print("Value not valid")
                nack = ujson.dumps({
                    "status": "Value not valid"
                })
                client.publish(MQTT_ACK_TOPIC, nack)
                return

            send_command(pin, value)

        except Exception as e:
            print("JSON error:", e)


def send_command(pin_num, value):
    """
    
    """
    if not conn_handle or not cmd_handle:
        print("BLE connection error")
        return

    try:
        # Sends format: "pin,cmd"
        data = f"{int(pin_num)},{int(value)}"
        print("Sending command:", data)

        # Send over BLE
        ble.gattc_write(conn_handle, cmd_handle, data.encode(), 1)
        
        # Respond to MQTT ACK topic if command write was succesfull
        ack = ujson.dumps({
            "status": "ok",
            "pin": int(pin_num),
            "cmd": int(value)
        })
        client.publish(MQTT_ACK_TOPIC, ack)


    except Exception as e:
        print("BLE write failed:", e)
        
        # Respond to MQTT ACK topic if command write failed
        nack = ujson.dumps({
            "status": "error",
            "pin": int(pin_num),
            "cmd": int(value),
            "reason": str(e)
        })
        client.publish(MQTT_ACK_TOPIC, nack)




def irq(event, data):
    """
    Handles BLE events:
    - Connection
    - Service discovery
    - Characteristic discovery
    - Notifications (temperature/humidity)
    - RuuviTag advertisement parsing
    """
    global conn_handle, start_handle, end_handle
    global temp_handle, hum_handle, cmd_handle
    global state
    global enable_temp_notify, enable_hum_notify
    global latest_ruuvi_data
    
    # Nano connected
    if event == _IRQ_PERIPHERAL_CONNECT:
        conn_handle, addr_type, addr = data
        print("Connected to Nano")
        state = "SERVICES"
        ble.gattc_discover_services(conn_handle)
    
    # Service discovered
    elif event == _IRQ_GATTC_SERVICE_RESULT:
        ch, start, end, uuid = data
        if uuid == ENV_SERVICE_UUID:
            start_handle = start
            end_handle = end
            print("Found ENV service")

    # Characteristics discovery
    elif event == _IRQ_GATTC_SERVICE_DONE:
        if state == "SERVICES" and start_handle and end_handle:
            print("Discovering characteristics...")
            state = "CHARS"
            ble.gattc_discover_characteristics(conn_handle, start_handle, end_handle)

    # Discovered characteristics
    elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
        ch, def_handle, value_handle, properties, uuid = data
        if uuid == TEMP_CHAR_UUID:
            temp_handle = value_handle
            print("Found Temperature characteristic")
        elif uuid == HUM_CHAR_UUID:
            hum_handle = value_handle
            print("Found Humidity characteristic")
        elif uuid == CMD_CHAR_UUID:
            cmd_handle = value_handle
            print("Found Command characteristic")
    
    # Charasteristic discovery done
    elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
        if state == "CHARS" and temp_handle and hum_handle:
            enable_temp_notify = True
            state = "READY"

    # Received notification (from Nano)
    elif event == _IRQ_GATTC_NOTIFY:
        ch, value_handle, notify_data = data
        value = struct.unpack("<f", notify_data)[0]

        if value_handle == temp_handle:
            send_mqtt("temperature", value, MQTT_ARDUINO_TEMP_TOPIC)
        
        elif value_handle == hum_handle:
            send_mqtt("humidity", value, MQTT_ARDUINO_HUM_TOPIC)
    
    # RuuviTag broadcasting
    elif event == _IRQ_SCAN_RESULT:
        
        # Make sure MAC address matches RuuviTag
        addr_type, addr, adv_type, rssi, adv_data = data
        addr_str = ':'.join('{:02X}'.format(b) for b in addr)
        ruuvi_str = ':'.join('{:02X}'.format(b) for b in RUUVI_MAC)
        if addr_str != ruuvi_str:
            return

        # Parse BLE advertisement
        i = 0
        while i < len(adv_data):
            
            length = adv_data[i]
            if length == 0:
                break
            
            type_ = adv_data[i + 1]
            # Actual payload
            field = adv_data[i + 2:i + 1 + length]
            
            if type_ == 0xFF and len(field) >= 3:
                payload = field[2:] # skip manufacturer ID
                result = decode_ruuvi(payload)
                if result:
                    latest_ruuvi_data = result
                    
            # Move to the next advertisement field
            i += length + 1
            

# Connect to wifi and MQTT broker
connect_wifi()
client = connect_mqtt()

# Subscribe to commands topic
client.set_callback(mqtt_callback)
client.subscribe(MQTT_COMMANDS_TOPIC)

# Reqister BLE irq
ble.irq(irq)

# Connect to Nano
ble.gap_connect(ADDR_TYPE, NANO_MAC)
# Start scanning for RuuviTag broadcasting
ble.gap_scan(0, 30000, 30000)

# MAIN LOOP
while True:
    
    # Check MQTT for new commands
    client.check_msg()
    
    # Enable temperature notifications first
    if enable_temp_notify and conn_handle and temp_handle:
        try:
            ble.gattc_write(conn_handle, temp_handle + 1, b'\x01\x00', 1)
            enable_temp_notify = False
            enable_hum_notify = True  # schedule humidity next
            print("Temperature notifications enabled")
        except OSError as e:
            if e.args[0] != 114:
                raise

    # Enable humidity notifications second
    if enable_hum_notify and conn_handle and hum_handle:
        try:
            ble.gattc_write(conn_handle, hum_handle + 1, b'\x01\x00', 1)
            enable_hum_notify = False
            print("Humidity notifications enabled")
        except OSError as e:
            if e.args[0] != 114:
                raise
    
    # Publish RuuviTag data over MQTT every 10 seconds
    if latest_ruuvi_data:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_ruuvi_print) > PUBLISH_INTERVAL:
            last_ruuvi_print = now
            send_mqtt(None, latest_ruuvi_data, MQTT_RUUVI_TOPIC)

    time.sleep(0.1)
