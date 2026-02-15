#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>

unsigned long lastUpdate = 0;
const int POLLING_INTERVAL = 10000;

// UUIDs
#define ENV_SERVICE_UUID  "12345678-1234-5678-1234-56789abcdef0"
#define TEMP_CHAR_UUID    "12345678-1234-5678-1234-56789abcdef1"
#define HUM_CHAR_UUID     "12345678-1234-5678-1234-56789abcdef2"
#define CMD_CHAR_UUID     "12345678-1234-5678-1234-56789abcdef3"

BLEService envService(ENV_SERVICE_UUID);

// Temperature and humidity BLE notify characteristics
BLEFloatCharacteristic tempChar(TEMP_CHAR_UUID, BLERead | BLENotify);
BLEFloatCharacteristic humChar(HUM_CHAR_UUID, BLERead | BLENotify);

// BLE write characteristic for commands
BLECharacteristic cmdChar(CMD_CHAR_UUID, BLEWrite | BLEWriteWithoutResponse, 128);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize HTS221 sensor / stop if error
  if (!HTS.begin()) {
    Serial.println("Failed to initialize HTS221 sensor");
    while (1);
  }

  // Initialize BLE / stop if error
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // BLE config
  BLE.setLocalName("nano");
  BLE.setAdvertisedService(envService);
  
  // Env service config
  envService.addCharacteristic(tempChar);
  envService.addCharacteristic(humChar);
  envService.addCharacteristic(cmdChar);
  BLE.addService(envService);

  // Init
  tempChar.writeValue(0.0f);
  humChar.writeValue(0.0f);

  // Start service advertising
  BLE.advertise();
  Serial.println("Service advertising started");
}

void loop() {
  BLEDevice central = BLE.central();

  
  if (central) {
    Serial.print("Connected to gateway");
    Serial.println(central.address());

    while (central.connected()) {

      // Keep alive
      BLE.poll();

      // Read sensor every polling interval
      unsigned long now = millis();
      if (now - lastUpdate >= POLLING_INTERVAL) {
        lastUpdate = now;

        // Read temp and humid, and round to 2 decimals
        float temperature = HTS.readTemperature();
        float humidity = HTS.readHumidity();
        temperature = round(temperature * 100.0) / 100.0;
        humidity    = round(humidity * 100.0) / 100.0;

        // Update BLE characteristics
        tempChar.writeValue(temperature);
        humChar.writeValue(humidity);

        // Print to serial monitor for debug
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" C Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
      }

      // Command handling
      if (cmdChar.written()) {

        int len = cmdChar.valueLength();
        char buf[32];

        // Copy data to puffer
        memcpy(buf, cmdChar.value(), len);
        buf[len] = '\0';

        // Command parsing
        // Command format: "pin,cmd/new state"
        int pin = atoi(strtok(buf, ","));
        int cmd = atoi(strtok(NULL, ","));

        // Print to serial monitor for debug
        Serial.print("Pin: ");
        Serial.print(pin);
        Serial.print(" Cmd: ");
        Serial.println(cmd);

        // Set pin mode and pin state to commanded state
        pinMode(pin, OUTPUT);
        digitalWrite(pin, cmd ? HIGH : LOW);
    }
    }
    Serial.println("Gateway disconnected");
  }
}
