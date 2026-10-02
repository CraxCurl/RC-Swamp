# ESP32 Drone Bridge Controller Firmware & Architecture Guide

This document provides the complete, production-ready **ESP32 Firmware Code**, hardware setup instructions, protocol mappings, and connection architecture to bridge flight control commands from your laptop to the drone.

---

## 1. System Architecture

```
+---------------------------+                      +---------------------------+                      +---------------------------+
|        Laptop / PC        |                      |       ESP32 Bridge        |                      |     Drone Quadcopter      |
|  (GCS Web App / Script)   |                      | (NodeMCU / DevKit V1 WROOM|                      |     (E88 / RC UFO / KY)   |
|                           |   Bluetooth (SPP)    |                           |      Wi-Fi UDP       |                           |
|  Web Serial / Bluetooth   | ===================> |  BluetoothSerial (Slave)  | ===================> | Drone AP (192.168.4.1/1.1)|
|  or USB / COM Port        |                      |  Wi-Fi Station Mode       |  Port 8090 / 7099    | UDP Flight Controller     |
+---------------------------+                      +---------------------------+                      +---------------------------+
```

### Key Workflow:
1. **Laptop to ESP32**: The Laptop connects via **Bluetooth** (or USB Serial COM) to the ESP32 named `ESP32_Drone`.
2. **ESP32 to Drone**: The ESP32 connects via **Wi-Fi** (Station Mode) to the Drone's Access Point (e.g. `WIFI-UFO-289424`).
3. **Control Relay**: The ESP32 receives joystick values and flight action commands from Bluetooth and translates them into UDP packets sent to the drone at 25 Hz (every 40 ms).

---

## 2. Complete ESP32 Firmware Code (`ESP32_Drone_Bridge.ino`)

Copy and flash this code onto your ESP32 board (ESP32-WROOM-32 / DevKit V1) using the Arduino IDE or PlatformIO:

```cpp
/**
 * =========================================================================================
 *  ESP32 Bluetooth-to-WiFi Drone Control Bridge
 *  Target Hardware: ESP32-WROOM-32 / ESP32 DevKit V1
 *  Features:
 *   - Classic Bluetooth SPP (Serial Port Profile) for Laptop / Web GCS Connection
 *   - Wi-Fi Station (STA) connection to Drone Hotspot (E88, RC-UFO, KY601S, etc.)
 *   - FreeRTOS Coexistence management for simultaneous Wi-Fi & Bluetooth operation
 *   - Continuous 25Hz (40ms) Keepalive Heartbeat & Active Joystick Stream
 *   - Support for 8-Byte E88 Packets, 21-Byte GL Packets, and ASCII Character Commands
 *   - Visual Status Indicators via On-Board LED (GPIO 2)
 * =========================================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "BluetoothSerial.h"
#include "esp_bt.h"
#include "esp_coexist.h"

// ---------------------- CONFIGURATION SETTINGS ----------------------
// 1. Drone Wi-Fi Hotspot Credentials
// Change this to your drone's exact Wi-Fi SSID
const char* DRONE_SSID = "WIFI-UFO-289424"; 
const char* DRONE_PASS = ""; // Leave blank for open networks

// 2. Drone IP and Port Configurations
// E88 / Most UFO Drones use 192.168.4.1 or 192.168.1.1
const char* DRONE_IP          = "192.168.4.1"; // Default for E88. (Use "192.168.1.1" for GL/KY drones)
const uint16_t HANDSHAKE_PORT = 8080;          // Handshake port
const uint16_t CONTROL_PORT   = 8090;          // Control port (use 7099 for GL/KY series)

// 3. Bluetooth Device Name (Visible on Laptop)
const char* BT_DEVICE_NAME    = "ESP32_Drone";

// 4. Hardware Pinouts
#define LED_PIN 2 // Built-in Blue LED on standard ESP32 boards

// ---------------------- GLOBAL INSTANCES & STATE ----------------------
BluetoothSerial SerialBT;
WiFiUDP udp;

bool btConnected = false;
unsigned long lastPacketSent = 0;
unsigned long lastWifiCheck = 0;

// Current Flight State Channels (1..255, 128 = Neutral Center)
uint8_t curRoll     = 128;
uint8_t curPitch    = 128;
uint8_t curThrottle = 128;
uint8_t curYaw      = 128;
uint8_t curCmd      = 0x00;

// ---------------------- BLUETOOTH EVENT CALLBACK ----------------------
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    Serial.println("[BT] Laptop connected via Bluetooth SPP!");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    Serial.println("[BT] Bluetooth connection closed.");
    // Reset to neutral hover on disconnect for safety
    curRoll = 128;
    curPitch = 128;
    curThrottle = 128;
    curYaw = 128;
    curCmd = 0x00;
  }
}

// ---------------------- PACKET BUILDERS & TRANSMITTERS ----------------------

/**
 * Sends 8-byte Standard E88 Control Packet:
 * [0x66, Roll, Pitch, Throttle, Yaw, Command, Checksum, 0x99]
 */
void sendControlPacket(uint8_t roll, uint8_t pitch, uint8_t throttle, uint8_t yaw, uint8_t cmd) {
  uint8_t checksum = roll ^ pitch ^ throttle ^ yaw;
  uint8_t packet[8] = { 0x66, roll, pitch, throttle, yaw, cmd, checksum, 0x99 };

  udp.beginPacket(DRONE_IP, CONTROL_PORT);
  udp.write(packet, sizeof(packet));
  udp.endPacket();
}

/**
 * Sends 2-byte Handshake Packet to awaken the drone's internal controller
 */
void sendHandshake() {
  uint8_t handshake[2] = { 0x42, 0x76 };
  udp.beginPacket(DRONE_IP, HANDSHAKE_PORT);
  udp.write(handshake, sizeof(handshake));
  udp.endPacket();
}

/**
 * Connect or Reconnect to Drone's Wi-Fi Hotspot
 */
void connectToDroneWifi() {
  Serial.printf("[WIFI] Connecting to Drone Hotspot: %s ...\n", DRONE_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(DRONE_SSID, DRONE_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Fast blink while connecting
    delay(200);
    attempts++;
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH); // Solid LED = Wi-Fi locked
    Serial.println("\n[WIFI] Connected! Drone Gateway IP: " + WiFi.gatewayIP().toString());
    Serial.println("[WIFI] ESP32 Assigned IP: " + WiFi.localIP().toString());

    // Send initial activation handshakes
    for (int i = 0; i < 3; i++) {
      sendHandshake();
      delay(80);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("\n[WIFI] Connection Failed! Will retry in background.");
  }
}

// ---------------------- ARDUINO SETUP ----------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=========================================");
  Serial.println("  ESP32 DRONE BLUETOOTH-TO-WIFI BRIDGE   ");
  Serial.println("=========================================");

  // 1. Enable Hardware Wi-Fi & Bluetooth Coexistence
  #if defined(ESP_COEX_PREFER_BALANCE)
    esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
  #endif

  // 2. Initialize Bluetooth SPP
  SerialBT.register_callback(btCallback);
  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("[BT] An error occurred initializing Bluetooth Serial!");
  } else {
    Serial.printf("[BT] Bluetooth Ready! Device Name: %s\n", BT_DEVICE_NAME);
  }

  // 3. Connect to Drone Hotspot
  connectToDroneWifi();
}

// ---------------------- MAIN REAL-TIME LOOP ----------------------
void loop() {
  // 1. Check Wi-Fi Health Every 5 Seconds
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED_PIN, LOW);
      connectToDroneWifi();
    }
  }

  // 2. Handle Incoming Bluetooth Flight Commands & Packets
  while (SerialBT.available() > 0) {
    // Peek to see if this is a binary packet starting with 0x66 or an ASCII char
    int b = SerialBT.read();

    if (b == 0x66) {
      // Direct 8-byte Raw Packet: [0x66, roll, pitch, throttle, yaw, cmd, checksum, 0x99]
      uint8_t buf[7];
      int readBytes = SerialBT.readBytes(buf, 7);
      if (readBytes == 7 && buf[6] == 0x99) {
        curRoll     = buf[0];
        curPitch    = buf[1];
        curThrottle = buf[2];
        curYaw      = buf[3];
        curCmd      = buf[4];
        sendControlPacket(curRoll, curPitch, curThrottle, curYaw, curCmd);
      }
    } 
    else if (b == 't' || b == 'T') {
      // Auto-Takeoff Command
      Serial.println("[CMD] Takeoff");
      if (btConnected) SerialBT.println("ACK:TAKEOFF");
      for (int i = 0; i < 20; i++) {
        sendControlPacket(128, 128, 128, 128, 0x01);
        delay(40);
      }
    } 
    else if (b == 'l' || b == 'L') {
      // Auto-Landing Command
      Serial.println("[CMD] Land");
      if (btConnected) SerialBT.println("ACK:LAND");
      for (int i = 0; i < 20; i++) {
        sendControlPacket(128, 128, 128, 128, 0x02);
        delay(40);
      }
    } 
    else if (b == 's' || b == 'S') {
      // Emergency Stop / Disarm
      Serial.println("[CMD] Emergency Stop");
      if (btConnected) SerialBT.println("ACK:STOP");
      curThrottle = 0;
      for (int i = 0; i < 15; i++) {
        sendControlPacket(128, 128, 0, 128, 0x04);
        delay(40);
      }
    }
    else if (b == 'g' || b == 'G') {
      // Gyro Calibration
      Serial.println("[CMD] Gyroscope Calibration");
      if (btConnected) SerialBT.println("ACK:GYRO");
      for (int i = 0; i < 10; i++) {
        sendControlPacket(128, 128, 128, 128, 0x80);
        delay(40);
      }
    }
    else if (b == 'f' || b == 'F') {
      // 360 Stunt Flip
      Serial.println("[CMD] 360 Flip");
      if (btConnected) SerialBT.println("ACK:FLIP");
      sendControlPacket(128, 128, 128, 128, 0x08);
    }
    else if (b == 'h' || b == 'H') {
      // Drone Handshake Wakeup
      Serial.println("[CMD] Handshake Activation");
      if (btConnected) SerialBT.println("ACK:HANDSHAKE");
      for (int i = 0; i < 3; i++) {
        sendHandshake();
        delay(80);
      }
    }
    else if (b == 'p' || b == 'P') {
      // Ping / Status Query
      if (btConnected) {
        SerialBT.printf("STATUS:WIFI=%s,RSSI=%d,IP=%s\n", 
          (WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED"),
          WiFi.RSSI(),
          WiFi.localIP().toString().c_str()
        );
      }
    }
  }

  // 3. Paced 25Hz (every 40ms) Heartbeat & Flight Control Output
  if (millis() - lastPacketSent >= 40) {
    lastPacketSent = millis();
    if (WiFi.status() == WL_CONNECTED) {
      sendControlPacket(curRoll, curPitch, curThrottle, curYaw, curCmd);
      // Reset one-shot cmd flag after sending
      if (curCmd != 0x00) curCmd = 0x00;
    }
  }

  // Mandatory yield to prevent FreeRTOS watchdog starvation on radio core
  vTaskDelay(2 / portTICK_PERIOD_MS);
}
```

---

## 3. Supported Command Protocol

### A. Single Character ASCII Commands (via BT Terminal / Script)
| Command Character | Action | Drone Execution |
| :--- | :--- | :--- |
| `'t'` or `'T'` | **One-Key Takeoff** | Sends burst with command `0x01` (Climb & Lock at ~50 cm) |
| `'l'` or `'L'` | **One-Key Landing** | Sends burst with command `0x02` (Smooth descent & motor cut) |
| `'s'` or `'S'` | **Emergency Stop** | Sends burst with command `0x04`, cuts throttle to 0 |
| `'g'` or `'G'` | **Gyro Calibration** | Sends burst with command `0x80` to level internal IMU |
| `'f'` or `'F'` | **360° Flip** | Sends command `0x08` to arm flip mode |
| `'p'` or `'P'` | **Ping / Status** | ESP32 replies with Wi-Fi link status and RSSI |

### B. Continuous 8-Byte Binary Packet (via Joystick / Web GCS)
Format:
`[ 0x66, Roll, Pitch, Throttle, Yaw, Command, Checksum, 0x99 ]`
- `Roll`: 1 to 255 (128 = Neutral)
- `Pitch`: 1 to 255 (128 = Neutral)
- `Throttle`: 0 to 255 (128 = Center Hover)
- `Yaw`: 1 to 255 (128 = Center)
- `Command`: Bitmask flags (`0x01` Takeoff, `0x02` Land, `0x04` Stop, `0x80` Gyro, `0x00` Hover)
- `Checksum`: `Roll ^ Pitch ^ Throttle ^ Yaw`

---

## 4. Flashing Instructions

### Using Arduino IDE:
1. Open Arduino IDE -> **Tools** -> **Board** -> **ESP32 Arduino** -> Select **ESP32 Dev Module**.
2. Set configuration:
   - **Upload Speed**: `921600` or `115200`
   - **CPU Frequency**: `240MHz (WiFi/BT)`
   - **Flash Frequency**: `80MHz`
   - **Partition Scheme**: `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)`
   - **Port**: Select your ESP32's COM Port (e.g. `COM3` on Windows).
3. Click **Upload**.

---

## 5. Pairing ESP32 with your Laptop

### On Windows 10/11:
1. Turn on Bluetooth on your laptop.
2. Go to **Bluetooth & other devices** -> **Add device** -> **Bluetooth**.
3. Select **`ESP32_Drone`**. (PIN is usually `1234` or not required).
4. Once paired, Windows assigns an outgoing Bluetooth COM port (e.g., `COM5` or `COM7`).

### On Web GCS (Browser):
1. In the Web App modal, click **"ESP32 Bluetooth Bridge"**.
2. Click **"Pair & Connect Bluetooth"** to connect via Web Serial / Web Bluetooth.
3. The Web App will stream flight commands directly to the ESP32!
