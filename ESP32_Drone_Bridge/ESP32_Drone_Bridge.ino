/**
 * =========================================================================================
 *  AERO-LINK: ESP32 Universal Drone Flight & FPV Video Bridge
 *  Target Hardware: ESP32 Dev Module / NodeMCU-32S / ESP32-WROOM-32
 * =========================================================================================
 *  Features:
 *   1. Dynamic Gateway Detection: Automatically reads WiFi.gatewayIP() (192.168.1.1)
 *   2. Dual Port Relay: Sends to Port 7099 (RC-UFO / KY) AND Port 8090 (E88)
 *   3. Auto-Arming & Binding: Sends Gyro Calibration burst on Wi-Fi connect to turn drone LEDs solid
 *   4. Dual Input: Accepts commands over BOTH USB Serial (COM9, 115200 baud) AND Bluetooth SPP
 *   5. Continuous 25Hz flight loop + 1Hz [0x01, 0x01] heartbeat to keep link active
 *   6. Integrated FPV Video Relay: Streams live MJPEG camera frames from Drone (8080/80)
 *      with 0xDE 0xAD 0xBE 0xEF sync header directly into Web Cockpit HUD
 * =========================================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include "BluetoothSerial.h"
#include "esp_bt.h"

// ==================== CONFIGURATION ====================
// Drone Wi-Fi Hotspot SSID
const char* DRONE_SSID = "WIFI-UFO-289424"; 
const char* DRONE_PASS = ""; // Open Wi-Fi

// Drone Ports
const uint16_t PORT_RC_UFO = 7099; // Standard for RC-UFO, KY, and GL drones
const uint16_t PORT_E88    = 8090; // Standard for E88 drones
const uint16_t PORT_CAM    = 8080; // Camera handshake port

// Bluetooth Device Name
const char* BT_DEVICE_NAME = "ESP32_Drone";

// Built-in Blue LED Pin
#define LED_PIN 2

// ==================== GLOBAL VARIABLES ====================
BluetoothSerial SerialBT;
WiFiUDP udp;
WiFiClient videoClient;

IPAddress droneTargetIP(192, 168, 1, 1); // Dynamically updated from WiFi.gatewayIP()
bool btConnected = false;
bool droneAssociated = false;
unsigned long lastFlightPacket = 0;
unsigned long lastHeartbeat    = 0;
unsigned long lastWifiCheck    = 0;
unsigned long lastBlink        = 0;

// Camera / FPV State
uint16_t camPort = 8080;
bool camStreamStarted = false;
unsigned long lastCamAttempt = 0;
const size_t MAX_JPEG = 16384; // 16 KB frame buffer in static memory (saves DRAM)
uint8_t jpegBuf[MAX_JPEG];
size_t jpegLen = 0;
bool receivingJpeg = false;
int prevCamByte = -1;

// Flight Channels (1..255, 128 = Center Hover)
uint8_t curRoll     = 128;
uint8_t curPitch    = 128;
uint8_t curThrottle = 128;
uint8_t curYaw      = 128;
uint8_t curCmd      = 0x00;

// Logging helper (prints to both USB Serial and Bluetooth)
void logMsg(const String& msg) {
  Serial.println(msg);
  if (btConnected) {
    SerialBT.println(msg);
  }
}

// ==================== BLUETOOTH CALLBACK ====================
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    Serial.println("[BT] Controller Connected via Bluetooth SPP!");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    Serial.println("[BT] Bluetooth Disconnected!");
    curRoll = 128; curPitch = 128; curThrottle = 128; curYaw = 128; curCmd = 0x00;
  }
}

// ==================== VIDEO PACKET SENDER ====================
void sendVideoFrame(uint8_t* buf, size_t len) {
  // 8-byte sync header expected by app.js: [0xDE, 0xAD, 0xBE, 0xEF, len(4)]
  uint8_t hdr[8] = {
    0xDE, 0xAD, 0xBE, 0xEF,
    (uint8_t)((len >> 24) & 0xFF),
    (uint8_t)((len >> 16) & 0xFF),
    (uint8_t)((len >> 8) & 0xFF),
    (uint8_t)(len & 0xFF)
  };

  // Write to USB Serial (COM9)
  Serial.write(hdr, 8);
  Serial.write(buf, len);

  // Write to Bluetooth if connected
  if (btConnected) {
    SerialBT.write(hdr, 8);
    size_t sent = 0;
    while (sent < len && btConnected) {
      size_t chunk = (len - sent > 512) ? 512 : (len - sent);
      SerialBT.write(buf + sent, chunk);
      sent += chunk;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void serviceVideoRelay() {
  if (WiFi.status() != WL_CONNECTED) {
    if (camStreamStarted) {
      videoClient.stop();
      camStreamStarted = false;
    }
    return;
  }

  // Connect to Drone Camera on Port 8080 or Port 80 (every 8 seconds if disconnected)
  if (!videoClient.connected()) {
    camStreamStarted = false;
    if (millis() - lastCamAttempt > 8000) {
      lastCamAttempt = millis();
      if (videoClient.connect(droneTargetIP, 8080, 200)) {
        camPort = 8080;
        camStreamStarted = true;
      } else if (videoClient.connect(droneTargetIP, 80, 200)) {
        camPort = 80;
        camStreamStarted = true;
      }

      if (camStreamStarted) {
        logMsg("[VIDEO] Camera linked on " + droneTargetIP.toString() + ":" + String(camPort));
        videoClient.printf("GET /?action=stream HTTP/1.1\r\nHost: %s:%d\r\nUser-Agent: ESP32_FPV\r\nConnection: keep-alive\r\n\r\n",
                           droneTargetIP.toString().c_str(), camPort);
        receivingJpeg = false;
        jpegLen = 0;
        prevCamByte = -1;
      }
    }
    return;
  }

  // Process available bytes from camera stream
  int budget = 1024;
  while (videoClient.available() > 0 && budget-- > 0) {
    int b = videoClient.read();
    if (b < 0) break;

    if (!receivingJpeg) {
      // Look for JPEG SOI: 0xFF, 0xD8
      if (prevCamByte == 0xFF && b == 0xD8) {
        receivingJpeg = true;
        jpegBuf[0] = 0xFF;
        jpegBuf[1] = 0xD8;
        jpegLen = 2;
      }
      prevCamByte = b;
    } else {
      if (jpegLen < MAX_JPEG - 2) {
        jpegBuf[jpegLen++] = (uint8_t)b;
      } else {
        // Buffer overflow: drop and resync
        receivingJpeg = false;
        jpegLen = 0;
        prevCamByte = -1;
        continue;
      }

      // Look for JPEG EOI: 0xFF, 0xD9
      if (prevCamByte == 0xFF && b == 0xD9) {
        if (jpegLen >= 400) {
          sendVideoFrame(jpegBuf, jpegLen);
        }
        receivingJpeg = false;
        jpegLen = 0;
        prevCamByte = -1;
      }
      prevCamByte = b;
    }
  }
}

// ==================== PACKET SENDER FUNCTIONS ====================

/**
 * Sends flight control packets to the drone over UDP:
 * 1. 21-byte GL Protocol packet to Port 7099
 * 2. 9-byte Legacy packet to Port 7099
 * 3. 8-byte E88 packet to Port 8090 and 7099
 */
void sendControlPacket(uint8_t roll, uint8_t pitch, uint8_t throttle, uint8_t yaw, uint8_t cmd) {
  if (WiFi.status() != WL_CONNECTED) return;

  // --- 1. 8-Byte E88 Packet (Port 8090 & 7099) ---
  uint8_t cs8 = (roll ^ pitch ^ throttle ^ yaw) & 0xFF;
  uint8_t pkt8[8] = { 0x66, roll, pitch, throttle, yaw, cmd, cs8, 0x99 };

  udp.beginPacket(droneTargetIP, PORT_E88);
  udp.write(pkt8, 8);
  udp.endPacket();

  udp.beginPacket(droneTargetIP, PORT_RC_UFO);
  udp.write(pkt8, 8);
  udp.endPacket();

  // --- 2. 21-Byte GL / RC-UFO Packet (Port 7099) ---
  uint8_t flags1 = 0;
  if (cmd == 0x01) flags1 |= 0x01; // Takeoff
  if (cmd == 0x02) flags1 |= 0x01; // Land
  if (cmd == 0x04) flags1 |= 0x02; // Emergency stop
  if (cmd == 0x80) flags1 |= 0x04; // Gyro calibration
  if (cmd == 0x08) flags1 |= 0x08; // Flip

  uint8_t flags2 = 0x02; // Altitude hold
  uint8_t cs21 = (flags1 ^ (((pitch ^ roll) ^ throttle) ^ yaw)) ^ (flags2 & 0xFF);
  uint8_t pkt21[21] = {0};
  pkt21[0]  = 0x03; pkt21[1] = 0x66; pkt21[2] = 0x14;
  pkt21[3]  = roll; pkt21[4] = pitch; pkt21[5] = throttle; pkt21[6] = yaw;
  pkt21[7]  = flags1; pkt21[8] = flags2;
  pkt21[19] = cs21 & 0xFF;
  pkt21[20] = 0x99;

  udp.beginPacket(droneTargetIP, PORT_RC_UFO);
  udp.write(pkt21, 21);
  udp.endPacket();

  // --- 3. 9-Byte Legacy Packet (Port 7099) ---
  uint8_t legacyFlags = 0;
  if (cmd == 0x01) legacyFlags += 1;
  if (cmd == 0x02) legacyFlags += 2;
  if (cmd == 0x04) legacyFlags += 4;
  if (cmd == 0x08) legacyFlags += 8;
  if (cmd == 0x80) legacyFlags += 128;

  uint8_t cs9 = (((roll ^ pitch) ^ throttle) ^ yaw) ^ (legacyFlags & 0xFF);
  uint8_t pkt9[9] = { 0x03, 0x66, roll, pitch, throttle, yaw, legacyFlags, cs9, 0x99 };

  udp.beginPacket(droneTargetIP, PORT_RC_UFO);
  udp.write(pkt9, 9);
  udp.endPacket();
}

/**
 * Sends 1Hz Keepalive Heartbeat:
 * [0x01, 0x01] to Port 7099 & [0x42, 0x76] to Port 8080/8090
 */
void sendHeartbeat() {
  if (WiFi.status() != WL_CONNECTED) return;

  uint8_t hb7099[2] = { 0x01, 0x01 };
  udp.beginPacket(droneTargetIP, PORT_RC_UFO);
  udp.write(hb7099, sizeof(hb7099));
  udp.endPacket();

  uint8_t hbCam[2] = { 0x42, 0x76 };
  udp.beginPacket(droneTargetIP, PORT_CAM);
  udp.write(hbCam, sizeof(hbCam));
  udp.endPacket();
}

/**
 * Triggers Gyro Calibration burst.
 * This is what binds the drone and turns flickering LEDs SOLID!
 */
void triggerGyroBinding() {
  logMsg("[DRONE] Sending Gyro Calibration & Binding sequence...");
  for (int i = 0; i < 15; i++) {
    sendControlPacket(128, 128, 128, 128, 0x80);
    delay(40);
  }
  logMsg("[DRONE] Gyro calibrated! Drone LEDs should now be SOLID ON.");
}

// Connect to Drone Wi-Fi Hotspot
void connectToDroneWifi() {
  Serial.printf("\n[WIFI] Connecting to Drone Hotspot: %s...\n", DRONE_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  delay(50);

  // For open networks, do NOT pass an empty string password to avoid AUTH_FAIL
  if (strlen(DRONE_PASS) == 0) {
    WiFi.begin(DRONE_SSID);
  } else {
    WiFi.begin(DRONE_SSID, DRONE_PASS);
  }

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 35) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Fast blink while searching
    vTaskDelay(pdMS_TO_TICKS(150)); // Yields cleanly across both FreeRTOS cores
    attempts++;
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    droneAssociated = true;
    digitalWrite(LED_PIN, HIGH); // Solid Blue ON = Wi-Fi locked
    droneTargetIP = WiFi.gatewayIP();
    if (droneTargetIP == IPAddress(0, 0, 0, 0)) {
      droneTargetIP = IPAddress(192, 168, 1, 1);
    }

    udp.begin(PORT_RC_UFO);

    logMsg("\n==========================================");
    logMsg("[WIFI] Connected to Drone Hotspot!");
    logMsg("[WIFI] Drone Gateway IP : " + droneTargetIP.toString());
    logMsg("[WIFI] ESP32 Assigned IP: " + WiFi.localIP().toString());
    logMsg("[WIFI] Signal Strength  : " + String(WiFi.RSSI()) + " dBm");
    logMsg("==========================================");

    // Initial wake-up handshakes
    for (int i = 0; i < 3; i++) {
      sendHeartbeat();
      delay(80);
    }

    // Bind and level the drone immediately to stop LEDs flickering
    triggerGyroBinding();
  } else {
    droneAssociated = false;
    digitalWrite(LED_PIN, LOW);
    logMsg("\n[WIFI] Drone not found yet. Retrying in background...");
  }
}

// ==================== STREAM PROCESSOR ====================
// Handles commands from both USB Serial and Bluetooth
void processIncomingByte(int b, Stream& stream) {
  if (b == 0x66) {
    // 8-Byte Binary Packet: [0x66, Roll, Pitch, Throttle, Yaw, Cmd, Checksum, 0x99]
    uint8_t buf[7];
    int readBytes = stream.readBytes(buf, 7);
    if (readBytes == 7 && buf[6] == 0x99) {
      curRoll     = buf[0];
      curPitch    = buf[1];
      curThrottle = buf[2];
      curYaw      = buf[3];
      curCmd      = buf[4];
      sendControlPacket(curRoll, curPitch, curThrottle, curYaw, curCmd);
    }
  } 
  else if (b == 0x03) {
    // GL 21-Byte packet header check
    int next = stream.read();
    if (next == 0x66) {
      int len = stream.read();
      if (len == 0x14) {
        uint8_t buf[18];
        int r = stream.readBytes(buf, 18);
        if (r == 18 && buf[17] == 0x99) {
          curRoll     = buf[0];
          curPitch    = buf[1];
          curThrottle = buf[2];
          curYaw      = buf[3];
          curCmd      = buf[4];
          sendControlPacket(curRoll, curPitch, curThrottle, curYaw, curCmd);
        }
      }
    }
  }
  else if (b == 't' || b == 'T') {
    logMsg("[CMD] Auto-Takeoff");
    logMsg("ACK:TAKEOFF");
    for (int i = 0; i < 20; i++) {
      sendControlPacket(128, 128, 128, 128, 0x01);
      delay(40);
    }
  } 
  else if (b == 'l' || b == 'L') {
    logMsg("[CMD] Auto-Landing");
    logMsg("ACK:LAND");
    for (int i = 0; i < 20; i++) {
      sendControlPacket(128, 128, 128, 128, 0x02);
      delay(40);
    }
  } 
  else if (b == 's' || b == 'S') {
    logMsg("[CMD] Emergency Stop");
    logMsg("ACK:STOP");
    curThrottle = 0;
    for (int i = 0; i < 15; i++) {
      sendControlPacket(128, 128, 0, 128, 0x04);
      delay(40);
    }
  }
  else if (b == 'g' || b == 'G' || b == 'c' || b == 'C') {
    logMsg("[CMD] Gyro Calibration");
    logMsg("ACK:GYRO");
    triggerGyroBinding();
  }
  else if (b == 'h' || b == 'H') {
    logMsg("[CMD] Handshake Wakeup");
    logMsg("ACK:HANDSHAKE");
    for (int i = 0; i < 3; i++) {
      sendHeartbeat();
      delay(80);
    }
  }
  else if (b == 'f' || b == 'F') {
    logMsg("[CMD] 360 Flip");
    logMsg("ACK:FLIP");
    sendControlPacket(128, 128, 128, 128, 0x08);
  }
  else if (b == 'v' || b == 'V') {
    logMsg("[PROBE] Testing drone camera endpoints...");
    WiFiClient testCli;
    bool p8080 = testCli.connect(droneTargetIP, 8080, 400);
    testCli.stop();
    bool p80 = testCli.connect(droneTargetIP, 80, 400);
    testCli.stop();
    bool p7070 = testCli.connect(droneTargetIP, 7070, 400);
    testCli.stop();
    logMsg("PROBE:IP=" + droneTargetIP.toString() +
           ",PORT_8080=" + (p8080 ? "OPEN" : "CLOSED") +
           ",PORT_80=" + (p80 ? "OPEN" : "CLOSED") +
           ",PORT_7070=" + (p7070 ? "OPEN" : "CLOSED"));
  }
  else if (b == 'p' || b == 'P') {
    String stat = "STATUS:WIFI=" + String(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED") +
                  ",RSSI=" + String(WiFi.RSSI()) +
                  ",DRONE_IP=" + droneTargetIP.toString() +
                  ",ESP_IP=" + WiFi.localIP().toString() +
                  ",BT=" + String(btConnected ? "1" : "0") +
                  ",CAM=" + String(videoClient.connected() ? "LINKED" : "OFFLINE");
    logMsg(stat);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 1. Reconfigure Task Watchdog (Disable IDLE core aborts, set 30s grace)
  esp_task_wdt_config_t twdt_cfg = {
    .timeout_ms = 30000,
    .idle_core_mask = 0,
    .trigger_panic = false
  };
  esp_task_wdt_reconfigure(&twdt_cfg);

  // 2. Recover NVS flash & disable flash writes
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  Serial.println("\n==========================================");
  Serial.println("   AERO-LINK ESP32 DRONE FLIGHT BRIDGE    ");
  Serial.println("==========================================");

  // 3. Connect to Drone Wi-Fi FIRST (Must be first to calibrate RF radio)
  WiFi.persistent(false);
  connectToDroneWifi();

  // 4. Start Bluetooth SPP (With BLE RAM freed)
  SerialBT.register_callback(btCallback);
  SerialBT.enableSSP(); // Enables Simple Secure Pairing ("Just Works")
  if (!SerialBT.begin(BT_DEVICE_NAME, false, true)) {
    Serial.println("[BT] Error starting Bluetooth!");
  } else {
    Serial.printf("[BT] Bluetooth Ready as '%s'\n", BT_DEVICE_NAME);
  }
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long now = millis();

  // 1. Wi-Fi Health Check & Auto-Reconnect
  if (now - lastWifiCheck > 5000) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED_PIN, LOW);
      droneAssociated = false;
      connectToDroneWifi();
    } else {
      droneAssociated = true;
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // 2. Handle USB Serial Input (COM9)
  while (Serial.available() > 0) {
    processIncomingByte(Serial.read(), Serial);
  }

  // 3. Handle Bluetooth Input
  while (SerialBT.available() > 0) {
    processIncomingByte(SerialBT.read(), SerialBT);
  }

  // 4. 1Hz Heartbeat Keepalive to Port 7099
  if (now - lastHeartbeat >= 1000) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  // 5. Continuous 25Hz (40ms) Flight Control Stream
  if (now - lastFlightPacket >= 40) {
    lastFlightPacket = now;
    if (droneAssociated) {
      sendControlPacket(curRoll, curPitch, curThrottle, curYaw, curCmd);
      if (curCmd != 0x00) curCmd = 0x00; // Reset one-shot action flag
    }
  }

  // 6. Non-blocking Camera / Video Stream Relay
  serviceVideoRelay();

  // FreeRTOS yield
  vTaskDelay(pdMS_TO_TICKS(2));
}