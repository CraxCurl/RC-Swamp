@echo off
title RC UFO Drone Ground Control Station
echo ============================================================
echo   RC UFO DRONE WEB GROUND CONTROL STATION
echo ============================================================
echo [*] Ensure your PC/Laptop is connected to the Drone Wi-Fi:
echo     SSID: KY-WiFi-xxxx / 4K-WiFi-xxxx / RC-UFO-xxxx
echo     Drone IP: 192.168.1.1:7099 (UDP) / 7070 (RTSP)
echo ============================================================
echo [*] Starting Ground Control Server on port 8080...
echo [*] Web Cockpit will open at: http://localhost:8080
echo ============================================================

start "" http://localhost:8080
node server.js
pause
