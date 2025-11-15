ESP32 Multi-Sensor Environmental Monitor

 Hardware
- ESP32-S2 WROVER Dev Module
- BMP280 (Temperature, Pressure, Altitude)
- MPU6050 (Accelerometer, Gyroscope)
- HD-38 (Soil Moisture)

Wiring

ESP32    →  BMP280
3.3V     →  VCC
GND      →  GND
GPIO 8   →  SDA
GPIO 9   →  SCL
3.3V     →  CSB
GND      →  SDO
```

Features
- Real-time sensor monitoring
- Web dashboard (HTML)
- JSON API endpoint
- Auto-reconnect on sensor failure
- Independent sensor operation
- WiFi Station + AP mode

Installation
1. Install Arduino IDE
2. Install libraries:
   - Adafruit BMP280
   - Adafruit MPU6050
   - Adafruit Unified Sensor
3. Upload code to ESP32
4. Connect to WiFi or ESP32_ENV network
5. Open browser to displayed IP address

Usage
- Serial Monitor: 115200 baud
- Web Dashboard: http://[IP_ADDRESS]/
- JSON API: http://[IP_ADDRESS]/json

Error Handling
- Sensor disconnection detection
- Data validation
- Automatic reconnection
- Independent sensor operation
