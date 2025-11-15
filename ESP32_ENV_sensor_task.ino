#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define SOIL_PIN 1

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
WebServer server(80);

const char* STA_SSID = "FH_GAST";
const char* STA_PASS = "";
const char* AP_SSID = "ESP32_ENV";
const char* AP_PASS = "12345678";

// BMP280 variables
float temperature = NAN, pressure = NAN, altitude = NAN;
float SEA_LEVEL_PRESSURE = 1013.25;
bool bmpConnected = false;

// MPU6050 variables
float accelX = NAN, accelY = NAN, accelZ = NAN;
float gyroX = NAN, gyroY = NAN, gyroZ = NAN;
float mpuTemp = NAN;
bool mpuConnected = false;

// Soil moisture variables
int soilRaw = 0;
int soilPercent = 0;

// ------------------ BMP280 INIT ------------------
bool initBMP280() {
  // Try address 0x76 first (SDO→GND)
  if (bmp.begin(0x76)) {
    Serial.println("✔ BMP280 detected at 0x76");
  } 
  // Try address 0x77 (SDO→VCC)
  else if (bmp.begin(0x77)) {
    Serial.println("✔ BMP280 detected at 0x77");
  }
  else {
    Serial.println("✘ BMP280 NOT DETECTED at 0x76 or 0x77!");
    Serial.println("   Check: VCC, GND, SDA, SCL, CSB→3.3V, SDO→GND");
    return false;
  }
  
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  return true;
}

// ------------------ MPU6050 INIT ------------------
bool initMPU6050() {
  if (!mpu.begin(0x68)) {
    Serial.println("✘ MPU6050 NOT DETECTED!");
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("✔ MPU6050 detected");
  return true;
}

// ------------------ READ BMP280 ------------------
void readBMP280() {
  if (!bmpConnected) {
    bmpConnected = initBMP280();
    if (!bmpConnected) {
      temperature = pressure = altitude = NAN;
      return;
    }
  }

  if (!bmp.takeForcedMeasurement()) {
    Serial.println("✘ BMP280 read failed!");
    bmpConnected = false;
    temperature = pressure = altitude = NAN;
    return;
  }

  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;
  altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
  
  // Validate readings
  if (isnan(temperature) || isnan(pressure) || 
      temperature < -40 || temperature > 85 || 
      pressure < 300 || pressure > 1200) {
    Serial.println("✘ BMP280 readings invalid!");
    bmpConnected = false;
    temperature = pressure = altitude = NAN;
    return;
  }
}

// ------------------ READ MPU6050 ------------------
void readMPU6050() {
  if (!mpuConnected) {
    mpuConnected = initMPU6050();
    if (!mpuConnected) {
      accelX = accelY = accelZ = NAN;
      gyroX = gyroY = gyroZ = NAN;
      mpuTemp = NAN;
      return;
    }
  }

  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println("✘ MPU6050 read failed!");
    mpuConnected = false;
    accelX = accelY = accelZ = NAN;
    gyroX = gyroY = gyroZ = NAN;
    mpuTemp = NAN;
    return;
  }

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  mpuTemp = temp.temperature;
  
  // Validate readings - if they're nonsensical, sensor has failed
  if (isnan(accelX) || isnan(accelY) || isnan(accelZ) ||
      isnan(gyroX) || isnan(gyroY) || isnan(gyroZ) ||
      abs(accelX) > 50 || abs(accelY) > 50 || abs(accelZ) > 50 ||
      abs(gyroX) > 10 || abs(gyroY) > 10 || abs(gyroZ) > 10) {
    Serial.println("✘ MPU6050 readings invalid! Sensor disconnected.");
    mpuConnected = false;
    accelX = accelY = accelZ = NAN;
    gyroX = gyroY = gyroZ = NAN;
    mpuTemp = NAN;
    return;
  }
}

// ------------------ READ SOIL MOISTURE ------------------
void readSoilMoisture() {
  soilRaw = analogRead(SOIL_PIN);
  
  // Validate reading - ESP32-S2 ADC should be 0-8191
  if (soilRaw < 0 || soilRaw > 8191) {
    Serial.println("✘ Soil sensor reading invalid!");
    soilRaw = 8191; // Default to dry
    soilPercent = 0;
    return;
  }
  
  // Map to percentage - CALIBRATED VALUES
  // Dry (air): 8191, Wet (water): ~1500
  soilPercent = map(soilRaw, 8191, 1500, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);
}

// ------------------ WARNING SYSTEM ------------------
String getWarning() {
  String warnings = "";
  
  if (!bmpConnected) warnings += "BMP280 DISCONNECTED | ";
  else {
    if (temperature > 45) warnings += "HIGH TEMP! | ";
    if (temperature < -10) warnings += "LOW TEMP! | ";
    if (pressure < 950) warnings += "LOW PRESSURE! | ";
    if (pressure > 1050) warnings += "HIGH PRESSURE! | ";
  }
  
  if (!mpuConnected) warnings += "MPU6050 DISCONNECTED | ";
  
  if (soilPercent < 20) warnings += "SOIL TOO DRY! | ";
  if (soilPercent > 80) warnings += "SOIL TOO WET! | ";
  
  if (warnings == "") return "NORMAL";
  return warnings;
}

// ------------------ HTML DASHBOARD ------------------
String htmlPage() {
  String warning = getWarning();
  String color = "green";
  if (warning.indexOf("!") >= 0) color = "red";
  else if (warning.indexOf("DISCONNECTED") >= 0) color = "orange";

  String page = "<html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='3'>";
  page += "<style>body{font-family:Arial;margin:20px;} ";
  page += ".sensor{border:1px solid #ccc;padding:10px;margin:10px 0;border-radius:5px;} ";
  page += ".warn{font-size:18px;color:" + color + ";font-weight:bold;padding:10px;background:#f0f0f0;}</style>";
  page += "</head><body>";

  page += "<h1>ESP32 Multi-Sensor Dashboard</h1>";
  
  // BMP280
  page += "<div class='sensor'><h3>BMP280 - Environment</h3>";
  if (bmpConnected) {
    page += "<p><b>Temperature:</b> " + String(temperature, 2) + " &deg;C</p>";
    page += "<p><b>Pressure:</b> " + String(pressure, 2) + " hPa</p>";
    page += "<p><b>Altitude:</b> " + String(altitude, 2) + " m</p>";
  } else {
    page += "<p style='color:red;'>SENSOR NOT CONNECTED</p>";
  }
  page += "</div>";
  
  // MPU6050
  page += "<div class='sensor'><h3>MPU6050 - Motion</h3>";
  if (mpuConnected) {
    page += "<p><b>Accel X/Y/Z:</b> " + String(accelX, 2) + " / " + String(accelY, 2) + " / " + String(accelZ, 2) + " m/s&sup2;</p>";
    page += "<p><b>Gyro X/Y/Z:</b> " + String(gyroX, 2) + " / " + String(gyroY, 2) + " / " + String(gyroZ, 2) + " rad/s</p>";
    page += "<p><b>MPU Temp:</b> " + String(mpuTemp, 2) + " &deg;C</p>";
  } else {
    page += "<p style='color:red;'>SENSOR NOT CONNECTED</p>";
  }
  page += "</div>";
  
  // Soil Moisture
  page += "<div class='sensor'><h3>HD-38 - Soil Moisture</h3>";
  page += "<p><b>Moisture:</b> " + String(soilPercent) + "% (Raw: " + String(soilRaw) + ")</p>";
  page += "</div>";
  
  page += "<div class='warn'>STATUS: " + warning + "</div>";
  page += "</body></html>";
  return page;
}

// ------------------ JSON DATA ------------------
String jsonData() {
  String json = "{";
  json += "\"bmp280\":{";
  json += "\"connected\":" + String(bmpConnected ? "true" : "false") + ",";
  json += "\"temperature\":" + String(temperature, 2) + ",";
  json += "\"pressure\":" + String(pressure, 2) + ",";
  json += "\"altitude\":" + String(altitude, 2);
  json += "},";
  
  json += "\"mpu6050\":{";
  json += "\"connected\":" + String(mpuConnected ? "true" : "false") + ",";
  json += "\"accel\":{\"x\":" + String(accelX, 2) + ",\"y\":" + String(accelY, 2) + ",\"z\":" + String(accelZ, 2) + "},";
  json += "\"gyro\":{\"x\":" + String(gyroX, 2) + ",\"y\":" + String(gyroY, 2) + ",\"z\":" + String(gyroZ, 2) + "},";
  json += "\"temperature\":" + String(mpuTemp, 2);
  json += "},";
  
  json += "\"soil\":{";
  json += "\"raw\":" + String(soilRaw) + ",";
  json += "\"percent\":" + String(soilPercent);
  json += "},";
  
  json += "\"warning\":\"" + getWarning() + "\"";
  json += "}";
  return json;
}

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Starting...");

  // I2C setup
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Analog pin setup
  pinMode(SOIL_PIN, INPUT);

  // Initialize sensors
  bmpConnected = initBMP280();
  mpuConnected = initMPU6050();

  // WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(STA_SSID, STA_PASS);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✔ WiFi Connected");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✘ WiFi Connection failed");
  }

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() { server.send(200, "text/html", htmlPage()); });
  server.on("/json", []() { server.send(200, "application/json", jsonData()); });
  server.begin();
  
  Serial.println("System ready!");
}

// ------------------ LOOP ------------------
void loop() {
  readBMP280();
  readMPU6050();
  readSoilMoisture();
  server.handleClient();

  Serial.println("========== SENSOR READINGS ==========");
  Serial.print("BMP280 - Temp: ");
  Serial.print(temperature);
  Serial.print("°C | Pressure: ");
  Serial.print(pressure);
  Serial.print("hPa | Altitude: ");
  Serial.print(altitude);
  Serial.println("m");
  
  Serial.print("MPU6050 - Accel: ");
  Serial.print(accelX);
  Serial.print(",");
  Serial.print(accelY);
  Serial.print(",");
  Serial.print(accelZ);
  Serial.print(" | Gyro: ");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.println(gyroZ);
  
  Serial.print("Soil - ");
  Serial.print(soilPercent);
  Serial.print("% (Raw: ");
  Serial.print(soilRaw);
  Serial.println(")");
  
  Serial.print("Status: ");
  Serial.println(getWarning());
  Serial.println("=====================================\n");

  delay(2000);
}