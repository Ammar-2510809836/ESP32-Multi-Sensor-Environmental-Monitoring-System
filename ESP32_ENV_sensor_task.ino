/*
 * ESP32 Multi-Sensor Environmental Monitoring System
 * 
 * This program reads data from multiple sensors and displays it via:
 * - Serial Monitor (for debugging)
 * - Web Dashboard (HTML interface)
 * - JSON API (for data integration)
 * 
 * Sensors:
 * - BMP280: Temperature, Pressure, Altitude
 * - MPU6050: Accelerometer, Gyroscope, Temperature
 * - HD-38: Soil Moisture
 * 
 * Features:
 * - Auto-reconnection if sensors disconnect
 * - Data validation to catch sensor errors
 * - Warning system for threshold violations
 * - WiFi Station + Access Point modes
 */

#include <Wire.h>                // I2C communication library
#include <Adafruit_BMP280.h>     // BMP280 sensor library
#include <Adafruit_MPU6050.h>    // MPU6050 sensor library
#include <Adafruit_Sensor.h>     // Unified sensor library
#include <WiFi.h>                // ESP32 WiFi functionality
#include <WebServer.h>           // HTTP web server

// Pin definitions - GPIO pins for I2C and analog input
#define SDA_PIN 8      // I2C data line (shared by BMP280 and MPU6050)
#define SCL_PIN 9      // I2C clock line (shared by BMP280 and MPU6050)
#define SOIL_PIN 1     // Analog input for soil moisture sensor

// Create sensor objects
Adafruit_BMP280 bmp;      // BMP280 environmental sensor
Adafruit_MPU6050 mpu;     // MPU6050 motion sensor
WebServer server(80);     // Web server on port 80 (HTTP)

// WiFi credentials
const char* STA_SSID = "FH_GAST";        // Network name to connect to
const char* STA_PASS = "";               // Network password (empty for open network)
const char* AP_SSID = "ESP32_ENV";       // Backup access point name
const char* AP_PASS = "12345678";        // Backup access point password

// BMP280 sensor variables
float temperature = NAN;           // Temperature in °C (NAN = Not A Number, indicates no data)
float pressure = NAN;              // Atmospheric pressure in hPa
float altitude = NAN;              // Calculated altitude in meters
float SEA_LEVEL_PRESSURE = 1013.25;// Standard sea level pressure for altitude calculation (adjust for your location)
bool bmpConnected = false;         // Track if BMP280 is connected and working

// MPU6050 sensor variables
float accelX = NAN, accelY = NAN, accelZ = NAN;  // Acceleration in m/s² for X, Y, Z axes
float gyroX = NAN, gyroY = NAN, gyroZ = NAN;     // Angular velocity in rad/s for X, Y, Z axes
float mpuTemp = NAN;                              // MPU6050's internal temperature in °C
bool mpuConnected = false;                        // Track if MPU6050 is connected and working

// Soil moisture sensor variables
int soilRaw = 0;         // Raw ADC value (0-8191 on ESP32-S2)
int soilPercent = 0;     // Moisture percentage (0-100%)

// Soil calibration constants (measured experimentally)
const int SOIL_RAW_DRY = 8191;  // Raw ADC value when completely dry (in air/no snow)
const int SOIL_RAW_WET = 1500;  // Raw ADC value when completely wet (in water/wet snow)

// Warning system debounce
unsigned long lastWarningTime = 0;
const unsigned long WARNING_INTERVAL = 3000; // Update warnings every 3 seconds
String cachedWarning = "NORMAL";

// ------------------ HELPER FUNCTIONS ------------------

/*
 * Safely convert float to string for HTML display
 * Returns "N/A" for invalid/NAN values instead of showing "nan"
 */
String floatToStringOrNA(float value, int decimals = 2) {
  if (isnan(value)) {
    return "N/A";
  }
  return String(value, decimals);
}

/*
 * Safely convert float to string for JSON output
 * Returns "null" for invalid/NAN values (valid JSON format)
 */
String floatToStringOrNull(float value, int decimals = 2) {
  if (isnan(value)) {
    return "null";
  }
  return String(value, decimals);
}

// ------------------ BMP280 INITIALIZATION ------------------
/*
 * Initialize the BMP280 sensor
 * Tries both possible I2C addresses (0x76 and 0x77)
 * Configures sensor for forced measurement mode (power saving)
 * Returns true if sensor found and configured, false otherwise
 */
bool initBMP280() {
  // Try address 0x76 first (when SDO pin connected to GND)
  if (bmp.begin(0x76)) {
    Serial.println("✔ BMP280 detected at 0x76");
  } 
  // Try address 0x77 (when SDO pin connected to VCC)
  else if (bmp.begin(0x77)) {
    Serial.println("✔ BMP280 detected at 0x77");
  }
  // Sensor not found at either address
  else {
    Serial.println("✘ BMP280 NOT DETECTED at 0x76 or 0x77!");
    Serial.println("   Check: VCC, GND, SDA, SCL, CSB→3.3V, SDO→GND");
    return false;
  }
  
  // Configure sensor settings for accuracy and power efficiency
  bmp.setSampling(
    Adafruit_BMP280::MODE_FORCED,      // Take measurement only when requested (saves power)
    Adafruit_BMP280::SAMPLING_X2,      // Temperature oversampling x2 (good accuracy)
    Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling x16 (high accuracy)
    Adafruit_BMP280::FILTER_X16,       // IIR filter coefficient 16 (reduces noise)
    Adafruit_BMP280::STANDBY_MS_500    // Standby time between measurements
  );
  return true;
}

// ------------------ MPU6050 INITIALIZATION ------------------
/*
 * Initialize the MPU6050 motion sensor
 * Configures accelerometer and gyroscope ranges
 * Returns true if sensor found and configured, false otherwise
 */
bool initMPU6050() {
  // Try to find MPU6050 at I2C address 0x68 (when AD0 pin connected to GND)
  if (!mpu.begin(0x68)) {
    Serial.println("✘ MPU6050 NOT DETECTED!");
    return false;
  }
  
  // Configure sensor ranges and filtering
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);    // Accelerometer: ±8g range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);         // Gyroscope: ±500°/s range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);      // Low-pass filter: 21 Hz cutoff
  
  Serial.println("✔ MPU6050 detected");
  return true;
}

// ------------------ READ BMP280 DATA ------------------
/*
 * Read temperature, pressure, and altitude from BMP280
 * Includes auto-reconnection logic and data validation
 * Sets values to NAN if sensor fails or disconnected
 */
void readBMP280() {
  // If sensor is not connected, try to initialize it
  if (!bmpConnected) {
    bmpConnected = initBMP280();
    if (!bmpConnected) {
      // Still not connected - set all values to NAN and exit
      temperature = pressure = altitude = NAN;
      return;
    }
  }

  // Trigger a forced measurement (sensor was sleeping to save power)
  if (!bmp.takeForcedMeasurement()) {
    Serial.println("✘ BMP280 read failed!");
    bmpConnected = false;  // Mark sensor as disconnected
    temperature = pressure = altitude = NAN;
    return;
  }

  // Read sensor values
  temperature = bmp.readTemperature();           // Read temperature in °C
  pressure = bmp.readPressure() / 100.0F;        // Read pressure in Pa, convert to hPa
  altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);  // Calculate altitude based on pressure
  
  // Validate readings - check if values are physically possible
  // BMP280 operating range: -40°C to 85°C, 300 to 1100 hPa
  if (isnan(temperature) || isnan(pressure) || 
      temperature < -40 || temperature > 85 || 
      pressure < 300 || pressure > 1200) {
    Serial.println("✘ BMP280 readings invalid!");
    bmpConnected = false;  // Invalid readings mean sensor has failed
    temperature = pressure = altitude = NAN;
    return;
  }
}

// ------------------ READ MPU6050 DATA ------------------
/*
 * Read acceleration, rotation, and temperature from MPU6050
 * Includes auto-reconnection logic and data validation
 * Sets values to NAN if sensor fails or disconnected
 */
void readMPU6050() {
  // If sensor is not connected, try to initialize it
  if (!mpuConnected) {
    mpuConnected = initMPU6050();
    if (!mpuConnected) {
      // Still not connected - set all values to NAN and exit
      accelX = accelY = accelZ = NAN;
      gyroX = gyroY = gyroZ = NAN;
      mpuTemp = NAN;
      return;
    }
  }

  // Create event structures to hold sensor data
  sensors_event_t a, g, temp;
  
  // Try to read all sensor data
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println("✘ MPU6050 read failed!");
    mpuConnected = false;  // Mark sensor as disconnected
    accelX = accelY = accelZ = NAN;
    gyroX = gyroY = gyroZ = NAN;
    mpuTemp = NAN;
    return;
  }

  // Extract acceleration values (m/s²)
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  // Extract gyroscope values (rad/s)
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  
  // Extract temperature (°C)
  mpuTemp = temp.temperature;
  
  // Validate readings - check if values are physically reasonable
  // For accelerometer: ±20 m/s² is more reasonable than ±50 (allows for movement)
  // For gyroscope: ±100 rad/s is very high but possible during rapid movement
  if (isnan(accelX) || isnan(accelY) || isnan(accelZ) ||
      isnan(gyroX) || isnan(gyroY) || isnan(gyroZ) ||
      abs(accelX) > 20 || abs(accelY) > 20 || abs(accelZ) > 20 ||
      abs(gyroX) > 100 || abs(gyroY) > 100 || abs(gyroZ) > 100) {
    Serial.println("✘ MPU6050 readings invalid! Sensor disconnected.");
    mpuConnected = false;
    accelX = accelY = accelZ = NAN;
    gyroX = gyroY = gyroZ = NAN;
    mpuTemp = NAN;
    return;
  }
}

// ------------------ READ SOIL MOISTURE DATA ------------------
/*
 * Read soil moisture from HD-38 analog sensor
 * Converts raw ADC value to percentage (0-100%)
 * Calibrated using measured dry and wet values
 * For ski slope: measures snow moisture content
 */
void readSoilMoisture() {
  // Read analog value from sensor (12-bit ADC: 0-8191)
  soilRaw = analogRead(SOIL_PIN);
  
  // Validate ADC range
  if (soilRaw < 0 || soilRaw > 8191) {
    Serial.println("✘ Soil sensor reading invalid (out of range)!");
    soilRaw = SOIL_RAW_DRY;    // Default to dry reading
    soilPercent = 0;
    return;
  }
  
  // Convert raw value to percentage using calibration values
  // For ski slope: 0% = dry/no snow, 100% = wet/slushy snow
  soilPercent = map(soilRaw, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100);
  
  // Constrain result to 0-100% range (in case raw value goes outside calibration)
  soilPercent = constrain(soilPercent, 0, 100);
}

// ------------------ WARNING SYSTEM ------------------
/*
 * Check all sensor values against warning thresholds
 * Returns a string describing current status/warnings
 * Multiple warnings can be combined with "|" separator
 * Uses debouncing to prevent warning flicker
 */
String getWarning() {
  // Only update warnings every 3 seconds to prevent flickering
  if (millis() - lastWarningTime < WARNING_INTERVAL) {
    return cachedWarning;
  }
  
  lastWarningTime = millis();
  String warnings = "";  // Start with empty warnings string
  
  // Check BMP280 sensor and its thresholds
  if (!bmpConnected) {
    warnings += "BMP280 DISCONNECTED | ";
  } else {
    // Temperature thresholds (can be adjusted for your needs)
    if (!isnan(temperature)) {
      if (temperature > 45) warnings += "HIGH TEMP! | ";      // Danger: too hot
      if (temperature < -10) warnings += "LOW TEMP! | ";      // Danger: too cold
    }
    
    // Pressure thresholds
    if (!isnan(pressure)) {
      if (pressure < 950) warnings += "LOW PRESSURE! | ";     // Storm approaching
      if (pressure > 1050) warnings += "HIGH PRESSURE! | ";   // Extreme high pressure
    }
  }
  
  // Check MPU6050 sensor
  if (!mpuConnected) {
    warnings += "MPU6050 DISCONNECTED | ";
  }
  
  // Check snow/soil moisture thresholds (for ski slope monitoring)
  if (soilPercent < 20) warnings += "SNOW TOO DRY / NO SNOW! | ";     // Needs snowmaking
  if (soilPercent > 80) warnings += "SNOW TOO WET / SLUSHY! | ";      // Too much water content
  
  // Cache the result
  if (warnings == "") {
    cachedWarning = "NORMAL";
  } else {
    cachedWarning = warnings;
  }
  
  return cachedWarning;
}

// ------------------ HTML WEB DASHBOARD ------------------
/*
 * Generate HTML page for web dashboard
 * Shows all sensor readings with color-coded status
 * Auto-refreshes every 3 seconds
 * Displays "N/A" instead of "nan" for invalid readings
 */
String htmlPage() {
  String warning = getWarning();  // Get current warning status
  String color = "green";         // Default color for normal status
  
  // Set color based on warning type
  if (warning.indexOf("!") >= 0) color = "red";              // Red for danger warnings
  else if (warning.indexOf("DISCONNECTED") >= 0) color = "orange";  // Orange for disconnected sensors

  // Start building HTML page
  String page = "<html><head>";
  page += "<meta charset='UTF-8'>";                          // Support special characters
  page += "<meta http-equiv='refresh' content='3'>";         // Auto-refresh every 3 seconds
  
  // CSS styling for clean dashboard appearance
  page += "<style>";
  page += "body{font-family:Arial;margin:20px;} ";
  page += ".sensor{border:1px solid #ccc;padding:10px;margin:10px 0;border-radius:5px;} ";
  page += ".warn{font-size:18px;color:" + color + ";font-weight:bold;padding:10px;background:#f0f0f0;}";
  page += "</style>";
  page += "</head><body>";

  // Page title
  page += "<h1>ESP32 Multi-Sensor Dashboard</h1>";
  
  // BMP280 sensor section
  page += "<div class='sensor'><h3>BMP280 - Environment</h3>";
  if (bmpConnected) {
    // Show readings if sensor is connected (use N/A for invalid values)
    page += "<p><b>Temperature:</b> " + floatToStringOrNA(temperature) + " &deg;C</p>";
    page += "<p><b>Pressure:</b> " + floatToStringOrNA(pressure) + " hPa</p>";
    page += "<p><b>Altitude:</b> " + floatToStringOrNA(altitude) + " m</p>";
    page += "<p style='font-size:12px;color:#666;'>Note: Altitude accuracy depends on sea-level pressure setting (" + String(SEA_LEVEL_PRESSURE, 2) + " hPa)</p>";
  } else {
    // Show error message if sensor disconnected
    page += "<p style='color:red;'>SENSOR NOT CONNECTED</p>";
  }
  page += "</div>";
  
  // MPU6050 sensor section
  page += "<div class='sensor'><h3>MPU6050 - Motion</h3>";
  if (mpuConnected) {
    // Show readings if sensor is connected (use N/A for invalid values)
    page += "<p><b>Accel X/Y/Z:</b> " + floatToStringOrNA(accelX) + " / " + floatToStringOrNA(accelY) + " / " + floatToStringOrNA(accelZ) + " m/s&sup2;</p>";
    page += "<p><b>Gyro X/Y/Z:</b> " + floatToStringOrNA(gyroX) + " / " + floatToStringOrNA(gyroY) + " / " + floatToStringOrNA(gyroZ) + " rad/s</p>";
    page += "<p><b>MPU Temp:</b> " + floatToStringOrNA(mpuTemp) + " &deg;C</p>";
  } else {
    // Show error message if sensor disconnected
    page += "<p style='color:red;'>SENSOR NOT CONNECTED</p>";
  }
  page += "</div>";
  
  // Soil moisture sensor section
  page += "<div class='sensor'><h3>HD-38 - Snow Moisture</h3>";
  page += "<p><b>Moisture:</b> " + String(soilPercent) + "% (Raw: " + String(soilRaw) + ")</p>";
  page += "<p style='font-size:12px;color:#666;'>Calibrated: Dry/No Snow=" + String(SOIL_RAW_DRY) + ", Wet/Slushy=" + String(SOIL_RAW_WET) + "</p>";
  page += "</div>";
  
  // Status/warning section with color-coded background
  page += "<div class='warn'>STATUS: " + warning + "</div>";
  
  page += "</body></html>";
  return page;
}

// ------------------ JSON API ------------------
/*
 * Generate JSON formatted data for API access
 * Returns all sensor readings in structured JSON format
 * Can be used by other applications to read sensor data
 * Uses "null" for invalid values (proper JSON format, not "nan")
 */
String jsonData() {
  String json = "{";
  
  // BMP280 data object
  json += "\"bmp280\":{";
  json += "\"connected\":" + String(bmpConnected ? "true" : "false") + ",";
  json += "\"temperature\":" + floatToStringOrNull(temperature) + ",";
  json += "\"pressure\":" + floatToStringOrNull(pressure) + ",";
  json += "\"altitude\":" + floatToStringOrNull(altitude) + ",";
  json += "\"sea_level_pressure\":" + String(SEA_LEVEL_PRESSURE, 2);
  json += "},";
  
  // MPU6050 data object
  json += "\"mpu6050\":{";
  json += "\"connected\":" + String(mpuConnected ? "true" : "false") + ",";
  json += "\"accel\":{\"x\":" + floatToStringOrNull(accelX) + ",\"y\":" + floatToStringOrNull(accelY) + ",\"z\":" + floatToStringOrNull(accelZ) + "},";
  json += "\"gyro\":{\"x\":" + floatToStringOrNull(gyroX) + ",\"y\":" + floatToStringOrNull(gyroY) + ",\"z\":" + floatToStringOrNull(gyroZ) + "},";
  json += "\"temperature\":" + floatToStringOrNull(mpuTemp);
  json += "},";
  
  // Soil moisture data object
  json += "\"snow_moisture\":{";
  json += "\"raw\":" + String(soilRaw) + ",";
  json += "\"percent\":" + String(soilPercent) + ",";
  json += "\"calibration\":{\"dry\":" + String(SOIL_RAW_DRY) + ",\"wet\":" + String(SOIL_RAW_WET) + "}";
  json += "},";
  
  // Current warning status
  json += "\"warning\":\"" + getWarning() + "\"";
  json += "}";
  return json;
}

// ------------------ SETUP (RUNS ONCE AT STARTUP) ------------------
void setup() {
  // Initialize serial communication for debugging (115200 baud rate)
  Serial.begin(115200);
  delay(1000);  // Wait for serial connection to stabilize
  Serial.println("ESP32 Starting...");

  // Initialize I2C bus with specified pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Configure analog pin for soil moisture sensor
  pinMode(SOIL_PIN, INPUT);

  // Try to initialize both sensors
  bmpConnected = initBMP280();
  mpuConnected = initMPU6050();

  // Setup WiFi in dual mode (Station + Access Point)
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(STA_SSID, STA_PASS);  // Try to connect to existing network

  // Wait up to 10 seconds for WiFi connection
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    Serial.print(".");
    delay(500);
  }

  // Check if WiFi connected successfully
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✔ WiFi Connected");
    Serial.println(WiFi.localIP());  // Print IP address for web access
  } else {
    Serial.println("\n✘ WiFi Connection failed");
  }

  // Start Access Point as backup (works even if station mode fails)
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());  // Print AP IP address

  // Setup web server routes
  server.on("/", []() { 
    server.send(200, "text/html", htmlPage());  // Main dashboard at root URL
  });
  server.on("/json", []() { 
    server.send(200, "application/json", jsonData());  // JSON data at /json
  });
  
  // Start the web server
  server.begin();
  
  Serial.println("System ready!");
}

// ------------------ LOOP (RUNS CONTINUOUSLY) ------------------
void loop() {
  // Read all sensors
  readBMP280();
  readMPU6050();
  readSoilMoisture();
  
  // Handle incoming web requests
  server.handleClient();

  // Print all readings to Serial Monitor
  Serial.println("========== SENSOR READINGS ==========");
  
  // Print BMP280 data
  Serial.print("BMP280 - Temp: ");
  Serial.print(floatToStringOrNA(temperature));
  Serial.print("°C | Pressure: ");
  Serial.print(floatToStringOrNA(pressure));
  Serial.print("hPa | Altitude: ");
  Serial.print(floatToStringOrNA(altitude));
  Serial.println("m");
  
  // Print MPU6050 data
  Serial.print("MPU6050 - Accel: ");
  Serial.print(floatToStringOrNA(accelX));
  Serial.print(",");
  Serial.print(floatToStringOrNA(accelY));
  Serial.print(",");
  Serial.print(floatToStringOrNA(accelZ));
  Serial.print(" | Gyro: ");
  Serial.print(floatToStringOrNA(gyroX));
  Serial.print(",");
  Serial.print(floatToStringOrNA(gyroY));
  Serial.print(",");
  Serial.println(floatToStringOrNA(gyroZ));
  
  // Print soil moisture data
  Serial.print("Snow Moisture - ");
  Serial.print(soilPercent);
  Serial.print("% (Raw: ");
  Serial.print(soilRaw);
  Serial.println(")");
  
  // Print current status
  Serial.print("Status: ");
  Serial.println(getWarning());
  Serial.println("=====================================\n");

  // Wait 2 seconds before next reading
  delay(2000);
}
