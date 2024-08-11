#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#define ssid "Shrikrishna"
#define password "shrikrishna"
#define NTP_SERVER "pool.ntp.org"
#define JSON_BUFFER_SIZE 1024
#define RETRY_INTERVAL 10000 // Retry interval in milliseconds
#define MAX_RETRY_ATTEMPTS 5

// Thresholds for detecting air quality and smoke
#define MQ2_THRESHOLD 50
#define MQ135_THRESHOLD 50

// Initialize the LCD with address 0x27, 20 columns, and 4 rows
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Define the built-in LED pin (GPIO2, usually labeled as D4)
const int ledPin = D4; // This corresponds to GPIO2 on NodeMCU

// Variable to store the last update time
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 1000; // 1 second

typedef struct {
  float temp_d;
  float humid_d;
  float presr_d;
  float Air_q; // Raw MQ135 value
  float Gas_d; // Raw MQ2 value
  float motion_d;
  float level_d;
  bool airQuality;  // Air quality boolean
  bool smokeDetected; // Smoke detected boolean
  float Air_q_percent; // Percentage representation of MQ135
  float Gas_d_percent; // Percentage representation of MQ2
} data_t;

data_t data_;

WiFiClient wifiClient;
HTTPClient httpClient;

void setup() {
  // Initialize serial communication with baud rate 9600 (for debugging)
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();

  connectToWiFi();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RTOS BASED IOT DATA");
  lcd.setCursor(0, 1);
  lcd.print(" ACQUASITION SYSTEM ");
  delay(2000);
  // Synchronize time
  configTime(0, 0, NTP_SERVER);

  // Initialize I2C communication
  Wire.begin(D2, D1); // SDA=D2, SCL=D1 (change these if needed)

  // Initialize the LCD
  // Initialize built-in LED pin as output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();

  // Update the LCD display and send data to the server every 1 second
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;

    // Check if data is available on UART0
    if (Serial.available()) {
      // Read the incoming data
      String dataString = Serial.readStringUntil('\n');
      dataString.trim();

      // Initialize values with default "0.00" if not found
      data_.temp_d = 0.00;
      data_.humid_d = 0.00;
      data_.presr_d = 0.00;
      data_.Air_q = 0.00;
      data_.Gas_d = 0.00;
      data_.motion_d = 0.00;
      data_.level_d = 0.00;
      data_.airQuality = false;
      data_.smokeDetected = false;
      data_.Air_q_percent = 0.00;
      data_.Gas_d_percent = 0.00;

      // Extract values from the data string
      int index = dataString.indexOf("T:");
      if (index != -1) {
        int endIndex = dataString.indexOf(',', index);
        data_.temp_d = dataString.substring(index + 2, endIndex).toFloat();
      }

      index = dataString.indexOf("P:");
      if (index != -1) {
        int endIndex = dataString.indexOf(',', index);
        data_.presr_d = dataString.substring(index + 2, endIndex).toFloat();
      }

      index = dataString.indexOf("H:");
      if (index != -1) {
        int endIndex = dataString.indexOf(',', index);
        data_.humid_d = dataString.substring(index + 2, endIndex).toFloat();
      }

      index = dataString.indexOf("mq135:");
      if (index != -1) {
        int endIndex = dataString.indexOf(',', index);
        data_.Air_q = dataString.substring(index + 6, endIndex).toFloat();
        // Calculate percentage and set boolean for air quality based on threshold
        data_.Air_q_percent = (data_.Air_q / 4096.0) * 100.0;
        data_.airQuality = data_.Air_q > MQ135_THRESHOLD;
      }

      index = dataString.indexOf("mq2:");
      if (index != -1) {
        int endIndex = dataString.indexOf(',', index);
        data_.Gas_d = dataString.substring(index + 4, endIndex).toFloat();
        // Calculate percentage and set boolean for smoke detection based on threshold
        data_.Gas_d_percent = (data_.Gas_d / 4096.0) * 100.0;
        data_.smokeDetected = data_.Gas_d > MQ2_THRESHOLD;
      }

      index = dataString.indexOf("L:");
      if (index != -1) {
        int endIndex = dataString.indexOf(',', index);
        data_.level_d = dataString.substring(index + 2, endIndex).toFloat();
      }

      // Only display data if all values are non-zero
      if (areValuesReady()) {
        // Display the parsed values on the LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:" + formatValue(String(data_.temp_d, 2), 6));
        lcd.print(" P:" + formatValue(String(data_.presr_d, 2), 6));

        lcd.setCursor(0, 1);
        lcd.print("H:" + formatValue(String(data_.humid_d, 2), 6));
        lcd.print(" AQ:" + formatValue(String(data_.Air_q_percent, 1), 4) + "%");

        lcd.setCursor(0, 2);
        lcd.print("GD:" + formatValue(String(data_.Gas_d_percent, 1), 4) + "%");
        
        lcd.setCursor(0, 3);
        lcd.print("L:" + formatValue(String(data_.level_d, 2), 6)+ "CM");
        delay(2000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Air Quality:" + String(data_.airQuality ? "GOOD" : "BAD"));
        lcd.setCursor(0, 1);
        lcd.print("Smoke Detected:" + String(data_.smokeDetected ? "NO" : "YES"));
        delay(2000);
        
        
        // Blink the built-in LED
        digitalWrite(ledPin, HIGH); // Turn LED on
        delay(500);                 // Wait for 500ms
        digitalWrite(ledPin, LOW);  // Turn LED off
        delay(500);                 // Wait for 500ms

        // Send data to the server
        String jsonData = structureToJson(data_);
        if (sendToServer(jsonData)) {
          // Update the LCD with sending status
          lcd.clear();
          lcd.setCursor(0, 3);
          lcd.print("Sent: Success");
          delay(1000);
        } else {
          // Update the LCD with sending status
          lcd.clear();
          lcd.setCursor(0, 3);
          lcd.print("Sent: Failure");
          delay(1000);
        }
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Waiting for data...");
        delay(500); 
      }
    }
  }
}


void connectToWiFi() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_RETRY_ATTEMPTS) {
    delay(RETRY_INTERVAL);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    lcd.setCursor(0, 1);
    lcd.print("successfully");
    delay(2000);
  } else {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Failed");
    delay(2000);
    // Handle WiFi connection failure
    // You might want to add code to handle this situation, like restarting the device or entering into a configuration mode.
  }
}

String structureToJson(data_t structuredData) {
  DynamicJsonDocument jsonDocument(JSON_BUFFER_SIZE);
  JsonObject obj = jsonDocument.to<JsonObject>(); // Create a JSON object

  obj["temperature"] = structuredData.temp_d;
  obj["humidity"] = structuredData.humid_d;
  obj["pressure"] = structuredData.presr_d;
  obj["Air_quality"] = structuredData.Air_q;
  obj["Gas_detection"] = structuredData.Gas_d;
  obj["Motion_detection"] = structuredData.motion_d;
  obj["Level_detection"] = structuredData.level_d;
  obj["Air_quality_percent"] = structuredData.Air_q_percent;
  obj["Gas_detection_percent"] = structuredData.Gas_d_percent;
  obj["Air_quality_boolean"] = structuredData.airQuality;
  obj["Smoke_detected_boolean"] = structuredData.smokeDetected;

  String jsonString;
  serializeJsonPretty(obj, jsonString);

  return jsonString;
}

bool sendToServer(String jsonData) {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor(0, 0);
    lcd.print("WiFi not connected");
    return false;
  }

  httpClient.begin(wifiClient, "http://192.168.12.222:5000/data"); // Change URL accordingly
  httpClient.addHeader("Content-Type", "application/json");

  int httpCode = httpClient.POST(jsonData);

  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      String payload = httpClient.getString();
      return true;
    }
  }

  return false;
}

// Function to format values to fit within a specific width
String formatValue(String value, int width) {
  if (value.length() > width) {
    return value.substring(0, width); // Truncate if too long
  } else {
    while (value.length() < width) {
      value = " " + value; // Pad with spaces if too short
    }
    return value;
  }
}
bool areValuesReady() {
  return data_.temp_d > 0.00 &&
         data_.humid_d > 0.00 &&
         data_.presr_d > 0.00 &&
         data_.Air_q > 0.00 &&
         data_.Gas_d > 0.00 &&
         data_.level_d > 0.00;
}
