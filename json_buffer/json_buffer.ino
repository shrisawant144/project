// #include <ESP8266HTTPClient.h>
// #include <ArduinoJson.h>
// #include <time.h> 
// #include <ESP8266WiFi.h>
// #include <WiFiClient.h>
// #define ssid "Shrikrishna"
// #define password "shrikrishna"
// typedef struct data 
// {
//   float temp_d;
//   float humid_d;
//   float presr_d;
//   float Air_q;
//   float Gas_d;
//   float motion_d;
//   float level_d;
// }data_t;
// #define JSON_BUFFER_SIZE  1024
// data_t data_[10];
// void setup() {
//   Serial.begin(9600);
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println();
//   Serial.println("connected");
//   configTime(0, 0, "pool.ntp.org");

// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   for(int i = 0;i < 10;i++)
//   {
//     data_[i].temp_d = random(1,100);
//     data_[i].humid_d = random(1,100);
//     data_[i].presr_d = random(1,100);
//     data_[i].Air_q = random(1,100);
//     data_[i].Gas_d = random(1,100);
//     data_[i].level_d = random(1,100);
//     data_[i].motion_d = random(1,100);
//   }
//   String jsondata =structuretojson(data_,10);
//   sendToGoogleAppsScript(jsondata);

// }

// String structuretojson(struct data structuredArray[], size_t arraySize) {
//   DynamicJsonDocument jsonDocument(JSON_BUFFER_SIZE);
//   JsonArray jsonArray = jsonDocument.createNestedArray(); // Create a JSON array

//     for (int i = 0; i < arraySize; i++) {
//     JsonObject obj = jsonArray.createNestedObject();
//     obj["temperature"] = data_[i].temp_d; // Add current timestamp in seconds since 1970
//     obj["humidity"] = data_[i].humid_d;
//     obj["pressure"] = data_[i].presr_d;
//     obj["Air_quality"] = data_[i].Air_q;
//     obj["Gas_detection"] = data_[i].Gas_d;
//     obj["Motion_detection"] = data_[i].motion_d;     // Used lowercase "speed"
//     obj["Level_detection"] = data_[i].level_d;
//   }

//   String jsonString;
//   serializeJson(jsonArray, jsonString);
//   Serial.println(jsonString);

//   jsonDocument.clear();
//   return jsonString;
// }

// void sendToGoogleAppsScript(String jsonData) {
//   if ((WiFi.status() == WL_CONNECTED)) {

//     WiFiClient client;
//     HTTPClient http;

//     Serial.print("[HTTP] begin...\n");
//     // configure traged server and url
//     http.begin(client, "http://192.168.122.222:4000/data");  // HTTP
//     http.addHeader("Content-Type", "application/json");

//     Serial.print("[HTTP] POST...\n");
//     // start connection and send HTTP header and body
//     int httpCode = http.POST(jsonData);

//     // httpCode will be negative on error
//     if (httpCode > 0) {
//       // HTTP header has been send and Server response header has been handled
//       Serial.printf("[HTTP] POST... code: %d\n", httpCode);

//       // file found at server
//       if (httpCode == HTTP_CODE_OK) {
//         const String& payload = http.getString();
//         Serial.println("received payload:\n<<");
//         Serial.println(payload);
//         Serial.println(">>");
//       }
//     } else {
//       Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
//     }

//     http.end();
//   }

//   delay(10000);
// }

#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h> 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#define ssid "Shrikrishna"
#define password "shrikrishna"
#define NTP_SERVER "pool.ntp.org"
#define JSON_BUFFER_SIZE 1024
#define DATA_SIZE 10
#define RETRY_INTERVAL 10000 // Retry interval in milliseconds
#define MAX_RETRY_ATTEMPTS 5

typedef struct {
  float temp_d;
  float humid_d;
  float presr_d;
  float Air_q;
  float Gas_d;
  float motion_d;
  float level_d;
} data_t;

data_t data_[DATA_SIZE];

WiFiClient wifiClient;
HTTPClient httpClient;

void setup() {
  Serial.begin(9600);

  connectToWiFi();

  // Synchronize time
  configTime(0, 0, NTP_SERVER);
}

void loop() {
  generateRandomData();
  String jsonData = structureToJson(data_, DATA_SIZE);

  if (sendToServer(jsonData)) {
    // Data sent successfully
    Serial.println("Data sent successfully");
  } else {
    // Failed to send data
    Serial.println("Failed to send data");
  }

  delay(10000); // Wait for next iteration
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_RETRY_ATTEMPTS) {
    delay(RETRY_INTERVAL);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
  } else {
    Serial.println("\nFailed to connect to WiFi");
    // Handle WiFi connection failure
    // You might want to add code to handle this situation, like restarting the device or entering into a configuration mode.
  }
}

void generateRandomData() {
  for (int i = 0; i < DATA_SIZE; i++) {
    data_[i].temp_d = random(1, 100);
    data_[i].humid_d = random(1, 100);
    data_[i].presr_d = random(1, 100);
    data_[i].Air_q = random(1, 100);
    data_[i].Gas_d = random(1, 100);
    data_[i].level_d = random(1, 100);
    data_[i].motion_d = random(1, 100);
  }
}

String structureToJson(data_t structuredArray[], size_t arraySize) {
  DynamicJsonDocument jsonDocument(JSON_BUFFER_SIZE);
  JsonArray jsonArray = jsonDocument.to<JsonArray>(); // Create a JSON array

  for (int i = 0; i < arraySize; i++) {
    JsonObject obj = jsonArray.createNestedObject();
    obj["temperature"] = structuredArray[i].temp_d;
    obj["humidity"] = structuredArray[i].humid_d;
    obj["pressure"] = structuredArray[i].presr_d;
    obj["Air_quality"] = structuredArray[i].Air_q;
    obj["Gas_detection"] = structuredArray[i].Gas_d;
    obj["Motion_detection"] = structuredArray[i].motion_d;
    obj["Level_detection"] = structuredArray[i].level_d;
  }

  String jsonString;
  serializeJsonPretty(jsonArray, jsonString);
  Serial.println(jsonString);

  return jsonString;
}

bool sendToServer(String jsonData) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }

  httpClient.begin(wifiClient, "http://192.168.122.222:4000/data"); // Change URL accordingly
  httpClient.addHeader("Content-Type", "application/json");

  int httpCode = httpClient.POST(jsonData);

  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      String payload = httpClient.getString();
      Serial.println("Server response:");
      Serial.println(payload);
      httpClient.end();
      return true;
    } else {
      Serial.printf("HTTP POST failed, error: %s\n", httpClient.errorToString(httpCode).c_str());
    }
  } else {
    Serial.println("HTTP POST failed, connection error");
  }

  httpClient.end();
  return false;
}
