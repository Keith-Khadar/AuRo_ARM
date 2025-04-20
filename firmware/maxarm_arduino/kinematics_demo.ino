#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "ESPMax.h"
#include "_espmax.h"

// Replace with your network credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Set specific IP address for the access point
IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

float pos_[3] ={0, 0, 0};

// Handle WebSocket events
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      // Handle received data if needed
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = String((char*)data);
    Serial.println("Received message: " + message);
    
    // Try to parse as JSON
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message);
    
    // If parsing succeeds and JSON contains x, y, z coordinates
    if (!error && doc.containsKey("x") && doc.containsKey("y") && doc.containsKey("z")) {
      float x = doc["x"];
      float y = doc["y"];
      float z = doc["z"];
      
      // Call your set_position function
      float pos_from_origin[3]={x,y,z};
      set_position_from_origin(pos_from_origin, 1000);
    }
  }
}

// Send sensor data to connected clients
void sendSensorData() {
  // Read arm position
  read_position(pos_);

  // Convert to JSON format
  String jsonString = "{\"x\":" + String(pos_[0]) + 
                      ",\"y\":" + String(pos_[1]) + 
                      ",\"z\":" + String(pos_[2]) + "}";
  
  // Send to all connected clients
  ws.textAll(jsonString);
}

void setup() {
  ESPMax_init();
  go_home(2000);

  Serial.begin(9600);

  // Configure the soft AP with a specific IP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, gateway, subnet);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)...");
  // Remove the password parameter if you want the AP to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Initialize WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Start server
  server.begin();
  Serial.println("WebSocket server started");
  Serial.println("Connect to the WebSocket at ws://192.168.4.1/ws");
}

unsigned long previousMillis = 0;
const long interval = 1000;  // Send data every second

void loop() {
  ws.cleanupClients();  // Clean up disconnected clients
  
  // Send sensor data periodically
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendSensorData();
  }
}