#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

// WiFi Credentials
const char *ssid = "Robocar";
const char *password = "12345678";

// Web server on port 80
WebServer server(80);

// Motor control values array
float motorControl[3] = {0, 0.0, 0};  // Initial values for the array

void setup() {
  Serial.begin(115200);

  // Connectting to WiFi in AP mode
  WiFi.softAP(ssid, password);
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  // Defining routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/setSpeed", HTTP_GET, handleSetSpeed);
  server.on("/setDirection", HTTP_GET, handleSetDirection);
  // server.on("/setMotorState", HTTP_GET, handleSetMotorState);
  server.begin();
}

void loop() {
  server.handleClient();
}

// Function to handle root 
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

// Setting motor speed
void handleSetSpeed() {
  if (server.hasArg("value")) {
    motorControl[2] = server.arg("value").toInt();
    Serial.print("Speed set to: ");
    Serial.println(motorControl[2]);
  }
  server.send(204);
}

// Setting motor direction 
void handleSetDirection() {
  if (server.hasArg("dir")) {
    int dir = server.arg("dir").toInt();
    if (dir == 2) {
      motorControl[1] -= 0.25;  // Decrementing by 0.25 - Left
    } else if (dir == 3) {
      motorControl[1] += 0.25;  // Increment by 0.25 - Right
    }
    else if (dir == 1){
      motorControl[0] = 1; // Up
    }
    else if (dir == -1){
      motorControl[0] = -1; // Down
    }
    else if (dir == 0){
      motorControl[0] = 0; // Stop
    }

  }
  server.send(204);
}

