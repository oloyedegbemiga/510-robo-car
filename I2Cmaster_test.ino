// ESP32-C3 I2C Master Code
// Test code writes and receives
#include <Wire.h>

#define I2C_SLAVE_ADDR 0x28
#define SDA_PIN 40
#define SCL_PIN 41

// Health system
uint8_t health = 100;
uint8_t wifiPacketCount = 0;

bool motorsRunning = true;

// Wi-Fi Packet Simulation (increment packet count)
void simulateWifiPacket() {
  send_I2C_byte(0x01);  // Simulate sending a Wi-Fi packet
  wifiPacketCount++;
  Serial.printf("Wi-Fi Packet Sent! Total Packets: %d\n", wifiPacketCount);
}

void send_I2C_byte(uint8_t data) {
  // Send data to slave
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(data);  // Send some test data
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("Data sent successfully");
    rgbLedWrite(2, 0, 20, 0);  // Green
  } else {
    Serial.printf("Error sending data: %d\n", error);
    rgbLedWrite(2, 20, 0, 0);  // Red
  }
}

uint8_t receive_I2C_byte() {
  // Request data from slave
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 1);
  uint8_t byteIn = 0;

  if (bytesReceived > 0) {
    Serial.print("Received from slave: ");
    while (Wire.available()) {
      byteIn = Wire.read();
      Serial.printf("0x%02X ", byteIn);
    }
    Serial.println();
  } else {
    Serial.println("No data received from slave.");
  }
  return byteIn;  // Return the received byte (current health)
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C master
  Wire.begin(SDA_PIN, SCL_PIN, 40000);
  Serial.println("ESP32-C3 I2C Master initialized");
  Serial.printf("SDA: %d, SCL: %d\n", SDA_PIN, SCL_PIN);
}

void loop() {
  static unsigned long lastExecutionTime = 0; // Keeps track of the last execution time
  unsigned long currentTime = millis();      // Get the current time

  // Check if 500 ms (2 Hz period) has passed since the last execution
  if (currentTime - lastExecutionTime >= 500) {
    lastExecutionTime = currentTime; // Update the last execution time

    // Simulate Wi-Fi packet transmission
    simulateWifiPacket();

    // Receive current health from the slave
    uint8_t health = receive_I2C_byte();
    Serial.printf("Current Health: %d\n", health);

    // Check if health is zero to stop motors
    if (health = 0) {
      Serial.println("HEALTH IS ZERO, TURNING OFF ROBOT!");
      rgbLedWrite(2, 0, 0, 0); // Turn off LED or indicate stop
      motorsRunning = false;   // Flag to indicate motors are off
    }
  }

  // Other non-time-critical tasks can go here
}
