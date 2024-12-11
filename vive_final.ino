/*
 * Sample Vive code
 */
#include "vive510.h"

#define SIGNALPIN1 37 // pin receiving signal from Vive circuit 1 
#define SIGNALPIN2 38 // pin receiving signal from Vive circuit 2

Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

#define FREQ 1 // in Hz
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);

  vive1.begin();
  vive2.begin();
  delay(2000);
  Serial.println("Vive trackers started");
}
                 
uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;  
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else    middle = (a <= b) ? a : b;
  return middle;
}
                               
void loop() {  
  static uint16_t x1, y1, x2, y2;
  static float yaw; // To store the yaw angle

  // Read data from the first Vive tracker
  if (vive1.status() == VIVE_RECEIVING) {
    static uint16_t x0_1, y0_1, oldx1_1, oldx2_1, oldy1_1, oldy2_1;
    oldx2_1 = oldx1_1; oldy2_1 = oldy1_1;
    oldx1_1 = x0_1;     oldy1_1 = y0_1;

    x0_1 = vive1.xCoord();
    y0_1 = vive1.yCoord();
    x1 = med3filt(x0_1, oldx1_1, oldx2_1);
    y1 = med3filt(y0_1, oldy1_1, oldy2_1);

    if (x1 > 8000 || y1 > 8000 || x1 < 1000 || y1 < 1000) {
      x1 = 0; y1 = 0;
    }
  } else {
    x1 = 0;
    y1 = 0;
    vive1.sync(5);
  }

  // Read data from the second Vive tracker
  if (vive2.status() == VIVE_RECEIVING) {
    static uint16_t x0_2, y0_2, oldx1_2, oldx2_2, oldy1_2, oldy2_2;
    oldx2_2 = oldx1_2; oldy2_2 = oldy1_2;
    oldx1_2 = x0_2;     oldy1_2 = y0_2;

    x0_2 = vive2.xCoord();
    y0_2 = vive2.yCoord();
    x2 = med3filt(x0_2, oldx1_2, oldx2_2);
    y2 = med3filt(y0_2, oldy1_2, oldy2_2);

    if (x2 > 8000 || y2 > 8000 || x2 < 1000 || y2 < 1000) {
      x2 = 0; y2 = 0;
    }
  } else {
    x2 = 0;
    y2 = 0;
    vive2.sync(5);
  }

  // Calculate the yaw angle
  if (x1 != 0 && y1 != 0 && x2 != 0 && y2 != 0) { // Ensure both trackers have valid data
    float dx = x2 - x1;
    float dy = y2 - y1;
    yaw = atan2(dy, dx) * 180 / PI; // Yaw in degrees

    // Normalize the angle to 0-360 degrees
    if (yaw < 0) {
      yaw += 360;
    }

    // Output the yaw angle
    Serial.print("Yaw angle: ");
    Serial.print(yaw);
    Serial.println(" degrees");
  } else {
    yaw = 0; // Reset yaw if data is invalid
    Serial.println("Invalid data for yaw calculation.");
  }

  // Output the coordinates for both Vive trackers
  Serial.print("Vive1: X1 - ");
  Serial.print(x1);
  Serial.print(" Y1 - ");
  Serial.print(y1);

  Serial.print("\t Vive2: X2 - ");
  Serial.print(x2);
  Serial.print(" Y2 - ");
  Serial.println(y2);

  // Control LED for status
  if ((x1 != 0 && y1 != 0) || (x2 != 0 && y2 != 0)) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(10);
}