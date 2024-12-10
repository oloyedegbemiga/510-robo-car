#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "index.h"
#include "html510.h"
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// set the pins to shutdown
#define SHT_LOX1 41
#define SHT_LOX2 40
#define SHT_LOX3 39

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 10

// Meters
#define HALF_WHEEL_BASE_B 0.13
#define WHEEL_RADIUS .07
#define PI 3.14159265358979323846


#define KD -.01
//*******WIFI***********//
// WiFi Credentials
const char *ssid = "Robocar";
const char *password = "12345678";
// Web server on port 80
WebServer server(80);

// Motor control values array
float motorControl[3] = {0.0, 0.0, 0.0}; 


//*******WIFI***********//

hw_timer_t * timer = NULL;
bool ret = false;
bool ledsig = false;

const int encoderPIN1[4] = {4, 5, 6, 7}; // channel 1 for 4 encoders
const int motorPINEN[4] = {15, 16, 17, 18}; // motor EN PWM pins
const int motorPINDIR[4] = {20, 3, 46, 21}; // motor DIR pins

volatile long ticksPerInterval[4] = {0, 0, 0, 0};

// We dictate motorDir after we set the directions on the motor directly
// To ensure the ISR will update correctly
volatile long motorDir[4] = {1, 1, 1, 1};

// Rad/seconds
volatile float motorVelSetpoint[4] = {0.0, 0.0, 0.0, 0.0};
volatile float kpMotor[4] = {12, 12, 12, 12};
volatile float kpMotorAng[4] = {2, 2, 2, 2};


volatile float currentWheelAngVel[4] = {0.0, 0.0, 0.0, 0.0};
volatile unsigned long currentWheelPwm[4] = {0, 0, 0, 0};

int CPR = 13.75;
float rotPerTick = 360.0 / CPR;
const int timer_interval = 100;

// after 
int initializeControl = 0;

void IRAM_ATTR encoderISR0() { ticksPerInterval[0] += motorDir[0];}
void IRAM_ATTR encoderISR1() { ticksPerInterval[1] += motorDir[1];}
void IRAM_ATTR encoderISR2() { ticksPerInterval[2] += motorDir[2];}
void IRAM_ATTR encoderISR3() { ticksPerInterval[3] += motorDir[3];}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));

    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot Third VL53L0X"));
    while(1);
  }


}

void read_tof() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.println(measure1.RangeMilliMeter);
  } else {
    Serial.println(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.println(measure2.RangeMilliMeter);
  } else {
    Serial.println(F("Out of range"));
  }


  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("3: "));
  if(measure3.RangeStatus != 4) {
    Serial.println(measure3.RangeMilliMeter);
  } else {
    Serial.println(F("Out of range"));
  }

  
  Serial.println();
}



void calcMotorVelSetpoint(float lin_vel, float ang_vel) {
  // we want to return an array that has each motor velocity

  float ang_v_left = (lin_vel - HALF_WHEEL_BASE_B * ang_vel) / WHEEL_RADIUS;
  float ang_v_right = (lin_vel + HALF_WHEEL_BASE_B * ang_vel) / WHEEL_RADIUS;

  motorVelSetpoint[0] = ang_v_right;
  motorVelSetpoint[1] = ang_v_right;
  motorVelSetpoint[2] = ang_v_left;
  motorVelSetpoint[3] = ang_v_left;

  Serial.println("Motor Setpoints:");

  Serial.print("Motor1: ");
  Serial.print(motorVelSetpoint[0]);
  Serial.println(" rad/s");

  Serial.print("Motor2: ");
  Serial.print(motorVelSetpoint[1]);
  Serial.println(" rad/s");

  Serial.print("Motor3: ");
  Serial.print(motorVelSetpoint[2]);
  Serial.println(" rad/s");

  Serial.print("Motor4: ");
  Serial.print(motorVelSetpoint[3]);
  Serial.println(" rad/s");

}

void controlMotor(float lin_vel, float ang_vel) {
  
  static unsigned long previous_timestamp;
  unsigned long current_timestamp = millis();
  static int prev_tick_dot[4];

  if (initializeControl == 0) {
    previous_timestamp = current_timestamp;
    initializeControl = 1;
    return;
  }

  double time_diff = current_timestamp - previous_timestamp;

  int u[4];
  float current_wheel_vel[4];
  static int prev_tick[4];
  static int ticks_between_control[4];
  Serial.println("Start");
  for (int i = 0; i < 4; i++) {
    // if (i == 0) {
    //   Serial.println("Ticks:");
    //   Serial.println(ticksPerInterval[i]);
  
    //   Serial.println(prev_tick[i]);
    // }

    float tick = ticksPerInterval[i] - prev_tick[i];
    // Serial.println(tick);
    // if (i == 0){
    //   Serial.println(tick);
    // }
    // every 34 ticks represents 1 
    tick = tick / (34.0);
    // if (i == 0){
    //   Serial.println(tick);
    // }
    Serial.print("tick: ");
    Serial.println(tick);
    // each tick is 30 degrees
    float rad = tick * 30 * PI / 180;

    Serial.print("rad: ");

    Serial.println(rad);
    // Serial.println(time_diff);

    float ang_vel = rad / (time_diff/1000.0);
    // if (i == 0){
    //   Serial.println("Ang Vel Calc");
    //   Serial.println(tick);
    //   Serial.println(rad);
    //   Serial.println(time_diff);

    //   Serial.println(ang_vel);

    // }
    
    Serial.print(ang_vel);
    Serial.print(" rad/s for motor ");
    Serial.println(i);
   
    prev_tick[i] = ticksPerInterval[i];
    currentWheelAngVel[i] = ang_vel;
  

  }
  
  float error[4];
  float acceleration[4];


  for (int i = 0; i < 4; i++) {
    error[i] =  abs(motorVelSetpoint[i]) - abs(currentWheelAngVel[i]);
    acceleration[i] = 1000 * (abs(currentWheelAngVel[i]) - abs(prev_tick_dot[i] / time_diff));
    Serial.print("D term: ");
    Serial.println(acceleration[i]*KD);
    Serial.print("P term: ");
    Serial.println(error[i]*kpMotor[i]);
    float u = kpMotor[i] * error[i] + KD* acceleration[i];
    // if (motorVelSetpoint[0] != motorVelSetpoint[2]) {
    //   // Rotating
    //   u = kpMotorAng[i] * error[i];


    // } else {
    //   // Linear
    //   u = kpMotor[i] * error[i];
    // }
    
    currentWheelPwm[i] = currentWheelPwm[i] + u;
    

    // Clamping
    if (currentWheelPwm[i] > 16383){
      currentWheelPwm[i] = 16383;
    } else if(currentWheelPwm[i] < 0) {
      currentWheelPwm[i] = 0;
    }
    Serial.print("U Control Signal: ");
    Serial.println(currentWheelPwm[i]);
    
    if (motorVelSetpoint[i] > 0) {
     
      digitalWrite(motorPINDIR[i], HIGH);
      motorDir[i] = 1;

     
      
    } else if (motorVelSetpoint[i] < 0) {
      
      digitalWrite(motorPINDIR[i], LOW);
      motorDir[i] = -1;

      
      
      // digitalWrite(motorPINDIR[i], LOW);
      // motorDir[i] = -1;
    }
    if (abs(lin_vel) < .01) {
      ledcWrite(motorPINEN[i], 2000);

    } else {
      if (abs(ang_vel) < .01) {
        ledcWrite(motorPINEN[i], currentWheelPwm[1]);
      } else {
        if ( i == 0 || i == 1) {
          ledcWrite(motorPINEN[i], currentWheelPwm[1]);
        } else if (i == 2 || i == 3) {
          ledcWrite(motorPINEN[i], currentWheelPwm[3]);
        }
      }
    
      

    }
    
    
    // ledcWrite(motorPINEN[i], currentWheelPwm[i]);
  }
  for (int i=0; i < 4; i++) {
    prev_tick_dot[i] = currentWheelAngVel[i];
  }
  previous_timestamp = current_timestamp;

}


void setup() {
  // put your setup code here, to run once:

  // timer configurations
  Serial.begin(115200);
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  // digitalWrite(SHT_LOX1, LOW);
  // digitalWrite(SHT_LOX2, LOW);
  // digitalWrite(SHT_LOX3, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Starting..."));
  setID();

  // setup encoder pins and interrupt
  //****************PIN Setup*************//
  for (int i = 0; i < 4; i++){
    pinMode(encoderPIN1[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPIN1[i]), i == 0 ? encoderISR0
    :(i == 1 ? encoderISR1 : (i == 2 ? encoderISR2 : encoderISR3)), CHANGE);
  }

  // // PWM EN Pins
  ledcAttach(motorPINEN[0], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttach(motorPINEN[1], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttach(motorPINEN[2], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttach(motorPINEN[3], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);

  // // DIR Pins
  pinMode(motorPINDIR[0], OUTPUT);
  pinMode(motorPINDIR[1], OUTPUT);
  pinMode(motorPINDIR[2], OUTPUT);
  pinMode(motorPINDIR[3], OUTPUT);
  // // //****************PIN Setup*************//


  // //****************WIFI*************//
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
  //****************WIFI*************//


}

//****************WIFI*************//
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
      motorControl[1] += 0.1;  // Decrementing by 0.25 - Left

      // motorControl[0] = 0;
      // motorControl[2] = 0;

    } else if (dir == 3) {
      motorControl[1] -= 0.1;  // Increment by 0.25 - Right
      // motorControl[0] = 0;
      // motorControl[2] = 0;
    }
    else if (dir == 1){
      motorControl[0] = 1; // Up
    }
    else if (dir == -1){
      motorControl[0] = -1; // Down
    }
    else if (dir == 0){
      motorControl[0] = 0; // Stop
      motorControl[1] = 0;
      // motorControl[2] = 0;


      
    }

  }
  server.send(204);
}
//****************WIFI*************//


void loop() {
  read_tof();
  float lin_vel = 0.0;
  float ang_vel = 0.0;
  // put your main code here, to run repeatedly:
  // Wifi Servr code to get Velocity command:
  // moveMotor(1, 1);

  //****************WIFI*************//
  server.handleClient();

  Serial.println("###############");
  Serial.println("User Input: ");
  for (int i = 0; i < 3; i++) {
    Serial.println(motorControl[i]);
  }
  Serial.println("\n");

  // Set the direction of each motor from the input

  lin_vel = motorControl[0] * motorControl[2]/100.0;
  ang_vel = motorControl[1];
  if (abs(motorControl[0]) < .01 && abs(motorControl[1]) < .01) {
   
    
    Serial.println("Received STOP");
    Serial.println(motorControl[1]);

    for (int i = 0; i < 4; i++) {
      ledcWrite(motorPINEN[i], 0);
      motorVelSetpoint[i] = 0.0;
    }
    return;

  }

  Serial.print("Linear Vel: ");
  Serial.println(lin_vel);
  Serial.print("Angular Vel: ");
  Serial.println(ang_vel);
  //****************WIFI*************//

  calcMotorVelSetpoint(lin_vel, ang_vel);
  controlMotor(lin_vel, ang_vel);
  delay(50);
}
