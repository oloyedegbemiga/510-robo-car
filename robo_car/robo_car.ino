#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "index.h"
#include "html510.h"
// HTML510Server h(80);

// #define EN_PIN 15
// #define MOTOR_DIR1 23
// #define MOTOR_DIR2 22

#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 10

// Meters
#define HALF_WHEEL_BASE_B 0.13
#define WHEEL_RADIUS .04
#define PI 3.14159265358979323846

#define KP 8
//*******WIFI***********//
// WiFi Credentials
const char *ssid = "Robocar";
const char *password = "12345678";
// Web server on port 80
WebServer server(80);

// Motor control values array
float motorControl[3] = {0, 0.0, 0}; 


//*******WIFI***********//

hw_timer_t * timer = NULL;
bool ret = false;
bool ledsig = false;

const int encoderPIN1[4] = {23, 22, 1, 3}; // channel 1 for 4 encoders
const int motorPINEN[4] = {4, 0, 2, 15}; // motor EN PWM pins
const int motorPINDIR[4] = {19, 18, 5, 17}; // motor DIR pins

volatile long ticksPerInterval[4] = {0, 0, 0, 0};

// We dictate motorDir after we set the directions on the motor directly
// To ensure the ISR will update correctly
volatile long motorDir[4] = {1, 1, 1, 1};

// Rad/seconds
volatile float motorVelSetpoint[4] = {0.0, 0.0, 0.0, 0.0};

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

void controlMotor() {
  
  static unsigned long previous_timestamp;
  unsigned long current_timestamp = millis();
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

    tick = tick / (34.0);
    // if (i == 0){
    //   Serial.println(tick);
    // }
    // Serial.println(tick);
    // each tick is 30 degrees
    float rad = tick * 30 * PI / 180;
   
    // Serial.println(rad);
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

  for (int i = 0; i < 4; i++) {
    error[i] =  motorVelSetpoint[i] - currentWheelAngVel[i];
    float u = KP * error[i];
    Serial.print("U Control Signal: ");
    Serial.println(u);
    currentWheelPwm[i] = currentWheelPwm[i] + u;

    // Clamping
    if (currentWheelPwm[i] > 16383){
      currentWheelPwm[i] = 16383;
    } else if(currentWheelPwm[i] < 0) {
      currentWheelPwm[i] = 0;
    }
    
    if (motorVelSetpoint[i] > 0) {
      digitalWrite(motorPINDIR[i], HIGH);
      motorDir[i] = 1;
    } else if (motorVelSetpoint[i] < 0) {
      digitalWrite(motorPINDIR[i], LOW);
      motorDir[i] = -1;
    }

    ledcWrite(motorPINEN[i], currentWheelPwm[i]);
  }

  previous_timestamp = current_timestamp;

}


void setup() {
  // put your setup code here, to run once:

  // timer configurations
  Serial.begin(115200);

  // setup encoder pins and interrupt
  //****************PIN Setup*************//
  for (int i = 0; i < 4; i++){
    pinMode(encoderPIN1[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPIN1[i]), i == 0 ? encoderISR0
    :(i == 1 ? encoderISR1 : (i == 2 ? encoderISR2 : encoderISR3)), CHANGE);
  }

  // PWM EN Pins
  ledcAttach(motorPINEN[0], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttach(motorPINEN[1], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttach(motorPINEN[2], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcAttach(motorPINEN[3], LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);

  // DIR Pins
  pinMode(motorPINDIR[0], OUTPUT);
  pinMode(motorPINDIR[1], OUTPUT);
  pinMode(motorPINDIR[2], OUTPUT);
  pinMode(motorPINDIR[3], OUTPUT);
  //****************PIN Setup*************//


  //****************WIFI*************//
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
      motorControl[1] = 0;
      
    }

  }
  server.send(204);
}
//****************WIFI*************//


void loop() {

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
  if (abs(motorControl[0]) < .01) {
    Serial.print("Received STOP");
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
  controlMotor();
  delay(50);
}
