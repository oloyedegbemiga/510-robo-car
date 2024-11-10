#define EN_PIN 1
#define MOTOR_DIR1 4
#define MOTOR_DIR2 10


// Meters
#define HALF_WHEEL_BASE_B 0.13
#define WHEEL_RADIUS .04

hw_timer_t * timer = NULL;
bool ret = false;
bool ledsig = false;

const int encoderPIN1[4] = {18, 21, 25, 26}; // channel 1 for 4 encoders
// const int encoderPIN2[4] = {19, 22, 27, 14}; // channel 2 for 4 encoders
const int motorPIN1[4] = {4, 10, 19, 0}; // channel 2 for motor pins
// const int motorPIN2[4] = {19, 22, 27, 14}; // channel 2 for 4 encoders

volatile long ticksPerInterval[4] = {0, 0, 0, 0};
// volatile long positionCounts[4] = {0, 0, 0, 0};
volatile long motorDir[4] = {1, 1, 1, 1};

// Rad/seconds
volatile float motorVelSetpoint[4] = {0.0, 0.0, 0.0, 0.0};

int CPR = 12;
float rotPerTick = 360.0 / CPR;
const int timer_interval = 100;

// after 
int initializeControl = 0;


void IRAM_ATTR encoderISR0() { ticksPerInterval[0] += motorDir[0];}
void IRAM_ATTR encoderISR1() { ticksPerInterval[1] += motorDir[1];}
void IRAM_ATTR encoderISR2() { ticksPerInterval[2] += motorDir[2];}
void IRAM_ATTR encoderISR3() { ticksPerInterval[3] += motorDir[3];}


void IRAM_ATTR onTimer(){

  for (int i = 0; i < 4; i++){
    long ticks = ticksPerInterval[i];
    ticksPerInterval[i] = 0;
    float rpm = (ticks / (float) CPR) * (60000.0 / timer_interval);
    // float motorPosDeg = positionCounts[i] * rotPerTick;
    float motorPosDeg = ticks * rotPerTick;
  }

}

void moveMotor(int motor_pin, int dir){

  if (dir == 1){
    // move forward
    digitalWrite(motor_pin, HIGH);
    digitalWrite(motor_pin, LOW);    
  } else if (dir == -1){
    // move backward
    digitalWrite(motor_pin, LOW);
    digitalWrite(motor_pin, HIGH);
  } else {
    // stop
    digitalWrite(motor_pin, LOW);
    digitalWrite(motor_pin, LOW); 
  }

}

void calcMotorVelSetpoint(float lin_vel, float ang_vel) {
  // we want to return an array that has each motor velocity

  float ang_v_left = (lin_vel - HALF_WHEEL_BASE_B * ang_vel) / WHEEL_RADIUS;
  float ang_v_right = (lin_vel + HALF_WHEEL_BASE_B * ang_vel) / WHEEL_RADIUS;

  motorVelSetpoint[0] = ang_v_right;
  motorVelSetpoint[1] = ang_v_right;
  motorVelSetpoint[2] = ang_v_left;
  motorVelSetpoint[3] = ang_v_left;
}

void motorControl() {

  static unsigned long current_timestamp = millis();
  if (initializeControl == 0) {
    static unsigned long previous_timestamp = current_timestamp;
    initializeControl = 1;
    return;
  }

  unsigned long time_diff = current_timestamp - previous_timestamp;

  int u[4];
  float current_wheel_vel[4];
  static int ticks_between_control[4];
  
  for (int i = 0; i < 4; i++) {
    ticks_between_control[i] = ticksPerInterval[i] - ticks_between_control[i];
    int8_t tick = ticks_between_control[i] / 12;


  }

  previous_timestamp = current_timestamp;

}


void setup() {
  // put your setup code here, to run once:

  // timer configurations
  Serial.begin(115200);
  timer = timerBegin(1000000); // set freq to 1us
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100000, true, 0); // 100000 ticks per cycle

  // setup encoder pins and interrupt
  for (int i = 0; i < 4; i++){
    pinMode(encoderPIN1[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPIN1[0]), i == 0 ? encoderISR0
    :(i == 1 ? encoderISR1 : (i == 2 ? encoderISR2 : encoderISR3)), CHANGE);
  }

  // setup motors
  for (int motor_num = 0; motor_num < 4; motor_num++){
    pinMode(motorPIN1[motor_num], OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // Wifi Servr code to get Velocity command:

  float lin_vel = 1.0;
  float ang_vel = 0.0;

  calcMotorVelSetpoint(lin_vel, ang_vel);
  motorControl();

}
