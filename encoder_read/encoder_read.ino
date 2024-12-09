#define ENCODER_PIN1 4
// #define ENCODER_PIN2 19

#define EN_PIN 15
#define MOTOR_DIR1 22
#define MOTOR_DIR2 23

#define LEDC_RESOLUTION_BITS 14
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQ_HZ 10

hw_timer_t * timer = NULL;

// volatile uint32_t encoderTimer = 0;

// volatile uint8_t prevEncoderVal = 0;


volatile long encoderTimer = 0;

volatile long prevEncoderVal = 0;


volatile uint8_t currDir = 0;

bool ret = false;
bool ledsig = false;

uint8_t tempVal = 0;

void IRAM_ATTR onTimer(){

  tempVal = digitalRead(ENCODER_PIN1);
  // Serial.println("tempVal");
  // Serial.println(tempVal);
  // Serial.println("prevEncoderVal");
  // Serial.println(prevEncoderVal);
  if (tempVal != prevEncoderVal){
    if (currDir == 1 ){
      encoderTimer++;
    } else{
      encoderTimer--;
    }
    prevEncoderVal = tempVal;
    // Serial.println(encoderTimer);
    uint8_t gear_norm = encoderTimer / 34;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  timer = timerBegin(1000000);
  // Serial.println("Before timer");
  // timer = timerBegin(100000, 0, 80, true);
  // timer = timerBegin(100000);
  // Serial.println("After timer");
  // delay(2000);
  ledcAttach(EN_PIN, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  // Serial.println("After ledc");


  // pinMode(LED_PINR, OUTPUT);
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  digitalWrite(MOTOR_DIR1, LOW);
  digitalWrite(MOTOR_DIR2, LOW);

  pinMode(ENCODER_PIN1, INPUT);
  timerAttachInterrupt(timer, &onTimer);

  timerAlarm(timer, 500, true, 0);
  // timerAlarm(timer, 500, true, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  tempVal = digitalRead(ENCODER_PIN1);


  // Serial.println(encoderTimer);

  digitalWrite(MOTOR_DIR1, LOW);
  digitalWrite(MOTOR_DIR2, HIGH);
  currDir = 1;


  // // ledsig = ledcWrite(EN_PIN, 0);
  // // delay(4500);
  // ledsig = ledcWrite(EN_PIN, 16000);
  // delay(4500);
  // Serial.println(encoderTimer);
  // Serial.println(encoderTimer/34);


  // // ledsig = ledcWrite(EN_PIN, 16000);

  Serial.println("Stop");


  digitalWrite(MOTOR_DIR1, LOW);
  digitalWrite(MOTOR_DIR2, LOW);
  delay(1500);
  Serial.println(encoderTimer);
  Serial.println(encoderTimer/34);


  // delay(5000);

  digitalWrite(MOTOR_DIR1, HIGH);
  digitalWrite(MOTOR_DIR2, LOW);
  currDir = 0;

  // ledsig = ledcWrite(EN_PIN, 0);
  // delay(4500);
  ledsig = ledcWrite(EN_PIN, 16000);
  delay(4500);

  // Serial.println("tempVal");
  // Serial.println(tempVal);
  // Serial.println("prevEncoderVal");
  // Serial.println(prevEncoderVal);
  Serial.println(encoderTimer);
  Serial.println(encoderTimer/34);


  // ledsig = ledcWrite(EN_PIN, 16000);

  Serial.println("Forward");
}
