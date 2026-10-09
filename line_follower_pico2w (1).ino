// ================= LINE FOLLOWER — Pico 2W version =================
// Hardware: Raspberry Pi Pico 2W (Arduino-Pico core), 16-channel
// Robojunkies digital IR sensor array, 2x TB6612FNG motor drivers
// (1 driver per motor), 300 RPM motors.
//
// NOTE: Pico W reserves GPIO 23, 24, 25, 29 for the onboard WiFi chip.
// Do NOT use those pins for anything in this sketch.
// =====================================================================

// ------ SENSOR PINS (16 digital sensors) ------
// Wire sensor 0 = leftmost, sensor 15 = rightmost
const int sensorPins[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const int numSensors = 16;

// If your sensors read LOW when over the line (common with open-collector
// comparator boards), set this to false.
#define SENSOR_ACTIVE_HIGH true

// ------ MOTOR DRIVER PINS (TB6612, one driver per motor) ------
// Left motor driver
#define AIN1 16
#define AIN2 17
#define PWMA 18   // must be a PWM-capable pin (all RP2040 GPIOs are)
#define STBY_A 19 // standby pin for left driver — must be HIGH to enable

// Right motor driver
#define BIN1 20
#define BIN2 21
#define PWMB 22
#define STBY_B 26 // standby pin for right driver — must be HIGH to enable

// ------ SENSOR WEIGHTS (symmetric, no center sensor since count is even) ------
int sensorWeight[16] = {8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8};

int sensorValue[16];   // 0 or 1 per sensor after active-state correction
int onLine = 0;

// ------ PID VARIABLES ------
// NOTE: these gains were tuned for the OLD analog sensor (0-1000 range).
// With digital 0/1 sensors the error range is much smaller
// (max |error| ~= 8 here vs ~1000 before), so you will very likely need
// to raise Kp and Kd substantially. Start retuning from scratch.
float Kp = 15.0;
float Ki = 0.0;
float Kd = 20.0;

float P, I, D;
float lastError = 0;
float PIDvalue;
float error = 0;

// ------ SPEED ------
int maxSpeed = 130;      // cap on analogWrite (0-255 scale)
int baseSpeed = 130;      // straight-line cruise speed, tune for your 300 RPM motors

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY_A, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY_B, OUTPUT);

  digitalWrite(STBY_A, HIGH); // enable left driver
  digitalWrite(STBY_B, HIGH); // enable right driver

  pinMode(LED_BUILTIN, OUTPUT); // Pico W onboard LED (WiFi-chip controlled, works fine as LED_BUILTIN)

  delay(1000); // brief pause before starting, no calibration needed for digital sensors
}

void loop() {
  readLine();

  if (onLine) {
    lineFollow();
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    // Lost line recovery: small corrective turn based on last known error
    if (error > 0) {
      motor1run(-80);
      motor2run(100);
    } else {
      motor1run(100);
      motor2run(-80);
    }
  }
}

// ------ SENSOR READING ------
void readLine() {
  onLine = 0;
  for (int i = 0; i < numSensors; i++) {
    int raw = digitalRead(sensorPins[i]);
    sensorValue[i] = SENSOR_ACTIVE_HIGH ? raw : !raw;
    if (sensorValue[i]) onLine = 1;
  }
}

// ------ LINE FOLLOWING PID ------
void lineFollow() {
  error = 0;
  int activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    error += sensorWeight[i] * sensorValue[i];
    activeSensors += sensorValue[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  P = error;
  I += error;
  D = error - lastError;
  PIDvalue = Kp * P + Ki * I + Kd * D;
  lastError = error;

  int lsp = baseSpeed - PIDvalue;
  int rsp = baseSpeed + PIDvalue;

  lsp = constrain(lsp, 0, maxSpeed);
  rsp = constrain(rsp, 0, maxSpeed);

  motor1run(lsp);
  motor2run(rsp);
}

// ------ MOTOR CONTROL ------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, abs(motorSpeed));
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, abs(motorSpeed));
}
