#include <Arduino.h>

#define L_MOTOR_1 7
#define L_MOTOR_2 8
#define L_MOTOR_PWM 9

#define R_MOTOR_1 11
#define R_MOTOR_2 12
#define R_MOTOR_PWM 10

#define MUX_A A3
#define MUX_B A4
#define MUX_C A5
#define MUX_OUT A6

const float Kp = 0.045;
const float Ki = 0.0008;
const float Kd = 0.65;

const float DERIV_ALPHA = 0.22;

const int MIN_SPEED = 125;
const int BASE_SPEED = 225;
const int MAX_SPEED = 255;
const int STRAIGHT_BOOST = 20;
const int ACCEL_LIMIT = 12;

const float SMOOTH_ALPHA = 0.35;

const int THRESH_HIGH[8] = {
  760, 660, 660, 640,
  640, 660, 660, 760
};

const int THRESH_LOW[8] = {
  720, 620, 620, 610,
  610, 620, 620, 720
};

const int SUSTAIN_REQUIRED = 3;
const unsigned long MUX_SETTLE_US = 50;

const unsigned long MEMORY_HOLD_MS = 350;
const unsigned long SEARCH_ARM_MS = 90;

const int SEARCH_NEAR_SPEED = 105;
const int SEARCH_FAR_SPEED = 155;

const unsigned long SEARCH_SWEEP_MS = 350;

const int TURN_POWER = 205;
const unsigned long TURN_TIMEOUT = 850;
const unsigned long TURN_LOCKOUT_MS = 180;

int rawSensor[8];
float smoothSensor[8];
bool sensorState[8];
int sustainCounter[8];

long lastKnownError = 0;
unsigned long lastSeenTime = 0;

float integral = 0;
float lastError = 0;
float derivativeFiltered = 0;

unsigned long lastPIDTime = 0;

int currentLeftPWM = 0;
int currentRightPWM = 0;

enum Mode {
  MODE_FOLLOW,
  MODE_SEARCH,
  MODE_PRIORITY_TURN
};

Mode mode = MODE_FOLLOW;

unsigned long searchStart = 0;
unsigned long priorityStart = 0;
unsigned long lastPriorityTurn = 0;

int priorityDirection = 0;

void setMotorRaw(int pinA, int pinB, int pwmPin, int speed) {

  speed = constrain(speed, -255, 255);

  if (speed >= 0) {

    if (pinA == L_MOTOR_1)
      PORTD |= (1 << PD7);

    if (pinB == L_MOTOR_2)
      PORTB &= ~(1 << PB0);

    if (pinA == R_MOTOR_1)
      PORTB |= (1 << PB3);

    if (pinB == R_MOTOR_2)
      PORTB &= ~(1 << PB4);

    analogWrite(pwmPin, speed);

  } else {

    if (pinA == L_MOTOR_1)
      PORTD &= ~(1 << PD7);

    if (pinB == L_MOTOR_2)
      PORTB |= (1 << PB0);

    if (pinA == R_MOTOR_1)
      PORTB &= ~(1 << PB3);

    if (pinB == R_MOTOR_2)
      PORTB |= (1 << PB4);

    analogWrite(pwmPin, -speed);
  }
}

int rampPWM(int current, int target) {

  if (target > current + ACCEL_LIMIT)
    return current + ACCEL_LIMIT;

  if (target < current - ACCEL_LIMIT)
    return current - ACCEL_LIMIT;

  return target;
}

void drive(int leftTarget, int rightTarget) {

  leftTarget = constrain(leftTarget, -MAX_SPEED, MAX_SPEED);
  rightTarget = constrain(rightTarget, -MAX_SPEED, MAX_SPEED);

  currentLeftPWM = rampPWM(currentLeftPWM, leftTarget);
  currentRightPWM = rampPWM(currentRightPWM, rightTarget);

  setMotorRaw(
    L_MOTOR_1,
    L_MOTOR_2,
    L_MOTOR_PWM,
    currentLeftPWM
  );

  setMotorRaw(
    R_MOTOR_1,
    R_MOTOR_2,
    R_MOTOR_PWM,
    currentRightPWM
  );
}

void stopMotors() {

  currentLeftPWM = 0;
  currentRightPWM = 0;

  setMotorRaw(
    L_MOTOR_1,
    L_MOTOR_2,
    L_MOTOR_PWM,
    0
  );

  setMotorRaw(
    R_MOTOR_1,
    R_MOTOR_2,
    R_MOTOR_PWM,
    0
  );
}

int readMux(int channel) {

  if (channel & 0x01)
    PORTC |= (1 << PC3);
  else
    PORTC &= ~(1 << PC3);

  if (channel & 0x02)
    PORTC |= (1 << PC4);
  else
    PORTC &= ~(1 << PC4);

  if (channel & 0x04)
    PORTC |= (1 << PC5);
  else
    PORTC &= ~(1 << PC5);

  delayMicroseconds(MUX_SETTLE_US);

  return analogRead(MUX_OUT);
}

void initSensors() {

  for (int i = 0; i < 8; i++) {

    rawSensor[i] = readMux(i);
    smoothSensor[i] = rawSensor[i];
    sensorState[i] = false;
    sustainCounter[i] = 0;
  }

  lastSeenTime = millis();
}

void updateSensors() {

  for (int i = 0; i < 8; i++) {

    rawSensor[i] = readMux(i);

    smoothSensor[i] +=
      SMOOTH_ALPHA *
      (rawSensor[i] - smoothSensor[i]);

    bool blackDetected;

    if (sensorState[i])
      blackDetected = smoothSensor[i] < THRESH_HIGH[i];
    else
      blackDetected = smoothSensor[i] < THRESH_LOW[i];

    if (blackDetected) {

      sustainCounter[i]++;

      if (sustainCounter[i] >= SUSTAIN_REQUIRED) {

        sensorState[i] = true;
        sustainCounter[i] = SUSTAIN_REQUIRED;
      }

    } else {

      sustainCounter[i]--;

      if (sustainCounter[i] <= 0) {

        sensorState[i] = false;
        sustainCounter[i] = 0;
      }
    }
  }
}

long computePosition() {

  const long weights[8] = {
    -8000,
    -4000,
    -2000,
    -1000,
     1000,
     2000,
     4000,
     8000
  };

  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < 8; i++) {

    float strength =
      (float)THRESH_HIGH[i] -
      smoothSensor[i];

    if (strength < 0)
      strength = 0;

    if (strength > 400)
      strength = 400;

    numerator +=
      (long)(strength * weights[i]);

    denominator +=
      (long)strength;
  }

  if (denominator == 0)
    return 0;

  return numerator / denominator;
}

int getLineCount() {

  int count = 0;

  for (int i = 0; i < 8; i++) {

    if (sensorState[i])
      count++;
  }

  return count;
}

bool centerDetected() {

  return sensorState[3] ||
         sensorState[4];
}

void enterFollow() {

  mode = MODE_FOLLOW;

  integral = constrain(
    integral,
    -12000,
    12000
  );

  searchStart = 0;
}

void enterSearch() {

  if (mode != MODE_SEARCH) {

    mode = MODE_SEARCH;
    searchStart = millis();
    integral = 0;
    derivativeFiltered = 0;
  }
}

void enterPriorityTurn(int direction) {

  mode = MODE_PRIORITY_TURN;

  priorityDirection = direction;

  priorityStart = millis();

  lastPriorityTurn = millis();

  integral = 0;
  derivativeFiltered = 0;
}

bool detectLeftTurn() {

  return
    sensorState[0] &&
    !sensorState[1] &&
    !sensorState[2] &&
    !sensorState[3];
}

bool detectRightTurn() {

  return
    sensorState[7] &&
    !sensorState[6] &&
    !sensorState[5] &&
    !sensorState[4];
}

void setup() {

  DDRD |= (1 << PD7);

  DDRB |=
    (1 << PB0) |
    (1 << PB3) |
    (1 << PB4);

  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_PWM, OUTPUT);

  DDRC |=
    (1 << PC3) |
    (1 << PC4) |
    (1 << PC5);

  pinMode(MUX_OUT, INPUT);

  Serial.begin(115200);

  initSensors();

  lastPIDTime = micros();
}

void runFollow(long position) {

  unsigned long nowMicros = micros();

  float dt =
    (nowMicros - lastPIDTime) /
    1000000.0;

  lastPIDTime = nowMicros;

  dt = constrain(dt, 0.0005, 0.05);

  float error = (float)position;

  if (abs(error) < 4000) {

    integral += error * dt;

    integral =
      constrain(
        integral,
        -12000,
        12000
      );
  }

  float derivative =
    (error - lastError) / dt;

  derivativeFiltered +=
    DERIV_ALPHA *
    (derivative - derivativeFiltered);

  float correction =
    Kp * error +
    Ki * integral +
    Kd * derivativeFiltered;

  correction =
    constrain(
      correction,
      -255,
      255
    );

  lastError = error;

  int absPosition =
    abs((int)position);

  int targetSpeed;

  if (absPosition < 700) {

    targetSpeed =
      BASE_SPEED +
      STRAIGHT_BOOST;

  } else if (absPosition < 1600) {

    targetSpeed =
      BASE_SPEED;

  } else if (absPosition < 3000) {

    targetSpeed =
      190;

  } else if (absPosition < 5000) {

    targetSpeed =
      155;

  } else {

    targetSpeed =
      MIN_SPEED;
  }

  targetSpeed =
    constrain(
      targetSpeed,
      MIN_SPEED,
      MAX_SPEED
    );

  int left =
    targetSpeed -
    (int)correction;

  int right =
    targetSpeed +
    (int)correction;

  left =
    constrain(
      left,
      -MAX_SPEED,
      MAX_SPEED
    );

  right =
    constrain(
      right,
      -MAX_SPEED,
      MAX_SPEED
    );

  drive(left, right);
}

void runSearch() {

  unsigned long elapsed =
    millis() - searchStart;

  int direction =
    (lastKnownError >= 0)
      ? 1
      : -1;

  if (millis() - lastSeenTime <
      MEMORY_HOLD_MS) {

    if (direction > 0) {

      drive(
        SEARCH_NEAR_SPEED,
        -SEARCH_NEAR_SPEED
      );

    } else {

      drive(
        -SEARCH_NEAR_SPEED,
        SEARCH_NEAR_SPEED
      );
    }

    return;
  }

  if ((elapsed / SEARCH_SWEEP_MS) % 2 == 0) {

    drive(
      SEARCH_FAR_SPEED,
      -SEARCH_FAR_SPEED
    );

  } else {

    drive(
      -SEARCH_FAR_SPEED,
      SEARCH_FAR_SPEED
    );
  }
}

void runPriorityTurn() {

  unsigned long elapsed =
    millis() - priorityStart;

  if (elapsed > TURN_TIMEOUT) {

    enterSearch();

    return;
  }

  if (priorityDirection < 0) {

    drive(
      -TURN_POWER,
       TURN_POWER
    );

  } else {

    drive(
       TURN_POWER,
      -TURN_POWER
    );
  }

  if (
    centerDetected() &&
    elapsed > 70
  ) {

    enterFollow();
  }
}

void loop() {

  updateSensors();

  int onCount =
    getLineCount();

  bool lineDetected =
    onCount > 0;

  unsigned long now =
    millis();

  long position =
    computePosition();

  if (lineDetected) {

    lastKnownError =
      position;

    lastSeenTime =
      now;
  }

  bool lineLost =
    !lineDetected;

  if (lineLost) {

    if (
      now - lastSeenTime >
      SEARCH_ARM_MS
    ) {

      enterSearch();
    }
  }

  if (
    mode == MODE_FOLLOW &&
    !lineLost &&
    onCount <= 2 &&
    now - lastPriorityTurn >
    TURN_LOCKOUT_MS
  ) {

    if (detectLeftTurn()) {

      enterPriorityTurn(-1);

    } else if (detectRightTurn()) {

      enterPriorityTurn(+1);
    }
  }

  switch (mode) {

    case MODE_FOLLOW:

      if (lineDetected)
        runFollow(position);
      else
        enterSearch();

      break;

    case MODE_SEARCH:

      if (lineDetected) {

        enterFollow();
        runFollow(position);

      } else {

        runSearch();
      }

      break;

    case MODE_PRIORITY_TURN:

      runPriorityTurn();

      break;
  }

  Serial.print("mode:");
  Serial.print((int)mode);

  Serial.print(" pos:");
  Serial.print(lastKnownError);

  Serial.print(" lost:");
  Serial.print(lineLost);

  Serial.print(" count:");
  Serial.print(onCount);

  Serial.print(" s:");

  for (int i = 0; i < 8; i++)
    Serial.print(sensorState[i] ? '1' : '0');

  Serial.print(" raw:");

  for (int i = 0; i < 8; i++) {

    Serial.print(
      (int)smoothSensor[i]
    );

    Serial.print(' ');
  }

  Serial.println();

  delay(2);
}
