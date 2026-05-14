#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

// === PINS MOTEURS ===
#define MOTOR1_PWM 37
#define MOTOR1_DIR 39
#define MOTOR2_PWM 13
#define MOTOR2_DIR 41
#define MOTOR3_PWM 14
#define MOTOR3_DIR 15
#define MOTOR4_PWM 36
#define MOTOR4_DIR 38

#define MOTOR1_INVERTED true
#define MOTOR2_INVERTED false
#define MOTOR3_INVERTED false
#define MOTOR4_INVERTED true

// === UART ===
#define UART_BAUD_RATE 115200

// === OTOS mapping ===
#define OTOS_SWAP_XY true
#define OTOS_X_SIGN 1.0f
#define OTOS_Y_SIGN 1.0f
#define OTOS_H_SIGN 1.0f
#define OTOS_H_OFFSET_DEG 0.0f

// === PID / contrôle ===
#define CONTROL_DT 0.02f
#define KP_X 34.0f
#define KI_X 0.8f
#define KD_X 3.0f
#define KP_Y 34.0f
#define KI_Y 0.8f
#define KD_Y 3.0f
#define KP_H 3.0f
#define KI_H 0.04f
#define KD_H 0.15f
#define Y_CONTROL_SIGN -1.0f

#define POSITION_DEADBAND 0.25f
#define HEADING_DEADBAND_DEG 1.5f
#define INTEGRAL_LIMIT 20.0f
#define INTEGRAL_H_LIMIT 80.0f
#define MAX_COMMAND 170.0f
#define MAX_ANGULAR_COMMAND 70.0f
#define MIN_ACTIVE_COMMAND 55.0f
#define MIN_ANGULAR_ACTIVE_COMMAND 45.0f
#define MAX_COMMAND_SLEW 700.0f
#define MAX_VELOCITY 255.0f

struct Position
{
  float x;
  float y;
  float heading;
};

QwiicOTOS myOtos;

Position currentPos = {0.0f, 0.0f, 0.0f};
Position targetPos = {0.0f, 0.0f, 0.0f};
bool maintainPosition = false;
bool useHeadingControl = true;

float errorX_prev = 0.0f, errorY_prev = 0.0f, errorH_prev = 0.0f;
float integralX = 0.0f, integralY = 0.0f, integralH = 0.0f;
float commandX_prev = 0.0f, commandY_prev = 0.0f, commandW_prev = 0.0f;

bool headingReferenceInitialized = false;
float headingReferenceDeg = 0.0f;
String uartRxLine = "";

static float normalizeAngleDeg(float angleDeg)
{
  while (angleDeg > 180.0f)
    angleDeg -= 360.0f;
  while (angleDeg <= -180.0f)
    angleDeg += 360.0f;
  return angleDeg;
}

void resetControllerState()
{
  integralX = 0.0f;
  integralY = 0.0f;
  integralH = 0.0f;
  errorX_prev = 0.0f;
  errorY_prev = 0.0f;
  errorH_prev = 0.0f;
  commandX_prev = 0.0f;
  commandY_prev = 0.0f;
  commandW_prev = 0.0f;
}

void setMotor(int motorNum, float speed)
{
  speed = constrain(speed, -255, 255);

  bool inverted = false;
  int pwmPin = -1, dirPin = -1;

  switch (motorNum)
  {
  case 1:
    pwmPin = MOTOR1_PWM;
    dirPin = MOTOR1_DIR;
    inverted = MOTOR1_INVERTED;
    break;
  case 2:
    pwmPin = MOTOR2_PWM;
    dirPin = MOTOR2_DIR;
    inverted = MOTOR2_INVERTED;
    break;
  case 3:
    pwmPin = MOTOR3_PWM;
    dirPin = MOTOR3_DIR;
    inverted = MOTOR3_INVERTED;
    break;
  case 4:
    pwmPin = MOTOR4_PWM;
    dirPin = MOTOR4_DIR;
    inverted = MOTOR4_INVERTED;
    break;
  default:
    return;
  }

  if (inverted)
  {
    speed = -speed;
  }

  int pwmValue = abs(speed);
  digitalWrite(dirPin, speed < 0 ? LOW : HIGH);
  analogWrite(pwmPin, pwmValue);
}

void stopAllMotors()
{
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
  analogWrite(MOTOR3_PWM, 0);
  analogWrite(MOTOR4_PWM, 0);
}

void setHolonomicVelocity(float velX, float velY, float velW)
{
  float m1 = velX + velY + velW;
  float m2 = velX - velY - velW;
  float m3 = velX + velY - velW;
  float m4 = velX - velY + velW;

  float maxSpeed = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));
  if (maxSpeed > MAX_VELOCITY)
  {
    m1 = (m1 / maxSpeed) * MAX_VELOCITY;
    m2 = (m2 / maxSpeed) * MAX_VELOCITY;
    m3 = (m3 / maxSpeed) * MAX_VELOCITY;
    m4 = (m4 / maxSpeed) * MAX_VELOCITY;
  }

  setMotor(1, m1);
  setMotor(2, m2);
  setMotor(3, m3);
  setMotor(4, m4);
}

void updateCurrentPosFromOtos(const sfe_otos_pose2d_t &otos_data)
{
  float mappedX = otos_data.x;
  float mappedY = otos_data.y;

  if (OTOS_SWAP_XY)
  {
    float tmp = mappedX;
    mappedX = mappedY;
    mappedY = tmp;
  }

  currentPos.x = mappedX * OTOS_X_SIGN;
  currentPos.y = mappedY * OTOS_Y_SIGN;

  if (!headingReferenceInitialized)
  {
    headingReferenceDeg = otos_data.h;
    headingReferenceInitialized = true;
  }

  float headingRelativeDeg = (otos_data.h - headingReferenceDeg) * OTOS_H_SIGN;
  currentPos.heading = normalizeAngleDeg(headingRelativeDeg + OTOS_H_OFFSET_DEG);
}

bool parseTwoFloatArgs(const String &args, float &first, float &second)
{
  int commaIndex = args.indexOf(',');
  if (commaIndex <= 0 || commaIndex >= (args.length() - 1))
  {
    return false;
  }

  String left = args.substring(0, commaIndex);
  String right = args.substring(commaIndex + 1);
  left.trim();
  right.trim();

  if (left.length() == 0 || right.length() == 0)
  {
    return false;
  }

  first = left.toFloat();
  second = right.toFloat();
  return true;
}

bool parseSingleFloatArg(const String &args, float &value)
{
  String trimmed = args;
  trimmed.trim();
  if (trimmed.length() == 0)
  {
    return false;
  }

  value = trimmed.toFloat();
  return true;
}

void holdPosition()
{
  float errorX = targetPos.x - currentPos.x;
  float errorY = (targetPos.y - currentPos.y) * Y_CONTROL_SIGN;
  float errorH = 0.0f;

  if (useHeadingControl)
  {
    errorH = normalizeAngleDeg(targetPos.heading - currentPos.heading);
  }

  if (abs(errorX) < POSITION_DEADBAND)
    errorX = 0.0f;
  if (abs(errorY) < POSITION_DEADBAND)
    errorY = 0.0f;
  if (abs(errorH) < HEADING_DEADBAND_DEG)
    errorH = 0.0f;

  if (errorX == 0.0f && errorY == 0.0f && errorH == 0.0f)
  {
    resetControllerState();
    stopAllMotors();
    return;
  }

  integralX += errorX * CONTROL_DT;
  integralY += errorY * CONTROL_DT;
  integralH += errorH * CONTROL_DT;
  integralX = constrain(integralX, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  integralY = constrain(integralY, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  integralH = constrain(integralH, -INTEGRAL_H_LIMIT, INTEGRAL_H_LIMIT);

  float velX = (KP_X * errorX) + (KI_X * integralX) + (KD_X * (errorX - errorX_prev) / CONTROL_DT);
  float velY = (KP_Y * errorY) + (KI_Y * integralY) + (KD_Y * (errorY - errorY_prev) / CONTROL_DT);
  float velW = (KP_H * errorH) + (KI_H * integralH) + (KD_H * (errorH - errorH_prev) / CONTROL_DT);

  errorX_prev = errorX;
  errorY_prev = errorY;
  errorH_prev = errorH;

  velX = constrain(velX, -MAX_COMMAND, MAX_COMMAND);
  velY = constrain(velY, -MAX_COMMAND, MAX_COMMAND);
  velW = constrain(velW, -MAX_ANGULAR_COMMAND, MAX_ANGULAR_COMMAND);

  if (errorX != 0.0f && abs(velX) < MIN_ACTIVE_COMMAND)
    velX = (velX >= 0.0f ? 1.0f : -1.0f) * MIN_ACTIVE_COMMAND;
  if (errorY != 0.0f && abs(velY) < MIN_ACTIVE_COMMAND)
    velY = (velY >= 0.0f ? 1.0f : -1.0f) * MIN_ACTIVE_COMMAND;
  if (errorH != 0.0f && abs(velW) < MIN_ANGULAR_ACTIVE_COMMAND)
    velW = (velW >= 0.0f ? 1.0f : -1.0f) * MIN_ANGULAR_ACTIVE_COMMAND;

  float maxStep = MAX_COMMAND_SLEW * CONTROL_DT;
  velX = commandX_prev + constrain(velX - commandX_prev, -maxStep, maxStep);
  velY = commandY_prev + constrain(velY - commandY_prev, -maxStep, maxStep);
  velW = commandW_prev + constrain(velW - commandW_prev, -maxStep, maxStep);
  commandX_prev = velX;
  commandY_prev = velY;
  commandW_prev = velW;

  float headingRad = currentPos.heading * DEG_TO_RAD;
  float cmdRobotX = (cosf(headingRad) * velX) + (sinf(headingRad) * velY);
  float cmdRobotY = (-sinf(headingRad) * velX) + (cosf(headingRad) * velY);

  setHolonomicVelocity(cmdRobotX, cmdRobotY, velW);
}

void handleUartLineCommand(const String &line)
{
  String cmdLine = line;
  cmdLine.trim();
  if (cmdLine.length() == 0)
  {
    return;
  }

  char cmd = static_cast<char>(tolower(cmdLine.charAt(0)));
  String args = cmdLine.substring(1);
  args.trim();

  float valueA = 0.0f;
  float valueB = 0.0f;

  switch (cmd)
  {
  case 'm':
    targetPos = currentPos;
    maintainPosition = true;
    resetControllerState();
    Serial1.println("OK m");
    return;

  case 's':
    maintainPosition = false;
    resetControllerState();
    stopAllMotors();
    Serial1.println("OK s");
    return;

  case 'g':
    if (!parseTwoFloatArgs(args, valueA, valueB))
    {
      Serial1.println("ERR g attendu: g X,Y");
      return;
    }
    targetPos.x = valueA;
    targetPos.y = valueB;
    targetPos.heading = currentPos.heading;
    maintainPosition = true;
    resetControllerState();
    Serial1.print("OK g ");
    Serial1.print(targetPos.x, 3);
    Serial1.print(",");
    Serial1.println(targetPos.y, 3);
    return;

  case 'o':
    if (!parseSingleFloatArg(args, valueA))
    {
      Serial1.println("ERR o attendu: o angle_deg");
      return;
    }
    targetPos.x = currentPos.x;
    targetPos.y = currentPos.y;
    targetPos.heading = normalizeAngleDeg(currentPos.heading + valueA);
    useHeadingControl = true;
    maintainPosition = true;
    resetControllerState();
    Serial1.print("OK o ");
    Serial1.println(targetPos.heading, 3);
    return;

  case 'p':
    Serial1.print("POS ");
    Serial1.print(currentPos.x, 3);
    Serial1.print(",");
    Serial1.print(currentPos.y, 3);
    Serial1.print(",");
    Serial1.println(currentPos.heading, 3);
    return;

  case 'z':
  {
    maintainPosition = false;
    stopAllMotors();
    myOtos.resetTracking();
    headingReferenceInitialized = false;
    sfe_otos_pose2d_t otos_data;
    myOtos.getPosition(otos_data);
    updateCurrentPosFromOtos(otos_data);
    targetPos = currentPos;
    Serial1.println("OK z");
    return;
  }

  default:
    Serial1.print("ERR unknown ");
    Serial1.println(cmd);
    return;
  }
}

void processUartRx()
{
  while (Serial1.available())
  {
    char c = static_cast<char>(Serial1.read());
    if (c == '\n' || c == '\r')
    {
      if (uartRxLine.length() > 0)
      {
        handleUartLineCommand(uartRxLine);
        uartRxLine = "";
      }
      continue;
    }

    if (uartRxLine.length() < 120)
    {
      uartRxLine += c;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(UART_BAUD_RATE);

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  stopAllMotors();

  Wire.begin();
  if (!myOtos.begin())
  {
    Serial.println("ERREUR: OTOS non detecte");
    while (1)
    {
      delay(10);
    }
  }

  delay(1000);
  myOtos.calibrateImu();
  myOtos.setLinearUnit(kSfeOtosLinearUnitInches);
  myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);
  myOtos.resetTracking();

  Serial.println("Teensy dev pret");
  Serial1.println("READY");
}

void loop()
{
  sfe_otos_pose2d_t otos_data;
  myOtos.getPosition(otos_data);
  updateCurrentPosFromOtos(otos_data);

  processUartRx();

  if (maintainPosition)
  {
    holdPosition();
  }

  delay(20);
}
