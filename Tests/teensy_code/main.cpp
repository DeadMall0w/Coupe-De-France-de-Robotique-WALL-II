#include <Arduino.h>

#include <Wire.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

// === PINS - À PERSONNALISER ===
#define MOTOR1_PWM 37 // Moteur 1 (avant-droit) - PWM
#define MOTOR1_DIR 39 // Moteur 1 - Direction
#define MOTOR2_PWM 13 // Moteur 2 (avant-gauche) - PWM
#define MOTOR2_DIR 41 // Moteur 2 - Direction
#define MOTOR3_PWM 14 // Moteur 3 (arrière-gauche) - PWM
#define MOTOR3_DIR 15 // Moteur 3 - Direction
#define MOTOR4_PWM 36 // Moteur 4 (arrière-droit) - PWM
#define MOTOR4_DIR 38 // Moteur 4 - Direction

// Inversion sens moteurs (true = inverse la consigne)
#define MOTOR1_INVERTED true
#define MOTOR2_INVERTED false
#define MOTOR3_INVERTED false
#define MOTOR4_INVERTED true

// ===  PARAMÈTRES PID ===
#define KP_X 16.0f
#define KI_X 0.9f
#define KD_X 3.2f

#define KP_Y 36.0f
#define KI_Y 0.9f
#define KD_Y 3.2f

// Signe de contrôle Y : -1 si la commande Y fait bouger OTOS dans le sens opposé
#define Y_CONTROL_SIGN -1.0f

#define KP_H 3.0f
#define KI_H 0.04f
#define KD_H 0.15f

#define CONTROL_DT 0.02f        // 20 ms
#define POSITION_DEADBAND 0.30f // unité OTOS (inch en config actuelle)
#define HEADING_DEADBAND_DEG 1.5f
#define INTEGRAL_LIMIT 20.0f
#define INTEGRAL_H_LIMIT 80.0f
#define MAX_COMMAND 180.0f // limite de commande avant mixage moteurs
#define MAX_ANGULAR_COMMAND 70.0f
#define MIN_ACTIVE_COMMAND 60.0f         // évite un retour trop lent en petits pas
#define MIN_ANGULAR_ACTIVE_COMMAND 50.0f // same mais avec les rotation
#define MAX_COMMAND_SLEW 700.0f          // unités/s (limitation d'accélération)
#define MAX_VELOCITY 255.0f
#define MANUAL_MOVE_DELTA 0.04f       // déplacement mini détecté entre 2 cycles (inch)
#define EXTERNAL_DETECT_CMD_MAX 35.0f // ignore la détection externe si le robot corrige déjà
#define REENGAGE_DELAY_MS 300         // pause courte avant reprise
#define TEST_RUN_MS 3000
#define TEST_PAUSE_MS 500
#define DEBUG_PRINT_MS 100UL // 0.1 s entre 2 points debug
#define UART_BAUD_RATE 115200

// Mapping repère OTOS -> repère robot (adapter selon montage)
#define OTOS_SWAP_XY true
#define OTOS_X_SIGN 1.0f
#define OTOS_Y_SIGN 1.0f
#define OTOS_H_SIGN 1.0f
#define OTOS_H_OFFSET_DEG 0.0f

// Si false: ignore l'angle OTOS pour stabiliser X/Y quand le cap dérive
#define USE_HEADING_CONTROL_DEFAULT true

// === STRUCTURE DE POSITION ===s
struct Position
{
  float x;
  float y;
  float heading;
};

// === FORWARD DECLARATIONS ===
void setup();
void loop();
void holdPosition();
void setHolonomicVelocity(float velX, float velY, float velW);
void setMotor(int motorNum, float speed);
void stopAllMotors();
void handleCommand(char cmd);
void testMotors();
void testSingleMotor(int motorNum);
void resetControllerState();
void printHoldDebug(const char *mode, float deltaPos, float errorX, float errorY, float errorH, float velX, float velY, float velW);
void testForwardDistanceCm(float distanceCm);
void moveRelativeInches(float deltaX, float deltaY);
void rotateRelativeDegrees(float deltaHeadingDeg);
void updateCurrentPosFromOtos(const sfe_otos_pose2d_t &otos_data);
void processUartRx();
void handleUartLineCommand(const String &line);
bool parseTwoFloatArgs(const String &args, float &first, float &second);
bool parseSingleFloatArg(const String &args, float &value);

// === INSTANCE OTOS ===
QwiicOTOS myOtos;

// === VARIABLES GLOBALES ===
Position currentPos = {0, 0, 0};
Position targetPos = {0, 0, 0};
bool maintainPosition = false;

// PID variables
float errorX_prev = 0, errorY_prev = 0;
float integralX = 0, integralY = 0;
float commandX_prev = 0, commandY_prev = 0;
float errorH_prev = 0;
float integralH = 0;
float commandW_prev = 0;
Position lastHoldPos = {0, 0, 0};
bool holdStateInitialized = false;
bool externalMoveActive = false;
unsigned long motionQuietSinceMs = 0;
bool debugMode = false;
bool useHeadingControl = USE_HEADING_CONTROL_DEFAULT;
unsigned long lastDebugPrintMs = 0;
float lastM1Cmd = 0, lastM2Cmd = 0, lastM3Cmd = 0, lastM4Cmd = 0;
bool headingReferenceInitialized = false;
float headingReferenceDeg = 0.0f;
float lastOtosRawHeadingDeg = 0.0f;
String uartRxLine = "";

static float normalizeAngleDeg(float angleDeg)
{
  while (angleDeg > 180.0f)
    angleDeg -= 360.0f;
  while (angleDeg <= -180.0f)
    angleDeg += 360.0f;
  return angleDeg;
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
  lastOtosRawHeadingDeg = otos_data.h;

  if (!headingReferenceInitialized)
  {
    headingReferenceDeg = otos_data.h;
    headingReferenceInitialized = true;
  }

  float headingRelativeDeg = (otos_data.h - headingReferenceDeg) * OTOS_H_SIGN;
  currentPos.heading = normalizeAngleDeg(headingRelativeDeg + OTOS_H_OFFSET_DEG);
}

// ===== INITIALISATION =====
void setup()
{
  Serial.begin(115200);
  Serial1.begin(UART_BAUD_RATE); // Teensy UART1: RX pin 0, TX pin 1
  delay(1000);

  // Config pins moteurs
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  // Initialiser tous les moteurs à l'arrêt
  stopAllMotors();

  // Initialiser I2C et OTOS
  Wire.begin(); // Pins par défaut: SDA=18, SCL=19

  Serial.println("Initialisation OTOS...");
  if (myOtos.begin() == false)
  {
    Serial.println("ERREUR: OTOS non détecté!");
    while (1)
      ;
  }

  Serial.println("Calibration de l'IMU (NE PAS TOUCHER LE ROBOT)...");
  // On attend un peu que le robot soit bien stable après l'allumage
  delay(1000);
  myOtos.calibrateImu();
  Serial.println("Calibration terminee !");

  // Configuration OTOS (à ajuster selon votre setup)
  myOtos.setLinearUnit(kSfeOtosLinearUnitInches);
  myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);
  myOtos.resetTracking();

  Serial.println("Système prêt!");
  Serial.println("Commandes:");
  Serial.println("  'm' = Maintenir position actuelle");
  Serial.println("  's' = Arrêter maintien");
  Serial.println("  'g' = Aller à position (X,Y)");
  Serial.println("  'r' = Déplacement relatif (dX,dY) en pouces");
  Serial.println("  'z' = Reset position OTOS à (0,0)");
  Serial.println("  'p' = Afficher position");
  Serial.println("  '1'..'4' = Tester un moteur individuellement");
  Serial.println("  't' = Tester les 4 moteurs (séquentiel)");
  Serial.println("  'f' = Test avance de X cm (4 moteurs)");
  Serial.println("  'o' = Test rotation de X degres");
  Serial.println("  'd' = Toggle debug maintien (CSV)");
  Serial.println("  'h' = Toggle asservissement angle OTOS");
  Serial.println("UART RPi actif sur Serial1 (pins 0/1) - commandes en ligne");
}

// ===== BOUCLE PRINCIPALE =====
void loop()
{
  // Lire la position actuelle de l'OTOS
  sfe_otos_pose2d_t otos_data;
  myOtos.getPosition(otos_data);
  updateCurrentPosFromOtos(otos_data);

  // Maintenir la position si actif
  if (maintainPosition)
  {
    holdPosition();
  }

  // Traiter les commandes série
  if (Serial.available())
  {
    char cmd = Serial.read();
    Serial.print("Commande reçue: ");
    Serial.println(cmd);
    handleCommand(cmd);
  }

  processUartRx();

  delay(20); // 50Hz contrôle loop
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
  case 'g':
    if (!parseTwoFloatArgs(args, valueA, valueB))
    {
      Serial1.println("ERR g attendu: g X,Y");
      return;
    }
    targetPos.x = valueA;
    targetPos.y = valueB;
    maintainPosition = true;
    holdStateInitialized = false;
    resetControllerState();
    Serial1.print("OK g ");
    Serial1.print(targetPos.x, 3);
    Serial1.print(",");
    Serial1.println(targetPos.y, 3);
    return;

  case 'r':
    if (!parseTwoFloatArgs(args, valueA, valueB))
    {
      Serial1.println("ERR r attendu: r dX,dY");
      return;
    }
    moveRelativeInches(valueA, valueB);
    Serial1.println("OK r");
    return;

  case 'f':
    if (!parseSingleFloatArg(args, valueA))
    {
      Serial1.println("ERR f attendu: f distance_cm");
      return;
    }
    testForwardDistanceCm(valueA);
    Serial1.println("OK f");
    return;

  case 'o':
    if (!parseSingleFloatArg(args, valueA))
    {
      Serial1.println("ERR o attendu: o angle_deg");
      return;
    }
    rotateRelativeDegrees(valueA);
    Serial1.println("OK o");
    return;

  case 'p':
    Serial1.print("POS ");
    Serial1.print(currentPos.x, 3);
    Serial1.print(",");
    Serial1.print(currentPos.y, 3);
    Serial1.print(",");
    Serial1.println(currentPos.heading, 3);
    return;

  case 'c':
    // Calibration manuelle à chaud
    maintainPosition = false;
    stopAllMotors();
    Serial1.print("Stabilisation (500ms)...");
    delay(500); // Laisse le robot absorber les vibrations

    Serial1.print("Calibration IMU en cours (NE PAS TOUCHER)...");
    myOtos.calibrateImu();
    myOtos.resetTracking(); // Remet X, Y et H à 0

    // On met à jour la cible pour éviter que le robot ne donne un à-coup
    sfe_otos_pose2d_t otos_data;
    myOtos.getPosition(otos_data);
    updateCurrentPosFromOtos(otos_data);
    targetPos = currentPos;

    Serial1.print("Calibration terminee !");
    break;

  default:
    handleCommand(cmd);
    Serial1.print("OK ");
    Serial1.println(cmd);
    return;
  }
}

// ===== FONCTION PRINCIPALE: MAINTENIR LA POSITION =====
void holdPosition()
{
  if (!holdStateInitialized)
  {
    lastHoldPos = currentPos;
    holdStateInitialized = true;
    externalMoveActive = false;
    motionQuietSinceMs = 0;
  }

  float deltaX = currentPos.x - lastHoldPos.x;
  float deltaY = currentPos.y - lastHoldPos.y;
  float deltaPos = sqrtf((deltaX * deltaX) + (deltaY * deltaY));
  lastHoldPos = currentPos;

  bool movingNow = deltaPos > MANUAL_MOVE_DELTA;
  bool controllerActive = (abs(commandX_prev) > EXTERNAL_DETECT_CMD_MAX) ||
                          (abs(commandY_prev) > EXTERNAL_DETECT_CMD_MAX);
  bool externalMotionDetected = movingNow && (!controllerActive || externalMoveActive);

  if (externalMotionDetected)
  {
    externalMoveActive = true;
    motionQuietSinceMs = millis();
    resetControllerState();
    stopAllMotors();
    if (debugMode)
    {
      printHoldDebug("FREEZE", deltaPos, 0, 0, 0, 0, 0, 0);
    }
    return;
  }

  if (externalMoveActive)
  {
    if ((millis() - motionQuietSinceMs) < REENGAGE_DELAY_MS)
    {
      stopAllMotors();
      if (debugMode)
      {
        float waitErrorX = targetPos.x - currentPos.x;
        float waitErrorY = targetPos.y - currentPos.y;
        printHoldDebug("WAIT", deltaPos, waitErrorX, waitErrorY, 0, 0, 0, 0);
      }
      return;
    }
    externalMoveActive = false;
    resetControllerState();
  }

  // Calculer les erreurs
  float errorX = targetPos.x - currentPos.x;
  float errorY = (targetPos.y - currentPos.y) * Y_CONTROL_SIGN;
  float errorH = 0.0f;
  if (useHeadingControl)
  {
    errorH = targetPos.heading - currentPos.heading;
    while (errorH > 180.0f)
      errorH -= 360.0f;
    while (errorH < -180.0f)
      errorH += 360.0f;
  }

  if (abs(errorX) < POSITION_DEADBAND)
  {
    errorX = 0;
  }
  if (abs(errorY) < POSITION_DEADBAND)
  {
    errorY = 0;
  }
  if (abs(errorH) < HEADING_DEADBAND_DEG)
  {
    errorH = 0;
  }

  if (errorX == 0 && errorY == 0 && errorH == 0)
  {
    integralX = 0;
    integralY = 0;
    integralH = 0;
  }

  // PID pour X
  integralX += errorX * CONTROL_DT;
  integralX = constrain(integralX, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float velX = (KP_X * errorX) + (KI_X * integralX) + (KD_X * (errorX - errorX_prev) / CONTROL_DT);
  errorX_prev = errorX;

  // PID pour Y
  integralY += errorY * CONTROL_DT;
  integralY = constrain(integralY, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float velY = (KP_Y * errorY) + (KI_Y * integralY) + (KD_Y * (errorY - errorY_prev) / CONTROL_DT);
  errorY_prev = errorY;

  integralH += errorH * CONTROL_DT;
  integralH = constrain(integralH, -INTEGRAL_H_LIMIT, INTEGRAL_H_LIMIT);
  float velW = (KP_H * errorH) + (KI_H * integralH) + (KD_H * (errorH - errorH_prev) / CONTROL_DT);
  errorH_prev = errorH;
  if (!useHeadingControl)
  {
    velW = 0.0f;
  }

  velX = constrain(velX, -MAX_COMMAND, MAX_COMMAND);
  velY = constrain(velY, -MAX_COMMAND, MAX_COMMAND);
  velW = constrain(velW, -MAX_ANGULAR_COMMAND, MAX_ANGULAR_COMMAND);

  // On empêche la commande de déplacement de tomber sous le seuil critique
  if (errorX != 0 && abs(velX) < MIN_ACTIVE_COMMAND)
  {
    velX = (velX >= 0.0f ? 1.0f : -1.0f) * MIN_ACTIVE_COMMAND;
  }
  if (errorY != 0 && abs(velY) < MIN_ACTIVE_COMMAND)
  {
    velY = (velY >= 0.0f ? 1.0f : -1.0f) * MIN_ACTIVE_COMMAND;
  }

  // On empêche la commande de rotation de tomber sous le seuil critique
  if (errorH != 0 && abs(velW) < MIN_ANGULAR_ACTIVE_COMMAND)
  {
    velW = (velW >= 0.0f ? 1.0f : -1.0f) * MIN_ANGULAR_ACTIVE_COMMAND;
  }

  float maxStep = MAX_COMMAND_SLEW * CONTROL_DT;
  velX = commandX_prev + constrain(velX - commandX_prev, -maxStep, maxStep);
  velY = commandY_prev + constrain(velY - commandY_prev, -maxStep, maxStep);
  velW = commandW_prev + constrain(velW - commandW_prev, -maxStep, maxStep);
  commandX_prev = velX;
  commandY_prev = velY;
  commandW_prev = velW;

  // Transformer la commande du repère monde (OTOS) vers le repère robot
  float headingRad = useHeadingControl ? (currentPos.heading * DEG_TO_RAD) : 0.0f;
  float cmdRobotX = (cosf(headingRad) * velX) + (sinf(headingRad) * velY);
  float cmdRobotY = (-sinf(headingRad) * velX) + (cosf(headingRad) * velY);

  // Appliquer les vitesses aux moteurs (cinématique holonome)
  setHolonomicVelocity(cmdRobotX, cmdRobotY, velW);

  if (debugMode)
  {
    printHoldDebug("CTRL", deltaPos, errorX, errorY, errorH, velX, velY, velW);
  }
}

// ===== CINÉMATIQUE HOLONOME =====
void setHolonomicVelocity(float velX, float velY, float velW)
{
  // Matrice standardisée pour Mecanum en "X"
  // Convention : +X (Avant), +Y (Gauche), +W (Sens anti-horaire)

  float m1_speed = velX + velY + velW; // Avant-droit (M1)
  float m2_speed = velX - velY - velW; // Avant-gauche (M2)
  float m3_speed = velX + velY - velW; // Arrière-gauche (M3)
  float m4_speed = velX - velY + velW; // Arrière-droit (M4)

  // Normalisation pour ne pas dépasser MAX_VELOCITY
  float maxSpeed = max(max(abs(m1_speed), abs(m2_speed)),
                       max(abs(m3_speed), abs(m4_speed)));

  if (maxSpeed > MAX_VELOCITY)
  {
    m1_speed = (m1_speed / maxSpeed) * MAX_VELOCITY;
    m2_speed = (m2_speed / maxSpeed) * MAX_VELOCITY;
    m3_speed = (m3_speed / maxSpeed) * MAX_VELOCITY;
    m4_speed = (m4_speed / maxSpeed) * MAX_VELOCITY;
  }

  lastM1Cmd = m1_speed;
  lastM2Cmd = m2_speed;
  lastM3Cmd = m3_speed;
  lastM4Cmd = m4_speed;

  // Envoi aux moteurs
  setMotor(1, m1_speed);
  setMotor(2, m2_speed);
  setMotor(3, m3_speed);
  setMotor(4, m4_speed);
}
// ===== CONTRÔLE MOTEUR (PWM + Direction) =====
void setMotor(int motorNum, float speed)
{
  // Limiter la vitesse
  speed = constrain(speed, -255, 255);

  bool inverted = false;

  int pwmPin, dirPin;

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

  // Appliquer PWM et direction
  int pwmValue = abs(speed);
  digitalWrite(dirPin, speed < 0 ? LOW : HIGH); // HIGH = une direction, LOW = autre
  analogWrite(pwmPin, pwmValue);
}

// ===== ARRÊTER TOUS LES MOTEURS =====
void stopAllMotors()
{
  analogWrite(MOTOR1_PWM, 0);
  analogWrite(MOTOR2_PWM, 0);
  analogWrite(MOTOR3_PWM, 0);
  analogWrite(MOTOR4_PWM, 0);
}

// ===== TRAITEMENT DES COMMANDES SÉRIE =====
void handleCommand(char cmd)
{
  int commaIndex;
  switch (cmd)
  {
  case 'm':
    // Maintenir position actuelle
    targetPos = currentPos;
    maintainPosition = true;
    holdStateInitialized = false;
    resetControllerState();
    Serial.println("Maintien de position ACTIVÉ");
    break;

  case 's':
    // Arrêter maintien
    maintainPosition = false;
    holdStateInitialized = false;
    externalMoveActive = false;
    motionQuietSinceMs = 0;
    resetControllerState();
    stopAllMotors();
    Serial.println("Maintien de position DÉSACTIVÉ");
    break;

  case 'p':
    // Afficher position
    Serial.print("Position actuelle: X=");
    Serial.print(currentPos.x);
    Serial.print(" Y=");
    Serial.print(currentPos.y);
    Serial.print(" H=");
    Serial.println(currentPos.heading);
    break;

  case 'z':
  {
    // Reset origine OTOS -> (0,0,0)
    maintainPosition = false;
    holdStateInitialized = false;
    externalMoveActive = false;
    motionQuietSinceMs = 0;
    resetControllerState();
    stopAllMotors();

    myOtos.resetTracking();
    headingReferenceInitialized = false;
    sfe_otos_pose2d_t otos_data;
    myOtos.getPosition(otos_data);
    updateCurrentPosFromOtos(otos_data);
    targetPos = currentPos;

    Serial.println("Position OTOS réinitialisée à 0,0");
    Serial.print("Position actuelle: X=");
    Serial.print(currentPos.x, 3);
    Serial.print(" Y=");
    Serial.print(currentPos.y, 3);
    Serial.print(" H=");
    Serial.println(currentPos.heading, 3);
    break;
  }

  case 'g':
  {
    // Aller à position absolue OTOS (en pouces)
    Serial.println("Entrez X,Y absolus en pouces (ex: 100,200):");
    while (Serial.available() == 0)
    { // Attendre l'entrée
      delay(1);
    }
    String input = Serial.readStringUntil('\n');
    commaIndex = input.indexOf(',');
    if (commaIndex > 0)
    {
      targetPos.x = input.substring(0, commaIndex).toFloat();
      targetPos.y = input.substring(commaIndex + 1).toFloat();
      maintainPosition = true;
      holdStateInitialized = false;
      resetControllerState();
      Serial.print("Aller à: X=");
      Serial.print(targetPos.x);
      Serial.print(" Y=");
      Serial.println(targetPos.y);
    }
    else
    {
      Serial.println("Format invalide. Utilisez: X,Y");
    }
    break;
  }

  case 'r':
  {
    maintainPosition = false; // Désactive le PID temporairement
    stopAllMotors();          // Arrête le robot pendant la frappe
    // Déplacement relatif en pouces (saisie séparée: dX puis dY)
    while (Serial.available() > 0)
    {
      char c = Serial.peek();
      if (c == '\n' || c == '\r' || c == ' ' || c == '\t')
      {
        Serial.read();
      }
      else
      {
        break;
      }
    }

    Serial.println("Entrez dX en pouces (ex: 12):");
    while (Serial.available() == 0)
    {
      delay(1);
    }
    float deltaX = Serial.readStringUntil('\n').toFloat();

    Serial.println("Entrez dY en pouces (ex: -6):");
    while (Serial.available() == 0)
    {
      delay(1);
    }
    float deltaY = Serial.readStringUntil('\n').toFloat();

    moveRelativeInches(deltaX, deltaY);
    break;
  }

  case 't':
    // Test moteurs
    testMotors();
    break;

  case '1':
  case '2':
  case '3':
  case '4':
    testSingleMotor(cmd - '0');
    break;

  case 'a':
    // Avancer de 100 mm (OTOS configuré en pouces)
    targetPos.x = currentPos.x + (100.0f / 25.4f);
    targetPos.y = currentPos.y;
    maintainPosition = true;
    holdStateInitialized = false;
    resetControllerState();
    Serial.println("Avancer de 100mm");
    break;

  case 'f':
  {
    Serial.println("Distance en cm (ex: 20):");
    while (Serial.available() == 0)
    {
      delay(1);
    }
    float distanceCm = Serial.readStringUntil('\n').toFloat();
    testForwardDistanceCm(distanceCm);
    break;
  }

  case 'o':
  {
    Serial.println("Angle en degres (ex: 90 ou -45):");
    while (Serial.available() == 0)
    {
      delay(1);
    }
    float angleDeg = Serial.readStringUntil('\n').toFloat();
    rotateRelativeDegrees(angleDeg);
    break;
  }

  case 'd':
    debugMode = !debugMode;
    if (debugMode)
    {
      Serial.println("DEBUG maintien: ON");
      Serial.println("DBG,time,mode,posX,posY,posH,posHraw,hRef,tgtX,tgtY,tgtH,errX,errY,errH,cmdX,cmdY,cmdW,m1,m2,m3,m4,deltaPos,extMove,headCtl");
    }
    else
    {
      Serial.println("DEBUG maintien: OFF");
    }
    break;

  case 'h':
    useHeadingControl = !useHeadingControl;
    resetControllerState();
    targetPos.heading = currentPos.heading;
    Serial1.println("Asservissement angle OTOS: ");
    Serial1.println(useHeadingControl ? "ON" : "OFF");
    break;
  }
}

// ===== TEST MOTEURS =====
void testMotors()
{
  Serial.println("\n=== TEST MOTEURS ===");
  Serial.print("M1 (PWM=");
  Serial.print(MOTOR1_PWM);
  Serial.print(", DIR=");
  Serial.print(MOTOR1_DIR);
  Serial.println("): ON");
  setMotor(1, 127); // 50% vitesse
  delay(TEST_RUN_MS);
  setMotor(1, 0);
  delay(TEST_PAUSE_MS);

  Serial.print("M2 (PWM=");
  Serial.print(MOTOR2_PWM);
  Serial.print(", DIR=");
  Serial.print(MOTOR2_DIR);
  Serial.println("): ON");
  setMotor(2, 127);
  delay(TEST_RUN_MS);
  setMotor(2, 0);
  delay(TEST_PAUSE_MS);

  Serial.print("M3 (PWM=");
  Serial.print(MOTOR3_PWM);
  Serial.print(", DIR=");
  Serial.print(MOTOR3_DIR);
  Serial.println("): ON");
  setMotor(3, 127);
  delay(TEST_RUN_MS);
  setMotor(3, 0);
  delay(TEST_PAUSE_MS);

  Serial.print("M4 (PWM=");
  Serial.print(MOTOR4_PWM);
  Serial.print(", DIR=");
  Serial.print(MOTOR4_DIR);
  Serial.println("): ON");
  setMotor(4, 127);
  delay(TEST_RUN_MS);
  setMotor(4, 0);

  Serial.println("=== FIN TEST ===\n");
}

void testSingleMotor(int motorNum)
{
  if (motorNum < 1 || motorNum > 4)
  {
    return;
  }

  maintainPosition = false;
  stopAllMotors();

  Serial.print("\n=== TEST MOTEUR M");
  Serial.print(motorNum);
  Serial.println(" ===");

  Serial.println("Sens 1");
  setMotor(motorNum, 110);
  delay(TEST_RUN_MS);
  setMotor(motorNum, 0);
  delay(TEST_PAUSE_MS);

  Serial.println("Sens 2");
  setMotor(motorNum, -110);
  delay(TEST_RUN_MS);
  setMotor(motorNum, 0);

  Serial.println("=== FIN TEST UNITAIRE ===\n");
}

void resetControllerState()
{
  integralX = 0;
  integralY = 0;
  integralH = 0;
  errorX_prev = 0;
  errorY_prev = 0;
  errorH_prev = 0;
  commandX_prev = 0;
  commandY_prev = 0;
  commandW_prev = 0;
}

void printHoldDebug(const char *mode, float deltaPos, float errorX, float errorY, float errorH, float velX, float velY, float velW)
{
  unsigned long nowMs = millis();
  if (lastDebugPrintMs == 0)
  {
    lastDebugPrintMs = nowMs;
  }

  if ((nowMs - lastDebugPrintMs) < DEBUG_PRINT_MS)
  {
    return;
  }

  // Cadence régulière à 100 ms (évite la dérive temporelle)
  lastDebugPrintMs += DEBUG_PRINT_MS;
  if ((nowMs - lastDebugPrintMs) >= DEBUG_PRINT_MS)
  {
    lastDebugPrintMs = nowMs;
  }

  Serial.print("DBG,");
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(mode);
  Serial.print(",");
  Serial.print(currentPos.x, 3);
  Serial.print(",");
  Serial.print(currentPos.y, 3);
  Serial.print(",");
  Serial.print(currentPos.heading, 3);
  Serial.print(",");
  Serial.print(lastOtosRawHeadingDeg, 3);
  Serial.print(",");
  Serial.print(headingReferenceDeg, 3);
  Serial.print(",");
  Serial.print(targetPos.x, 3);
  Serial.print(",");
  Serial.print(targetPos.y, 3);
  Serial.print(",");
  Serial.print(targetPos.heading, 3);
  Serial.print(",");
  Serial.print(errorX, 3);
  Serial.print(",");
  Serial.print(errorY, 3);
  Serial.print(",");
  Serial.print(errorH, 3);
  Serial.print(",");
  Serial.print(velX, 2);
  Serial.print(",");
  Serial.print(velY, 2);
  Serial.print(",");
  Serial.print(velW, 2);
  Serial.print(",");
  Serial.print(lastM1Cmd, 1);
  Serial.print(",");
  Serial.print(lastM2Cmd, 1);
  Serial.print(",");
  Serial.print(lastM3Cmd, 1);
  Serial.print(",");
  Serial.print(lastM4Cmd, 1);
  Serial.print(",");
  Serial.print(deltaPos, 4);
  Serial.print(",");
  Serial.print(externalMoveActive ? 1 : 0);
  Serial.print(",");
  Serial.println(useHeadingControl ? 1 : 0);
}

void testForwardDistanceCm(float distanceCm)
{
  if (distanceCm == 0.0f)
  {
    Serial.println("Distance nulle, test annulé.");
    return;
  }

  maintainPosition = false;
  holdStateInitialized = false;
  externalMoveActive = false;
  motionQuietSinceMs = 0;
  resetControllerState();
  stopAllMotors();

  const float distanceIn = distanceCm / 2.54f;

  sfe_otos_pose2d_t otos_data;
  myOtos.getPosition(otos_data);
  updateCurrentPosFromOtos(otos_data);

  float startX = currentPos.x;
  float targetX = startX + distanceIn;
  unsigned long startMs = millis();
  unsigned long lastPrintMs = 0;
  unsigned long timeoutMs = (unsigned long)(3000.0f + (abs(distanceCm) * 120.0f));

  Serial.print("TEST AVANCE: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  while (true)
  {
    myOtos.getPosition(otos_data);
    updateCurrentPosFromOtos(otos_data);

    float remainingIn = targetX - currentPos.x;
    float remainingCm = remainingIn * 2.54f;

    if (abs(remainingCm) < 1.0f)
    {
      break;
    }

    float pwm = constrain(remainingCm * 6.0f, -140.0f, 140.0f);
    if (abs(pwm) < 80.0f)
    {
      pwm = (pwm >= 0.0f) ? 80.0f : -80.0f;
    }

    setMotor(1, pwm);
    setMotor(2, pwm);
    setMotor(3, pwm);
    setMotor(4, pwm);

    unsigned long nowMs = millis();
    if ((nowMs - lastPrintMs) > 100)
    {
      lastPrintMs = nowMs;
      Serial.print("FWD,posX=");
      Serial.print(currentPos.x, 3);
      Serial.print(",targetX=");
      Serial.print(targetX, 3);
      Serial.print(",remainCm=");
      Serial.print(remainingCm, 1);
      Serial.print(",pwm=");
      Serial.println(pwm, 1);
    }

    if ((nowMs - startMs) > timeoutMs)
    {
      Serial.println("Timeout test avance.");
      break;
    }

    delay(20);
  }

  stopAllMotors();
  delay(200);

  myOtos.getPosition(otos_data);
  updateCurrentPosFromOtos(otos_data);
  float traveledCm = (currentPos.x - startX) * 2.54f;

  Serial.print("FIN TEST AVANCE, parcouru = ");
  Serial.print(traveledCm, 1);
  Serial.println(" cm");
}

void moveRelativeInches(float deltaX, float deltaY)
{
  targetPos.x = currentPos.x + deltaX;
  targetPos.y = currentPos.y + deltaY;
  targetPos.heading = currentPos.heading;

  maintainPosition = true;
  holdStateInitialized = false;
  externalMoveActive = false;
  motionQuietSinceMs = 0;
  resetControllerState();

  Serial.print("Déplacement relatif demandé: dX=");
  Serial.print(deltaX, 3);
  Serial.print(" dY=");
  Serial.print(deltaY, 3);
  Serial.print(" -> cible X=");
  Serial.print(targetPos.x, 3);
  Serial.print(" Y=");
  Serial.println(targetPos.y, 3);
}

void rotateRelativeDegrees(float deltaHeadingDeg)
{
  targetPos.x = currentPos.x;
  targetPos.y = currentPos.y;
  targetPos.heading = normalizeAngleDeg(currentPos.heading + deltaHeadingDeg);

  useHeadingControl = true;
  maintainPosition = true;
  holdStateInitialized = false;
  externalMoveActive = false;
  motionQuietSinceMs = 0;
  resetControllerState();

  Serial.print("Rotation demandee: dH=");
  Serial.print(deltaHeadingDeg, 2);
  Serial.print(" deg -> cible H=");
  Serial.println(targetPos.heading, 2);
}