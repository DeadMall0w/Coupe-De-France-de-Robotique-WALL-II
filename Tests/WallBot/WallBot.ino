//* Driver 1
#define DIR1 1
#define DIR2 3
#define PWM1 2
#define PWM2 4

//* Driver 2 (Attention aux pins 0, voyez ma note plus bas)
#define DIR3 8
#define DIR4 6
#define PWM3 5
#define PWM4 7

//* Prototypes 
void setMotor(int motorID, int speed);
void setMotors(int speed1, int speed2, int speed3, int speed4);


void setup() {
  Serial.begin(9600);



  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  analogWriteFrequency(PWM1, 20000);
  analogWriteFrequency(PWM2, 20000);
  analogWriteFrequency(PWM3, 20000);
  analogWriteFrequency(PWM4, 20000);

 }

void loop() {
  // Serial.println("Hello3 !");
   setMotor(1, -50); 
     setMotor(3, -50); 
  // delay(200);
  setMotor(2, 50);
  setMotor(4, 50);

 
  delay(1000);
  setMotor(1, 50); 
     setMotor(3, 50); 
    // delay(200);
  setMotor(4, -50);
  setMotor(2, -50);

  // setMotors(30, -20, 20, -20);

  delay(1000);
}




void setMotor(int motorID, int speed) {
  switch (motorID) {
    case 1:
      if (speed <= 0) {
        digitalWrite(DIR1, LOW);
        analogWrite(PWM1, -speed);
      } else {
        digitalWrite(DIR1, HIGH);
        analogWrite(PWM1, speed);
      }
      break;
    case 2:
      if (speed <= 0) {
        digitalWrite(DIR2, LOW);
        analogWrite(PWM2, -speed);
      } else {
        digitalWrite(DIR2, HIGH);
        analogWrite(PWM2, speed);
      }
      break;
    case 3:
      if (speed <= 0) {
        digitalWrite(DIR3, LOW);
        analogWrite(PWM3, -speed);
      } else {
        digitalWrite(DIR3, HIGH);
        analogWrite(PWM3, speed);
      }
      break;
    case 4:
      if (speed <= 0) {
        digitalWrite(DIR4, LOW);
        analogWrite(PWM4, -speed);
      } else {
        digitalWrite(DIR4, HIGH);
        analogWrite(PWM4, speed);
      }
      break;
    default:
      Serial.print("Moteur inexistant !"); 
  }
}

void setMotors(int speed1, int speed2, int speed3, int speed4) {
  // --- Moteur 1 ---
  if (speed1 >= 0) {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, speed1);
  } else {
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, -speed1);
  }

  // --- Moteur 2 ---
  if (speed2 >= 0) {
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, speed2);
  } else {
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, -speed2);
  }

  // --- Moteur 3 ---
  if (speed3 >= 0) {
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, speed3);
  } else {
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, -speed3);
  }

  // --- Moteur 4 ---
  if (speed4 >= 0) {
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM4, speed4);
  } else {
    digitalWrite(DIR4, LOW);
    analogWrite(PWM4, -speed4);
  }
}