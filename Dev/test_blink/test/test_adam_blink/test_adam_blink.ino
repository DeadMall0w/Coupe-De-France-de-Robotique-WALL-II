#include <SBC_MotoDriver3.h>
#include <Wire.h>  // inclut la librairie

//https://chatgpt.com/c/691618c7-c8f0-8327-8d18-18b401324cda

// Adresse I2C et pin OE, à adapter à ton montage
SBCMotoDriver3 MotoDriver(0x15, 4);

double dX = 0.0, dY = 0.0;
String inputString = "";      // Chaîne pour stocker les données entrantes
boolean stringComplete = false;  // Indique si la chaîne est complète

// LED de debug (pin 13 intégrée sur Arduino)
#define LED_PIN 13
#define LED_RX_PIN 12  // LED pour indiquer réception données
#define LED_MOVE_PIN 11  // LED pour indiquer mouvement
unsigned long lastBlink = 0;
unsigned long lastDataReceived = 0;
int dataReceivedCount = 0;

void setup() {
  // Initialiser les LED de debug
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_RX_PIN, OUTPUT);
  pinMode(LED_MOVE_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, HIGH);  // LED allumée = setup en cours
  digitalWrite(LED_RX_PIN, LOW);
  digitalWrite(LED_MOVE_PIN, LOW);
  
  Serial.begin(9600);
  inputString.reserve(32);  // Réserve de la mémoire pour la chaîne
  
  // Attendre que le port série soit prêt
  delay(100);
  
  Wire.begin();
  
  // Pull the oe_pin low to activate the board
  MotoDriver.enabled(true);
  
  // Starts the I2C communication
  MotoDriver.begin();
  
  // Switch off all outputs
  MotoDriver.allOff();
  
  // Clignoter 3 fois = setup OK
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_RX_PIN, HIGH);
    digitalWrite(LED_MOVE_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_RX_PIN, LOW);
    digitalWrite(LED_MOVE_PIN, LOW);
    delay(200);
  }
  digitalWrite(LED_PIN, LOW);
  
  lastBlink = millis();
}


void setMotor(int v1, int v2, int v3, int v4){
    // Moteur 1
    if (v1 < 0){
        v1 = -v1;
        MotoDriver.pwm(0, 0);
        MotoDriver.pwm(1, v1);
    }else{
        MotoDriver.pwm(1, 0);
        MotoDriver.pwm(0, v1);
    }

    // Moteur 2
    if (v2 < 0){
        v2 = -v2;
        MotoDriver.pwm(2, 0);
        MotoDriver.pwm(3, v2);
    }else{
        MotoDriver.pwm(3, 0);
        MotoDriver.pwm(2, v2);
    }

    // Moteur 3
    if (v3 < 0){
        v3 = -v3;
        MotoDriver.pwm(4, 0);
        MotoDriver.pwm(5, v3);
    }else{
        MotoDriver.pwm(5, 0);
        MotoDriver.pwm(4, v3);
    }

    // Moteur 4
    if (v4 < 0){
        v4 = -v4;
        MotoDriver.pwm(6, 0);
        MotoDriver.pwm(7, v4);
    }else{
        MotoDriver.pwm(7, 0);
        MotoDriver.pwm(6, v4);
    }

}

void parseSerialData() {
  // Parse les données reçues au format "dX,dY\n"
  if (stringComplete) {
    // LED RX clignote = données reçues et parsées
    digitalWrite(LED_RX_PIN, HIGH);
    lastDataReceived = millis();
    dataReceivedCount++;
    
    int commaIndex = inputString.indexOf(',');
    if (commaIndex > 0) {
      String dxStr = inputString.substring(0, commaIndex);
      String dyStr = inputString.substring(commaIndex + 1);
      
      dX = dxStr.toFloat();
      dY = dyStr.toFloat();
      
      // Appliquer un seuil de zone morte
      if (dX < 0.1 && dX > -0.1) {
        dX = 0;
      }
      if (dY < 0.1 && dY > -0.1) {
        dY = 0;
      }
      
      // LED_MOVE allumée si mouvement détecté
      if (dX != 0.0 || dY != 0.0) {
        digitalWrite(LED_MOVE_PIN, HIGH);
      } else {
        digitalWrite(LED_MOVE_PIN, LOW);
      }
    }
    
    // Réinitialiser pour la prochaine lecture
    inputString = "";
    stringComplete = false;
  }
}

void loop() {
  // Lire les données série caractère par caractère
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // LED_PIN clignote à chaque caractère reçu (debug TX/RX)
    digitalWrite(LED_PIN, HIGH);
    delay(2);  // Flash très court
    digitalWrite(LED_PIN, LOW);
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  
  // Parser les données reçues
  parseSerialData();
  
  // Éteindre la LED RX après 100ms (preuve de réception)
  if (millis() - lastDataReceived > 100) {
    digitalWrite(LED_RX_PIN, LOW);
  }
  
  // Clignotement lent LED_PIN si pas de données depuis 2 secondes (watchdog)
  if (millis() - lastDataReceived > 2000) {
    if (millis() - lastBlink > 1000) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastBlink = millis();
    }
  }

  // Le reste du code utilise maintenant dX et dY reçus via UART
  // au lieu de les lire depuis les joysticks

  /*
  *2   4

  *0   6 
  */
 
    // setMotor(-100,100,0,0);
    // delay(1000);
    // setMotor(100,0,0,0);
    // delay  (1000);
    // MotoDriver.chanPwm(0, 200);
    // MotoDriver.chanPwm(2, 40);
    // MotoDriver.chanPwm(4, 80);
    // MotoDriver.chanPwm(6, 100);

    int speed = 180;
    if (dY >= 0.7 && dX == 0){ // avancer tout droit
        setMotor(speed, speed, speed, speed);
    }else if (dY <= -0.7 && dX == 0){ // marche arriere
        setMotor(-speed, -speed, -speed, -speed);
    }else if (dY == 0 && dX >= 0.6){ // droite
        setMotor(-speed, speed, -speed, speed);
    }else if (dY == 0 && dX <= -0.6){ // droite
        setMotor(speed, -speed, speed, -speed);
    }else if (dX >= .7 && dY >= 0.7)
       setMotor(0, speed, 0, speed);
    else if (dX >= .7 && dY <= -0.7){
       setMotor(0, -speed, 0, -speed);
    }
       else if (dX <= -.7 && dY >= 0.7){
       setMotor(speed, 0, speed, 0);
       }
    else if (dX <= -.7 && dY <= -0.7)
    {
       setMotor(-speed,0,-speed,0);
    }
    else{
        setMotor(0,0,0,0);
    }
}













// void setup() {
//   Serial.begin(9600); // Démarre le port série à 9600 bauds
// }


// double map(double x, double in_min, double in_max, double out_min, double out_max);
// void readValue();

// double dX, dY;

// void loop() {

//     readValue();
//     Serial.print(" dX : ");
//     Serial.println(dX);
//     Serial.print(" dY : ");
//     Serial.println(dY);
//     Serial.println("");
//     Serial.println("");



//     delay(500); // pause de 500 ms entre les lectures
// }


// void readValue(){
//   int valA0 = analogRead(A0); // Lit la valeur analogique sur A0 (0 à 1023)
//   int valA1 = analogRead(A1); // Lit la valeur analogique sur A1


//     dX = (valA0 / 1023.0) * 2 - 1;
//     dY = (valA1 / 1023.0) * 2 - 1;

//     if (dX < 0.1 && dX > -0.1){
//         dX = 0;
//     }
    

//     if (dY < 0.1 && dY > -0.1){
//         dY = 0;
//     }
// }
