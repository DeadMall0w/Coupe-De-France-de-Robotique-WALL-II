#include <SBC_MotoDriver3.h>
#include <Wire.h>  // inclut la librairie

// Adresse I2C et pin OE, à adapter à ton montage
SBCMotoDriver3 MotoDriver(0x15, 4);

double dX, dY;
void setup() {
  Serial.begin(9600);
   Wire.begin();
  // Pull the oe_pin low to activate the board
  MotoDriver.enabled(true);
  // Starts the I2C communication
  MotoDriver.begin();
  // Switch off all outputs
  MotoDriver.allOff();
  //MotoDriver.allOn(true, true);
  

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

void loop() {

    int valA0 = -analogRead(A0); // Lit la valeur analogique sur A0 (0 à 1023)
    int valA1 = analogRead(A1); // Lit la valeur analogique sur A1


    dX = (valA0 / 1023.0) * 2 - 1;
    dY = (valA1 / 1023.0) * 2 - 1;

    if (dX < 0.1 && dX > -0.1){
        dX = 0;
    }
    

    if (dY < 0.1 && dY > -0.1){
        dY = 0;
    }

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

    int speed = 170;
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

