#include <SPI.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "../lib/MagCal/mag_cal.h"
#include "../include/magnetotorquer.h"

#define XtX_RANK 4

SFE_MMC5983MA myMag;
int csPin = 17;
int RX = 1; 
int TX = 0; 
int C740Wire1 = 14; //x axis coil +
int C740Wire2 = 15; //x axis coil -
int C722Wire1 = 7;  //y axis coil +
int C722Wire2 = 6; //y axis coil -
const float initialPWM = 1.0;  // Initial PWM value 100% (255*1)
const float pwmStep = 0.1;  // Step size of PWM (255*0.1= 10%)
T_MATRIX Bp[XtX_RANK]={0.0,0.0,0.0,0.0};
T_MATRIX HardIronoffset[XtX_RANK]={0,0,0,0};

static uint32_t offsetX = 131072;
static uint32_t offsetY = 131072;
static uint32_t offsetZ = 131072;

unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
bool measure = false; 

bool calibrationRequested = false;

void setup() {
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(C740Wire1, OUTPUT);
  pinMode(C740Wire2, OUTPUT);
  pinMode(C722Wire1, OUTPUT);
  pinMode(C722Wire2, OUTPUT);
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial.println("MMC5983MA");
    SPI.begin();
    while (myMag.begin(csPin) == false)
    {
        Serial.println("MMC5983MA did not respond. Retrying...");
        delay(500);
        myMag.softReset();
        delay(500);
    }
    Serial.println("MMC5983MA connected");
    MagSensorInitCalibration();
  
}

void loop() {
 if (Serial1.available()) {
    String command = "";
    char receivedChar;
    while (Serial1.available() > 0) {
      receivedChar = Serial1.read();
      command += receivedChar;
      delay(5);  
    }
    
    Serial1.print("Received command: ");
    Serial1.println(command);
    if (command[0] == 'X' || command[0] == 'Y') {
      int value = command.substring(1).toInt();
      char commandType = command[0];
      
      switch(commandType) {
        case 'X':
          randomExcitation(C740Wire1, C740Wire2, value); 
          break;
        case 'Y':
          randomExcitation(C722Wire1, C722Wire2, value); 
          break;
      }
    } else if (command[0] == 'R' ) {
        int value = command.substring(1).toInt();
        randomExcitation(C740Wire1, C740Wire2, value); 
      }else {
      switch(command[0]) {
        case 'C': 
          Calibrate(); 
          break; 
        case 'm':
          readData(); 
          break; 
        case 'M': 
          ReadDataCal(); 
          break; 
        case 'D': 
          demagnetization(C740Wire1, C740Wire2);
          demagnetization(C722Wire1, C722Wire2);
          break; 
        case 'S': 
          Bdot(); 
          break;
        case 'E': 
          TurnOff(); 
          break;
      }
    }
  }
  delay(100);
}

void TurnOff(){
        analogWrite(C740Wire1, 0);
        analogWrite(C740Wire2, 0);
        analogWrite(C722Wire1, 0);
        analogWrite(C722Wire2, 0);
}

void Calibrate(){
  for(int i=0; i<XtX_RANK; i++){ 
    HardIronoffset[i] = 0; //reset hard iron offset before calibration 
    }
  MagSensorInitCalibration();
  updateOffset(&offsetX, &offsetY, &offsetZ); //calculates magnetometers internal offset

  for(int i=0; i<50; i++){
    float Bp[4]={0,0,0,1.0};
    GetMagMeasurements(&Bp[0],&Bp[1],&Bp[2]); //reads magnetometer readings-offset 
    MagSensorUpdateCalibration(Bp); //updates calibration 
    delay(random(500)); 
  }
  MagSensorCalculateCalibrationResult(); 
  MagSensorGetCalibrationResult(HardIronoffset); //gets hard iron offset
  Serial1.println("Calibration values: ");
  Serial1.print(HardIronoffset[0],4);
  Serial1.print(" ");
  Serial1.print(HardIronoffset[1],4);
  Serial1.print(" ");
  Serial1.println(HardIronoffset[2],4);
  Serial1.print("Magnetic field strength: ");
  Serial1.println(MagSensorGetMagneticFieldStrength_uT(),4);
}


void readData(){
  Serial.println("Before Calibration: ");
  for(int i = 0; i < 60; i++){
      T_MATRIX uncalX, uncalY, uncalZ;
      GetMagMeasurements(&uncalX, &uncalY, &uncalZ); 
      Serial1.print(uncalX,4);
      Serial1.print(" ");
      Serial1.print(uncalY,4);
      Serial1.print(" ");
      Serial1.println(uncalZ,4);
      delay(100);
      }
}

void ReadDataCal(){
  Serial.println("After Calibration: ");
  for(int i = 0; i < 60; i++){
      T_MATRIX uncalX, uncalY, uncalZ;
      GetMagMeasurements(&uncalX, &uncalY, &uncalZ); 
      T_MATRIX calX, calY, calZ;
      calX = uncalX - HardIronoffset[0];
      calY = uncalY - HardIronoffset[1];
      calZ = uncalZ - HardIronoffset[2];
      Serial1.print(calX,4);
      Serial1.print(" ");
      Serial1.print(calY,4);
      Serial1.print(" ");
      Serial1.println(calZ,4);
      delay(100);
      }
}

void GetMagMeasurements(float *scaledX, float *scaledY, float *scaledZ) {
    uint32_t currentX = 131072;
    uint32_t currentY = 131072;
    uint32_t currentZ = 131072;
    myMag.getMeasurementXYZ(&currentX, &currentY, &currentZ);
    *scaledX = (((float)currentX - (float)offsetX) / 131072.0) * 8.0;
    *scaledY = (((float)currentY - (float)offsetY) / 131072.0) * 8.0; 
    *scaledZ = (((float)currentZ - (float)offsetZ) / 131072.0) * 8.0; 
}


bool updateOffset(uint32_t *offsetX, uint32_t *offsetY, uint32_t *offsetZ) 
{
  bool success = true; 
  success &= myMag.performSetOperation(); 

  uint32_t setX = 131072;
  uint32_t setY = 131072;
  uint32_t setZ = 131072;

  success &= myMag.getMeasurementXYZ(&setX, &setY, &setZ); 
  success &= myMag.getMeasurementXYZ(&setX, &setY, &setZ); 

  success &= myMag.performResetOperation();

  uint32_t resetX = 131072;
  uint32_t resetY = 131072;
  uint32_t resetZ = 131072;

  success &= myMag.getMeasurementXYZ(&resetX, &resetY, &resetZ); 
  success &= myMag.getMeasurementXYZ(&resetX, &resetY, &resetZ); 

  if (success)
  {
    *offsetX = (setX + resetX) / 2;
    *offsetY = (setY + resetY) / 2;
    *offsetZ = (setZ + resetZ) / 2;

  }
  return success;
}

void demagnetization(int wire1, int wire2){
    float pwmValue = initialPWM;
   while (pwmValue >= 0.0) {
    int pwmOutput = (int)(pwmValue * 255);  
    analogWrite(wire1, pwmOutput);
    analogWrite(wire2, pwmOutput);
    pwmValue -= pwmStep;
    delay(100);
  }
  analogWrite(wire1, 0);
  analogWrite(wire2, 0);
  Serial1.println("Demagnetization complited");
}

void randomExcitation(int wire1, int wire2, int pwm){
  T_MATRIX tempX, tempY, tempZ;
  T_MATRIX calX, calY, calZ;
    if(pwm>0){
      int pwmOutput = abs((int)((pwm/100) * 255));
      analogWrite(wire1, pwmOutput);
      GetMagMeasurements(&tempX, &tempY, &tempZ);
      calX = tempX - HardIronoffset[0];
      Serial1.println(calX,4);
      analogWrite(wire1, 0);
    }else{
      int pwmOutput = abs((int)((pwm/100) * 255));
      analogWrite(wire2, pwmOutput);
      GetMagMeasurements(&tempX, &tempY, &tempZ);
      calY = tempY - HardIronoffset[1];
      Serial1.println(calY,4);
      analogWrite(wire2, 0);
    }
    delay(100);
  }

    void TriggerCoils(int coil,  float a){
      int pwmOutput = (int)(abs(a) * 255); 
      analogWrite(coil, pwmOutput);
    }

    int sgn(float val) {
      return (val > 0) - (val < 0);
    }

void Bdot(){
    int dts = 0.2;
    int dtms = 200;
    float Bx1, By1, Bz1;
    float Bx2, By2, Bz2;
    float Mx, My;
    float dBx_dt, dBy_dt, dBz_dt;
    float omega, theta_B1, theta_B2;

     Bx1 = 0; 
     By1 = 0; 
     Bz1 = 0; 
    theta_B1 = fmod(atan2(By1, Bx1) * 180.0 / PI, 360.0);
    Serial1.print("Theta_B1: "); 
    Serial1.println(theta_B1);
    omega = 0; 
    int direction =  sgn(omega);
    
    do{
      demagnetization(C740Wire1, C740Wire2);
      demagnetization(C722Wire1, C722Wire2); 
      GetMagMeasurements(&Bx2, &By2, &Bz2);
      theta_B2= fmod(atan2(By2, Bx2) * 180.0 / PI, 360.0);

      if (direction>=0){
        Mx = fmod(cos((theta_B2-90))* PI/180.0, 360.0); //M2 also perpendicular vector to B but in other direction 
        My = fmod(sin((theta_B2-90))* PI/180.0, 360.0);
      }else if(direction<0){
        Mx = fmod(cos(90+theta_B2)* PI/180.0, 360.0); //M1 perpendicular vector to B 
        My = fmod(sin(90+theta_B2)* PI/180.0, 360.0);
      }

      float theta_M = fmod(atan2(My, Mx) * 180.0 / PI + 360.0, 360.0);
      if(theta_M>0 && theta_M<=90){
        TriggerCoils(C740Wire1, Mx); //x axis coil +
        TriggerCoils(C722Wire1, My);

      }else if(theta_M>90 && theta_M<=180){
        TriggerCoils(C740Wire2, Mx); //x axis coil -
        TriggerCoils(C722Wire1, My);
      }else if(theta_M>180 && theta_M<=270){
        TriggerCoils(C740Wire2, Mx); //x axis coil -
        TriggerCoils(C722Wire2,My);
  
      }else if(theta_M>270 && theta_M<=360){
        TriggerCoils(C740Wire1, Mx); //x axis coil +
        TriggerCoils(C722Wire2, My);

      }

    Bx1 = Bx2; 
    By1 = By2; 
    Bz1 = Bz2; 
    theta_B1 = fmod(atan2(By1, Bx1) * 180.0 / PI, 360.0);
    Serial1.println(theta_B1);
    delay(dtms);

    Serial1.println(theta_B2);
    dBx_dt = (Bx2 - Bx1) / dts;
    dBy_dt = (By2 - By1) / dts;
    dBz_dt = (Bz2 - Bz1) / dts;

    omega = theta_B2 - theta_B1;  
    Serial1.print("Angular speed: ");
    Serial1.println(omega);

    }while(abs(omega)>1); 
        analogWrite(C740Wire1, 0);
        analogWrite(C740Wire2, 0);
        analogWrite(C722Wire1, 0);
        analogWrite(C722Wire2, 0);
}

void RotateTo(int target)
{
   float Bx, By, Bz, Rx, Ry;
   float Input_angle, Setpoint_angle; 
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);

   int direction =  sgn(target); 

   GetMagMeasurements(&Bx, &By, &Bz);
   Input_angle = fmod(atan2(By, Bx)*180/PI, 360.0); 
   Setpoint_angle = Input_angle + target; 
  
  while(Setpoint_angle!=Input_angle){
   double error = Setpoint_angle - Input_angle;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;

   Output = kp * error + ki * errSum + kd * dErr;

   lastErr = error;
   lastTime = now;

    Rx = cos(Output * PI / 180.0 / 2.0) * Bx + sin(Output * PI / 180.0 / 2.0) * By;
    Ry = cos(Output * PI / 180.0 / 2.0) * By - sin(Output * PI / 180.0 / 2.0) * Bx;

    if(direction>0){
      TriggerCoils(C740Wire1, Rx);
      TriggerCoils(C722Wire1, Ry);
    }else if(direction<0){
      TriggerCoils(C740Wire2, Rx); 
      TriggerCoils(C722Wire2, Ry);
    }

    GetMagMeasurements(&Bx, &By, &Bz);
    Input_angle = fmod(atan2(By, Bx)*180/PI, 360.0); 
  }
        analogWrite(C740Wire1, 0);
        analogWrite(C740Wire2, 0);
        analogWrite(C722Wire1, 0);
        analogWrite(C722Wire2, 0);
}
