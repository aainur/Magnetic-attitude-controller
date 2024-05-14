#include "Magnetometer.h"
#include <SPI.h>
#include <Arduino.h>
#include "mag_cal.h"
#define XtX_RANK 4

SFE_MMC5983MA myMag; 
// Constants for magnetometer and coils
int csPin = 17;   

// Calibration matrices
T_MATRIX HardIronoffset[XtX_RANK]={0,0,0,0};

// Offsets for magnetic sensor
static uint32_t offsetX = 131072;
static uint32_t offsetY = 131072;
static uint32_t offsetZ = 131072;   


void initializeMagnetometer() {
    SPI.begin();
    while (!myMag.begin(csPin)) {
        Serial.println("MMC5983MA did not respond. Retrying...");
        delay(500);
        myMag.softReset();
        delay(500);
    }
    Serial.println("MMC5983MA connected");
    MagSensorInitCalibration();
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

void Calibrate(){
  Serial1.println("Calibration initiated");

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
  Serial1.println("Calibration completed");
  delay(1000); 
}

void readData(){
      T_MATRIX uncalX, uncalY, uncalZ;
      GetMagMeasurements(&uncalX, &uncalY, &uncalZ); 
      Serial1.print(uncalX,4);
      Serial1.print(";");
      Serial1.print(uncalY,4);
      Serial1.print(";");
      Serial1.print(uncalZ,4);
      Serial1.print("\n");
      delay(100);
}

void ReadDataCal(){
      T_MATRIX uncalX, uncalY, uncalZ;
      GetMagMeasurements(&uncalX, &uncalY, &uncalZ); 
      T_MATRIX calX, calY, calZ;
      calX = uncalX - HardIronoffset[0];
      calY = uncalY - HardIronoffset[1];
      calZ = uncalZ - HardIronoffset[2];
      Serial1.print(calX,4);
      Serial1.print(";");
      Serial1.print(calY,4);
      Serial1.print(";");
      Serial1.print(calZ,4);
      Serial1.println(";");
      delay(100);
}

void GetCalMagMeasurements(float *scaledX, float *scaledY, float *scaledZ) {
    T_MATRIX uncalX, uncalY, uncalZ;
    GetMagMeasurements(&uncalX, &uncalY, &uncalZ); 
    *scaledX = uncalX - HardIronoffset[0];
    *scaledY = uncalY - HardIronoffset[1];
    *scaledZ = uncalZ - HardIronoffset[2];
      // Serial1.print(*scaledX,4);
      // Serial1.print(";");
      // Serial1.print(*scaledY,4);
      // Serial1.print(";");
      // Serial1.print(*scaledZ,4);
      // Serial1.print(";");

}

void TriggerCoils(int coil,  float a){
      int pwmOutput = (int)(abs(a) * 255); 
      analogWrite(coil, pwmOutput);
    }

void randomExcitation(int wire1, int wire2, int pwm){
  T_MATRIX tempX, tempY, tempZ;
  T_MATRIX calX, calY;
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
    delay(1000);
  }

