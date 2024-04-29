#include <SPI.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_MMC5983MA
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "../lib/Magnetometer/Magnetometer.h"
#include "../lib/MagCal/mag_cal.h"
#include "../include/magnetotorquer.h"

#define XtX_RANK 4

int RX = 1;   
int TX = 0; 
int C740Wire1 = 14; 
int C740Wire2 = 15;
int C722Wire1 = 7;  
int C722Wire2 = 6; 

enum State {
  IDLE,
  READING,
  EXECUTING_COMMAND,
  SHOW_HELP
};

State currentState = IDLE; 

void setup() {
  configurePins(); 
  Serial1.begin(115200);
  initializeMagnetometer(); 
}

void configurePins(){
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(C740Wire1, OUTPUT);
  pinMode(C740Wire2, OUTPUT);
  pinMode(C722Wire1, OUTPUT);
  pinMode(C722Wire2, OUTPUT);
}

void TurnOff(){
        analogWrite(C740Wire1, 0);
        analogWrite(C740Wire2, 0);
        analogWrite(C722Wire1, 0);
        analogWrite(C722Wire2, 0);
}

void showHelp() {
  Serial1.println("Command Menu:");
  Serial1.println("Command 'M': Initiate continuous data measuremnts");
  Serial1.println("Command 'E': Escape current operation");
  Serial1.println("Command 'H': Display this command menu");
  Serial1.println("Command 'X...': Activate X-axis excitation");
  Serial1.println("Command 'Y...': Activate Y-axis excitation");
  Serial1.println("Command 'C': Start magnetometer calibration");
  Serial1.println("Command 'D': Begin demagnetization");
  Serial1.println("Command 'S': Detumble rotational motion");
  Serial1.println("Command 'R...': Rotate to specified degree");
}

int sgn(float val) {
      return (val > 0) - (val < 0);
    }

void Bdot(){
    //Magnetic field B measured in 
    //w = angular velocity rad/s dO/dt
    //Mdesired = -k*w
      Serial1.println("Command initiated");
      int dts = 0.2;
      int dtms = 200;
      float Bx1, By1, Bz1;
      float Bx2, By2, Bz2;
      float Mx, My;
      float omega, theta_B1, theta_B2;
      Bx1 = 0; 
      By1 = 0; 
      Bz1 = 0; 
      theta_B1 = 0;
      omega = 0; 
      int direction = sgn(omega);
      do{
        demagnetization(C740Wire1, C740Wire2);
        demagnetization(C722Wire1, C722Wire2); 
        GetMagMeasurements(&Bx2, &By2, &Bz2);
        theta_B2= atan2(By2, Bx2); //[-Pi; Pi]
        if (direction>=0){
          Mx = cos(theta_B2-PI/2); //M2 also perpendicular vector to B but in other direction 
          My = sin(theta_B2-PI/2);
        }else if(direction<0){
          Mx = cos(PI/2+theta_B2); //M1 perpendicular vector to B 
          My = sin(PI/2+theta_B2);
        }
  //atan2(y, x)
  //       atan2(+, -)  -> π/2 and π [1.57; 3.15]
  //       atan2(+, +) -> 0 and π/2 [0; 1.57]
  //       atan2(-, +) ->  -π and -π/2 [-1.57; 0]
  //       atan2(-, -) -> -π and -π/2 [-3.15; -1.57]
        float theta_M = atan2(My, Mx);
        Serial1.print("Torque : ");
        Serial1.println(theta_M);
        if(theta_M>0 && theta_M<=(PI/2)){
          TriggerCoils(C740Wire1, Mx); //x axis coil +
          TriggerCoils(C722Wire1, My);
        }else if(theta_M>(PI/2) && theta_M<=PI){
          TriggerCoils(C740Wire2, Mx); //x axis coil -
          TriggerCoils(C722Wire1, My);
        }else if(theta_M> (-PI) && theta_M<=(-PI/2)){
          TriggerCoils(C740Wire2, Mx); //x axis coil -
          TriggerCoils(C722Wire2,My);
        }else if(theta_M>(-PI/2) && theta_M<=0){
          TriggerCoils(C740Wire1, Mx); //x axis coil +
          TriggerCoils(C722Wire2, My);
        }
        Bx1 = Bx2; 
        By1 = By2; 
        Bz1 = Bz2; 

        Serial1.println(theta_B1);
        Serial1.println(theta_B2);
        omega = theta_B2 - theta_B1;
        direction =  sgn(omega);
        Serial1.print("fAngular speed: ");
        Serial1.print(abs(omega)/dtms);
        Serial1.println("rad/s");
        Serial1.print("direction: ");
        Serial1.println(sgn(omega));
        theta_B1 = atan2(By1, Bx1);
        delay(dtms);
      }while(omega>0); 
          analogWrite(C740Wire1, 0);
          analogWrite(C740Wire2, 0);
          analogWrite(C722Wire1, 0);
          analogWrite(C722Wire2, 0);
      Serial1.println("Command completed");
  }

void executeCommand(String command) {
  char commandType = command.charAt(0);
  int value = command.substring(1).toInt(); 

  switch (commandType) {
        case 'X':
          randomExcitation(C740Wire1, C740Wire2, value); 
          break;
        case 'Y':
          randomExcitation(C722Wire1, C722Wire2, value); 
          break;
        case 'C': 
          Calibrate(); 
          break; 
        case 'M':
          currentState = READING;
          break;
        case 'D': 
          demagnetization(C740Wire1, C740Wire2);
          demagnetization(C722Wire1, C722Wire2);
          Serial1.println("Demagnetization complited");
          break; 
        case 'S': 
          Bdot(); 
          break;
        default:
          Serial1.println("Unknown command.");
          break;
  }
  currentState = READING; 
}

void handleCommand(String command) {
  Serial1.println("Received command: " + command);
  char commandType = command.charAt(0);
  switch (commandType) {
    case 'H': 
      currentState = SHOW_HELP;
      showHelp(); 
      break;
    case 'E': 
      currentState = IDLE;
      TurnOff();
      break;
    case 'M': 
      currentState = READING;
      break;
    default: 
      currentState = EXECUTING_COMMAND; 
      executeCommand(command);
      break;
  }
}

void loop() {
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n'); 
    handleCommand(command);
  }
  if (currentState == READING) {
    ReadDataCal(); 
  }
  delay(100);
}

