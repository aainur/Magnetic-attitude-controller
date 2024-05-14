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
int Coil_X_positive = 14; 
int Coil_X_negative = 15;
int Coil_Y_negative = 7;  
int Coil_Y_positive = 6; 

const float initialPWM = 1.0;  // Initial PWM value 100% (255*1)
const float pwmStep = 0.1; 

enum State {
  IDLE,
  READING,
  EXECUTING_COMMAND,
  SHOW_HELP
};

State currentState = IDLE; 

void setup() {
  configurePins(); 
  analogWriteResolution(8); //PWM resolution so that the maximum is 255 
  Serial1.begin(115200);
  while (Serial1.available()) {
    Serial1.read(); 
  }
  initializeMagnetometer(); 
}

void configurePins(){
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(Coil_X_positive, OUTPUT);
  pinMode(Coil_X_negative, OUTPUT);
  pinMode(Coil_Y_negative, OUTPUT);
  pinMode(Coil_Y_positive, OUTPUT);
}

void TurnOff(){
        analogWrite(Coil_X_positive, 0);
        analogWrite(Coil_X_negative, 0);
        analogWrite(Coil_Y_negative, 0);
        analogWrite(Coil_Y_positive, 0);
}

void demagnetization(){
  float pwmValue = initialPWM; //initialPWM=1.0
   while (pwmValue >= 0.0) {
    int pwmOutput = (int)(pwmValue * 255); 
    analogWrite(Coil_X_positive, pwmOutput);
    analogWrite(Coil_Y_negative, pwmOutput);
    analogWrite(Coil_X_negative, pwmOutput);
    analogWrite(Coil_Y_positive, pwmOutput);
    pwmValue -= pwmStep; //pwmStep=0.1
    delay(10);
  }
  analogWrite(Coil_X_positive, 0);
  analogWrite(Coil_Y_negative, 0);
  analogWrite(Coil_X_negative, 0);
  analogWrite(Coil_Y_positive, 0);
  delay(1);
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

bool checkForExitCommand() {
  while (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    if (command == "E") {
      Serial1.println("Exiting current operation");
      currentState = IDLE;
      return true; 
    }
  }
  return false; 
}

float normalizeAngle(float angle) {
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

float calAngularVelocity(float angle1, float angle2, float dt) {
    float dAngle = normalizeAngle(angle2) - normalizeAngle(angle1);
    // if (dAngle > M_PI) {
    //     dAngle -= 2 * M_PI;
    // } else if (dAngle < -M_PI) {
    //     dAngle += 2 * M_PI;
    // }
    return dAngle / dt;
}

void Bdot(){
      Serial.println("B-dot algorithm initiated");
      float dt; //time in seconds
      float Bx1 =0, By1=0, Bz1=0;
      float Bx2, By2, Bz2;
      float Mx, My, azimuth_M;
      float acceleration, omega_z1, omega_z2=0, azimuth_B2;
      unsigned long lastTime = millis();

      demagnetization();
      GetCalMagMeasurements(&Bx1, &By1, &Bz1); 
      float azimuth_B1 = atan2(By1, Bx1);

      float maxTorque = 1.0;  // Max torque when rotation is fast
      float minTorque = 0.1;  // Min torque to avoid stopping abruptly
      float criticalSpeed = 1.0; // Speed at which torque starts to decrease
      
      do{
        unsigned long currentTime = millis();
        dt = (currentTime - lastTime) / 1000.0;
        demagnetization();
        GetCalMagMeasurements(&Bx2, &By2, &Bz2);
        azimuth_B2 = normalizeAngle(atan2(By2, Bx2)); 
        
        omega_z2 = calAngularVelocity(azimuth_B1,azimuth_B2,dt); // angular velocity in rad/s [0; 2Pi]

        Serial1.print("Prev_deg: ");   
        Serial1.print(azimuth_B1*180/PI,4); 
        Serial1.print(" Current_deg: ");   
        Serial1.print(azimuth_B2*180/PI,4); 
        Serial1.print(" Vel deg/s: ");   
        Serial1.print(omega_z2*180/PI,4); 

        int direction = (omega_z2 >= 0) ? 1 : -1;
        omega_z2 = fabs(omega_z2); 
        acceleration = (omega_z2-omega_z1) / (dt);

        float k = maxTorque - (maxTorque - minTorque) * fmin(abs(omega_z2) / criticalSpeed, 1.0);

        if (direction>=0){
          azimuth_M = normalizeAngle(azimuth_B2-PI/2); // torque should oppose conterclockwise rotation
        }else if(direction<0){
          azimuth_M = normalizeAngle(azimuth_B2+PI/2); // torque should oppose conterclockwise rotation
        }

        Serial1.print(" M deg :");   
        Serial1.println(azimuth_M*180/PI,4); 

        Mx = cos(azimuth_M) * k; 
        My = sin(azimuth_M) * k;

        float theta_M = normalizeAngle(atan2(My, Mx));
        if(theta_M>0 && theta_M<=(PI/2)){
          TriggerCoils(Coil_X_positive, Mx); 
          TriggerCoils(Coil_Y_positive, My); 
        }else if(theta_M>(PI/2) && theta_M<=PI){
          TriggerCoils(Coil_X_negative, Mx);
          TriggerCoils(Coil_Y_positive, My); 
        }else if(theta_M> (PI) && theta_M<=(3*PI/2)){
          TriggerCoils(Coil_X_negative, Mx); 
          TriggerCoils(Coil_Y_negative,My);  
        }else if(theta_M>(3*PI/2) && theta_M<=(2*PI)){
          TriggerCoils(Coil_X_positive, Mx); 
          TriggerCoils(Coil_Y_negative, My); 
        }

        // if (Mx >= 0) {
        //     TriggerCoils(Coil_X_positive, Mx);
        // } else {
        //     TriggerCoils(Coil_X_negative, -Mx);
        // }

        // if (My >= 0) {
        //     TriggerCoils(Coil_Y_positive, My);
        // } else {
        //     TriggerCoils(Coil_Y_negative, -My);
        // }

        // Serial1.print(Bx2,4);
        // Serial1.print(";");
        // Serial1.print(By2,4);
        // Serial1.print(";");
        // Serial1.print(Bz2,4);
        // Serial1.print(";");
        // Serial1.print(omega_z2,4); //rad/s
        // Serial1.print(";");
        // Serial1.print(acceleration,4); 
        // Serial1.print(";");
        // Serial1.print((azimuth_M*180/PI),4); 
        // Serial1.print(";");
        // Serial1.print(Mx,4);
        // Serial1.print(";");
        // Serial1.print(My,4);
        // Serial1.print(";");
        // Serial1.println(normalizeAngle(azimuth_B2)*180/PI,4);

        Bx1 = Bx2; 
        By1 = By2; 
        Bz1 = Bz2; 
        azimuth_B1 = azimuth_B2;
        omega_z1 = omega_z2; 
        lastTime = currentTime;
        delay(2000);
          if (checkForExitCommand()) {
              TurnOff(); 
              return; 
            }
      }while(omega_z2>=0.1); 
      TurnOff();
      Serial1.println("Detumbling completed");
      currentState = IDLE;
  }

  void PDcontroller(int InputAngleDeg){ //positive angle = clockwise, negative to conterclockwise 
    float Kp = 0.1; 
    float Kd = 0.05; 

    float Bx1 = 0, By1 = 0, Bz1 = 0;
    float Bx2, By2, Bz2;
    float initialAngle=0, currentAngle=0, error=0, derivative=0, previousError=0, azimuth_M=0;
    unsigned long lastTime = millis();
    float dt;

    demagnetization(); 
    GetCalMagMeasurements(&Bx1, &By1, &Bz1);
    initialAngle = normalizeAngle(atan2(By1, Bx1));

    float InputAngleRad = InputAngleDeg * PI / 180.0;
    float desiredAngle = normalizeAngle(initialAngle + InputAngleRad);

    do{ 
      // Serial1.print("Initial:");   
      // Serial1.print(initialAngle*180/PI,4); 
      // Serial1.print(" Desired:");   
      // Serial1.print(desiredAngle*180/PI,4); 

      unsigned long currentTime = millis();
      dt = (currentTime - lastTime) / 1000.0;

      demagnetization();
      GetCalMagMeasurements(&Bx2, &By2, &Bz2);
      currentAngle = normalizeAngle(atan2(By2, Bx2));

      Serial1.print(" Current:");   
      Serial1.print(currentAngle*180/PI,4); 

      float difference = desiredAngle-currentAngle; //[-2Pi; 2Pi]
      error = fmod(difference + M_PI, 2 * M_PI);
      if(error<0){
        error += 2 * M_PI;
      }else{
        error -= M_PI;
      }

      // Serial1.print(" Error:");   
      // Serial1.println(error*180/PI,4); 

      derivative = (error - previousError) / dt;

      float controlMagnitude = Kp * error + Kd * derivative;

      int direction = (difference >= 0) ? 1 : -1;

      if (direction >= 0) {
            azimuth_M = normalizeAngle(currentAngle + M_PI / 2); // Rotate to the clockwise perpendicular
        } else {
            azimuth_M = normalizeAngle(currentAngle - M_PI / 2); // Rotate to the counterclockwise perpendicular
        }

        float Mx = cos(azimuth_M) * fabs(controlMagnitude);
        float My = sin(azimuth_M) * fabs(controlMagnitude);

        if (Mx >= 0) {
            TriggerCoils(Coil_X_positive, Mx);
        } else {
            TriggerCoils(Coil_X_negative, -Mx);
        }

        if (My >= 0) {
            TriggerCoils(Coil_Y_positive, My);
        } else {
            TriggerCoils(Coil_Y_negative, -My);
        }

        Serial1.print(Bx2,4);
        Serial1.print(";");
        Serial1.print(By2,4);
        Serial1.print(";");
        Serial1.print(Bz2,4);
        Serial1.print(";");
        Serial1.print(desiredAngle*180/PI,4); 
        Serial1.print(";");
        Serial1.print(currentAngle*180/PI,4); 
        Serial1.print(";");
        Serial1.print(error*180/PI,4);
        Serial1.print(";");
        Serial1.print(Mx,4);
        Serial1.print(";");
        Serial1.print(My,4);
        Serial1.print(";");
        Serial1.println((azimuth_M*180/PI),4); 

        previousError = error;
        lastTime = currentTime;

        delay(150);
        if (checkForExitCommand()) {
              TurnOff(); 
              return; 
            }
    } while(fabs(error) > (1 * PI / 180.0)); 
        Serial1.println("Orientation adjustment completed");
        TurnOff();
  }


void executeCommand(String command) {
  char commandType = command.charAt(0);
  int value = command.substring(1).toInt(); 

  switch (commandType) {
        case 'X':
          randomExcitation(Coil_X_positive, Coil_X_negative, value); 
          currentState = EXECUTING_COMMAND;
          break;
        case 'Y':
          randomExcitation(Coil_Y_negative, Coil_Y_positive, value); 
          currentState = EXECUTING_COMMAND;
          break;
        case 'T':

          do{
          if(value == 1)
          TriggerCoils(Coil_X_positive, 100); // Coil X generates dipole moment to its left , green led becomes north 
          else if (value ==2)
          {
            TriggerCoils(Coil_X_negative, 100); //Coil X generates dipole moment to its  right, red led is north 
          }else if (value==3)
          {
            TriggerCoils(Coil_Y_negative, 100);  //y axis coil - generate dipole moment to its green diode direction as if green diode is north
          }else if (value ==4)
          {
           TriggerCoils(Coil_Y_positive, 100);  //y axis coil + 
          }
          currentState = EXECUTING_COMMAND;
          if (checkForExitCommand()) {
              TurnOff(); 
              return; 
            }
          }while(commandType!= 'D');
          break;
        case 'R':
          PDcontroller(value); 
          currentState = EXECUTING_COMMAND;
          break;
        case 'C': 
          Calibrate(); 
          currentState = EXECUTING_COMMAND;
          break; 
        case 'M':
          currentState = READING;
          break;
        case 'D': 
          demagnetization();
          Serial1.println("Demagnetization complited");
          currentState = EXECUTING_COMMAND;
          break; 
        case 'S': 
          Bdot(); 
          break;
        default:
          Serial1.println("Unknown command.");
          break;
  }
  currentState = IDLE; 
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
    command.trim(); 
    if (command.length() > 0) { 
      handleCommand(command);
    }
  }
  if (currentState == READING) {
    ReadDataCal(); 
  }
}






