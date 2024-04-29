#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include "mag_cal.h"
#define XtX_RANK 4

extern SFE_MMC5983MA myMag; 
extern int csPin;           
typedef float T_MATRIX;
extern T_MATRIX HardIronoffset[XtX_RANK]; 

void initializeMagnetometer();
void GetMagMeasurements(float *scaledX, float *scaledY, float *scaledZ) ;
bool updateOffset(uint32_t *offsetX, uint32_t *offsetY, uint32_t *offsetZ);
void Calibrate();
void readData();
void ReadDataCal();


void TriggerCoils(int coil,  float a);
void demagnetization(int wire1, int wire2); 
void randomExcitation(int wire1, int wire2, int pwm);

#endif
