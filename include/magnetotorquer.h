#ifndef __MAGNETOTORQUER_HEADER__
#define __MAGNETOTORQUER_HEADER__

#include <cstdint>

void TurnOff(); 

void Calibrate();

void readData();

void ReadDataCal();

void GetMagMeasurements(float *scaledX, float *scaledY, float *scaledZ);

bool updateOffset(uint32_t *offsetX, uint32_t *offsetY, uint32_t *offsetZ);

void demagnetization(int wire1, int wire2);

void randomExcitation(int wire1, int wire2, int pwm);

void TriggerCoils(int coilX, int coilY, float x, float y);

int sgn(float val);

void Bdot();



#endif