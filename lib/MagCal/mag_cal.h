#ifndef _H_MAG_CAL
#define _H_MAG_CAL
#ifdef DEBUG
  # define DEBUG_PRINT(...) Serial1.print(__VA_ARGS__)
  # define DEBUG_PRINTLN(...) Serial1.println(__VA_ARGS__)
#else
  # define DEBUG_PRINT(...) do {} while (0);
  # define DEBUG_PRINTLN(...) do {} while (0);
#endif
typedef float T_MATRIX;

int MagSensorInitCalibration();
int MagSensorUpdateCalibration(float *Bp);
int MagSensorCalculateCalibrationResult(void);
int MagSensorGetCalibrationResult(float *V_hard_iron_offset);
T_MATRIX MagSensorGetMagneticFieldStrength_uT();
#endif