#ifndef __MAGNETOTORQUER_HEADER__
#define __MAGNETOTORQUER_HEADER__

#include <cstdint>

void configurePins(); 

void TurnOff(); 

void showHelp();

void handleCommand(String command);

void executeCommand(String command);

int sgn(float val);

void Bdot();



#endif