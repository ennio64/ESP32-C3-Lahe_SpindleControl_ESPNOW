#ifndef SPINDLE_CONTROL_H
#define SPINDLE_CONTROL_H

#include "Machine_State.h"

namespace SpindleControl {

void earlyInitLEDs();
void begin();
void updateMultiplier();
void updateFromEncoder();
void updateCommand();
void updateFreezeState();

void showBeforeReadyLEDs();
void showMultiplierLEDs();
void showSpindleRotationLEDs(bool cw);
void showAlarmStateLEDs();

bool isCW();
bool isRotationFrozen();

}

#endif