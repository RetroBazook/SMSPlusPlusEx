#pragma once
#include "Config.h"
#include "Types.h"

void enableReset();
void disableReset();
void enablePause();
void disablePause();
void handle_reset_button();
void handle_pause_button(bool gamepad_start_pressed);
void reset_console();
void pause_console();
bool isThActive();

#ifdef FMSOUND_OUT_PIN
void setupFmSoundSwitchState();
void setFmSoundSwitchStateAndReboot(SwitchMode mode);
#endif
