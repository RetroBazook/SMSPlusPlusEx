#pragma once
#include "Types.h"

void setupVideoMode();
void update_mode_leds();
void save_mode();
void set_mode(VideoMode mode);
void change_mode(int increment);
void next_mode();
void prev_mode();
VideoMode getCurrentVideoMode();
