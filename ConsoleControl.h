/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#pragma once

#include "Config.h"
#include "Types.h"

void assertReset();
void releaseReset();
void assertPause();
void releasePause();

void updateResetButton();
void updatePauseButton(bool gamepadStartPressed);

void pulseReset();
void pulsePause();
bool isThActive();

#ifdef FMSOUND_OUT_PIN
void initializeFmSound();
void switchFmSoundAndReset(SwitchMode mode);
#endif
