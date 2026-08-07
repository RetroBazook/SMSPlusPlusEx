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
#include <Arduino.h>
#include "Config.h"
#include "DebouncedButton.h"
#include "Types.h"
class VideoModeManager;
class ConsoleController {
public:
    explicit ConsoleController(VideoModeManager& video);
    void holdReset(); void releaseReset(); void pulseReset();
    void holdPause(); void releasePause(); void pulsePause();
    void initializeInputs(); void updateResetButton(); void updatePauseButton(bool gamepadStartPressed);
#ifdef FMSOUND_OUT_PIN
    void initializeFmSound(); void switchFmSoundAndReset(SwitchMode mode);
#endif
private:
    VideoModeManager& video_;
    DebouncedButton resetButton_;
    DebouncedButton pauseButton_;
#ifdef FMSOUND_OUT_PIN
    SwitchMode currentFmMode_=PSG;
#endif
#ifdef RESET_IN_PIN
    static byte readResetInput();
#endif
#ifdef PAUSE_IN_PIN
    static byte readPauseInput(bool gamepadStartPressed);
#endif
};
