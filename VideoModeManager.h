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

#include "Types.h"

class VideoModeManager {
public:
    void begin();
    void saveIfNeeded();

    void set(VideoMode mode);
    void next();
    void previous();

    VideoMode current() const { return currentMode_; }

private:
    VideoMode currentMode_ = VID_50HZ;
    unsigned long lastChangeAt_ = 0;

    void updateLeds() const;
    void blinkSaved() const;
};

extern VideoModeManager videoModeManager;
