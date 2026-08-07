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
class AutoFireManager; class ConsoleController; class RemappingManager; class VideoModeManager;
class ComboHandler {
public:
    ComboHandler(ConsoleController& console, VideoModeManager& video, RemappingManager& remap, AutoFireManager& autoFire)
        : console_(console), video_(video), remap_(remap), autoFire_(autoFire) {}
    void update(uint16_t padStatus);
private:
    ConsoleController& console_; VideoModeManager& video_; RemappingManager& remap_; AutoFireManager& autoFire_;
    unsigned long lastHandledAt_=0;
    static bool pressed(uint16_t status,uint16_t combo){return (status&combo)==combo;}
    void handled(){lastHandledAt_=millis();}
};
