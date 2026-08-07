/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include <Arduino.h>
#include <EEPROM.h>

#include "Config.h"
#include "Debug.h"
#include "VideoModeManager.h"

VideoModeManager videoModeManager;

namespace {
#if defined(MODE_LED_R_PIN) || defined(MODE_LED_G_PIN)
constexpr byte kModeLedColors[][2] = {
    MODE_LED_50HZ_COLOR,
    MODE_LED_60HZ_COLOR,
};
#endif
}

void VideoModeManager::updateLeds() const {
#if defined(MODE_LED_R_PIN) || defined(MODE_LED_G_PIN)
    const byte* colors = kModeLedColors[currentMode_];

#ifdef MODE_LED_R_PIN
    byte red = colors[0];
#ifdef MODE_LED_COMMON_ANODE
    red = 255 - red;
#endif
    analogWrite(MODE_LED_R_PIN, red);
#endif

#ifdef MODE_LED_G_PIN
    byte green = colors[1];
#ifdef MODE_LED_COMMON_ANODE
    green = 255 - green;
#endif
    digitalWrite(MODE_LED_G_PIN, green);
#endif
#endif

#ifdef MODE_LED_SINGLE_PIN
    for (uint8_t i = 0; i < static_cast<uint8_t>(currentMode_) + 1U; ++i) {
        digitalWrite(MODE_LED_SINGLE_PIN, LOW);
        delay(40);
        digitalWrite(MODE_LED_SINGLE_PIN, HIGH);
        delay(80);
    }
#endif
}

void VideoModeManager::blinkSaved() const {
#if defined(MODE_LED_R_PIN) || defined(MODE_LED_G_PIN)
    byte off = 0;
#ifdef MODE_LED_COMMON_ANODE
    off = 255 - off;
#endif
#ifdef MODE_LED_R_PIN
    digitalWrite(MODE_LED_R_PIN, off);
#endif
#ifdef MODE_LED_G_PIN
    digitalWrite(MODE_LED_G_PIN, off);
#endif
    delay(200);
    updateLeds();
#endif

#ifdef MODE_LED_SINGLE_PIN
    digitalWrite(MODE_LED_SINGLE_PIN, LOW);
    delay(500);
    digitalWrite(MODE_LED_SINGLE_PIN, HIGH);
#endif
}

void VideoModeManager::begin() {
#ifdef MODE_LED_R_PIN
    pinMode(MODE_LED_R_PIN, OUTPUT);
#endif
#ifdef MODE_LED_G_PIN
    pinMode(MODE_LED_G_PIN, OUTPUT);
#endif
#ifdef MODE_LED_SINGLE_PIN
    pinMode(MODE_LED_SINGLE_PIN, OUTPUT);
#endif

    pinMode(VIDEOMODE_PIN, OUTPUT);
    currentMode_ = VID_50HZ;

#ifdef MODE_ROM_OFFSET
    const byte storedMode = EEPROM.read(MODE_ROM_OFFSET);
    debug(F("Loaded video mode from EEPROM: "));
    debugln(storedMode);
    if (storedMode < VID_MODES_NO) {
        currentMode_ = static_cast<VideoMode>(storedMode);
    }
#endif

    set(currentMode_);
    lastChangeAt_ = 0;
}

void VideoModeManager::saveIfNeeded() {
#ifdef MODE_ROM_OFFSET
    if (lastChangeAt_ == 0 || millis() - lastChangeAt_ < MODE_SAVE_DELAY) {
        return;
    }

    debug(F("Saving video mode to EEPROM: "));
    debugln(currentMode_);

    const byte storedMode = EEPROM.read(MODE_ROM_OFFSET);
    if (storedMode != static_cast<byte>(currentMode_)) {
        EEPROM.write(MODE_ROM_OFFSET, static_cast<byte>(currentMode_));
    } else {
        debugln(F("Mode unchanged, not saving"));
    }

    lastChangeAt_ = 0;
    blinkSaved();
#endif
}

void VideoModeManager::set(VideoMode mode) {
    switch (mode) {
        case VID_60HZ: digitalWrite(VIDEOMODE_PIN, LOW); break;
        case VID_50HZ:
        default:       digitalWrite(VIDEOMODE_PIN, HIGH); break;
    }

    currentMode_ = mode;
    updateLeds();
    lastChangeAt_ = millis();
}

void VideoModeManager::next() {
    set(static_cast<VideoMode>((currentMode_ + 1) % VID_MODES_NO));
}

void VideoModeManager::previous() {
    set(static_cast<VideoMode>((currentMode_ + VID_MODES_NO - 1) % VID_MODES_NO));
}
