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

#include "Config.h"
#include "ConsoleControl.h"
#include "Debug.h"
#include "PadHandler.h"
#include "PadProtocol.h"
#include "Remapping.h"
#include "VideoMode.h"

namespace {
void indicateDetectedPad(PadType padType) {
    switch (padType) {
        case PAD_SMS:     blinkBuiltInLed(2); break;
        case PAD_MD:      blinkBuiltInLed(3); break;
        case PAD_MD_6BTN: blinkBuiltInLed(6); break;
        case PAD_NONE:
        default: break;
    }
}

bool ensureGamepadDetected() {
    if (getPadType() != PAD_NONE) {
        return true;
    }

    detectGamepad();
    if (getPadType() == PAD_NONE) {
        return false;
    }

    indicateDetectedPad(getPadType());
    return true;
}
}  // namespace

void setup() {
#ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
#endif
    debugln(F("Starting up..."));

    // Hold the console in reset until every I/O path is configured.
    assertReset();

#ifdef PAD_LED_PIN
    pinMode(PAD_LED_PIN, OUTPUT);
#endif

    initializeVideoMode();
    initializeMapping();
    initializePadInput();
    initializeOutputTraces();
    initializeElectronicSwitch();

#if defined(PAUSE_IN_PIN) && !defined(ARDUINO_NANO)
    pinMode(PAUSE_IN_PIN, INPUT_PULLUP);
#endif
    releasePause();

#if defined(RESET_IN_PIN) && !defined(ARDUINO_NANO)
    pinMode(RESET_IN_PIN, INPUT_PULLUP);
#endif

#ifdef FMSOUND_OUT_PIN
    initializeFmSound();
#endif

    releaseReset();
}

void loop() {
    if (!ensureGamepadDetected()) {
        return;
    }

    if (isRemapping()) {
        updateRemapping();
    } else {
        updateResetButton();
        updatePad();
        saveVideoModeIfNeeded();
    }

    updateBlinkIndicator();
}
