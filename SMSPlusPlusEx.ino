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
#include "ConsoleController.h"
#include "Debug.h"
#include "PadController.h"
#include "PadHandler.h"
#include "RemappingManager.h"
#include "StatusLed.h"
#include "VideoModeManager.h"

namespace {
void indicateDetectedPad(PadType type) {
    switch (type) {
        case PAD_SMS:     statusLed.blinkBlocking(2); break;
        case PAD_MD:      statusLed.blinkBlocking(3); break;
        case PAD_MD_6BTN: statusLed.blinkBlocking(6); break;
        case PAD_NONE:
        default: break;
    }
}

bool ensureGamepadDetected() {
    if (padController.isDetected()) {
        return true;
    }

    padController.detect();
    if (!padController.isDetected()) {
        return false;
    }

    indicateDetectedPad(padController.type());
    return true;
}
}  // namespace

void setup() {
#ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
#endif
    debugln(F("Starting up..."));

    // Keep the console in reset until every I/O path is configured.
    consoleController.holdReset();

#ifdef PAD_LED_PIN
    pinMode(PAD_LED_PIN, OUTPUT);
#endif

    videoModeManager.begin();
    remappingManager.begin();
    padController.begin();
    consoleController.initializeInputs();

#ifdef FMSOUND_OUT_PIN
    consoleController.initializeFmSound();
#endif

    consoleController.releaseReset();
}

void loop() {
    if (!ensureGamepadDetected()) {
        return;
    }

    if (remappingManager.isActive()) {
        remappingManager.update();
    } else {
        consoleController.updateResetButton();
        padHandler.update();
        videoModeManager.saveIfNeeded();
    }

    statusLed.update();
}
