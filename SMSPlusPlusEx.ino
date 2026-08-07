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

#include "AutoFireManager.h"
#include "ComboHandler.h"
#include "Config.h"
#include "ConsoleController.h"
#include "Debug.h"
#include "PadController.h"
#include "PadHandler.h"
#include "PadPort.h"
#include "RemappingManager.h"
#include "StatusLed.h"
#include "VideoModeManager.h"

namespace {
// Composition root: dependencies are assembled here and nowhere else.
VideoModeManager videoMode;
StatusLed statusLed;
PadPort padPort;
PadController pad(padPort);
ConsoleController console(videoMode);
RemappingManager remapping(pad, statusLed);
AutoFireManager autoFire;
ComboHandler combos(console, videoMode, remapping, autoFire);
PadHandler padHandler(pad, console, remapping, autoFire, combos);

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
    if (pad.isDetected()) {
        return true;
    }

    pad.detect();
    if (!pad.isDetected()) {
        return false;
    }

    indicateDetectedPad(pad.type());
    return true;
}
}  // namespace

void setup() {
#ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
#endif
    debugln(F("Starting up..."));

    // Keep the console reset until every I/O path is configured.
    console.holdReset();

#ifdef PAD_LED_PIN
    pinMode(PAD_LED_PIN, OUTPUT);
#endif

    videoMode.begin();
    remapping.begin();
    pad.begin();
    console.initializeInputs();

#ifdef FMSOUND_OUT_PIN
    console.initializeFmSound();
#endif

    console.releaseReset();
}

void loop() {
    if (!ensureGamepadDetected()) {
        return;
    }

    if (remapping.isActive()) {
        remapping.update();
    } else {
        console.updateResetButton();
        padHandler.update();
        videoMode.saveIfNeeded();
    }

    statusLed.update();
}
