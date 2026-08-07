/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include <EEPROM.h>

#include "Config.h"
#include "ConsoleController.h"
#include "Debug.h"
#include "VideoModeManager.h"

ConsoleController consoleController;

#ifdef RESET_IN_PIN
byte ConsoleController::readResetInput() {
#ifdef ARDUINO_NANO
    return analogRead(RESET_IN_PIN) > ANALOG_IN_THRESHOLD ? HIGH : LOW;
#else
    return digitalRead(RESET_IN_PIN);
#endif
}
#endif

#ifdef PAUSE_IN_PIN
byte ConsoleController::readPauseInput(bool gamepadStartPressed) {
    if (gamepadStartPressed) {
        return LOW;
    }
#ifdef ARDUINO_NANO
    return analogRead(PAUSE_IN_PIN) > ANALOG_IN_THRESHOLD ? HIGH : LOW;
#else
    return digitalRead(PAUSE_IN_PIN);
#endif
}
#endif

void ConsoleController::holdReset() {
#ifdef RESET_OUT_PIN
    pinMode(RESET_OUT_PIN, OUTPUT);
#endif
}

void ConsoleController::releaseReset() {
#ifdef RESET_OUT_PIN
    pinMode(RESET_OUT_PIN, INPUT);
#endif
}

void ConsoleController::pulseReset() {
    debugln(F("Resetting console"));
    holdReset();
    delay(PULSE_LEN);
    releaseReset();
}

void ConsoleController::holdPause() {
#ifdef PAUSE_OUT_PIN
    pinMode(PAUSE_OUT_PIN, OUTPUT);
#endif
}

void ConsoleController::releasePause() {
#ifdef PAUSE_OUT_PIN
    pinMode(PAUSE_OUT_PIN, INPUT);
#endif
}

void ConsoleController::pulsePause() {
    debugln(F("Pausing console"));
    holdPause();
    delay(PULSE_LEN);
    releasePause();
}

void ConsoleController::initializeInputs() {
#if defined(PAUSE_IN_PIN) && !defined(ARDUINO_NANO)
    pinMode(PAUSE_IN_PIN, INPUT_PULLUP);
#endif
    releasePause();

#if defined(RESET_IN_PIN) && !defined(ARDUINO_NANO)
    pinMode(RESET_IN_PIN, INPUT_PULLUP);
#endif
}

#ifdef FMSOUND_OUT_PIN
void ConsoleController::initializeFmSound() {
    pinMode(FMSOUND_OUT_PIN, OUTPUT);
    pinMode(JAP_FMSOUND_OUT_PIN, OUTPUT);

    currentFmMode_ = static_cast<SwitchMode>(EEPROM.read(FM_MOD_OFFSET));
    if (currentFmMode_ > JAP_FM) {
        currentFmMode_ = PSG;
    }

    digitalWrite(FMSOUND_OUT_PIN, LOW);
    digitalWrite(JAP_FMSOUND_OUT_PIN, LOW);
    delayMicroseconds(100);

    switch (currentFmMode_) {
        case FM:     digitalWrite(FMSOUND_OUT_PIN, HIGH); break;
        case JAP_FM: digitalWrite(JAP_FMSOUND_OUT_PIN, HIGH); break;
        case PSG:
        default: break;
    }
}

void ConsoleController::switchFmSoundAndReset(SwitchMode mode) {
    if (currentFmMode_ == mode) {
        return;
    }

    EEPROM.write(FM_MOD_OFFSET, static_cast<uint8_t>(mode));
    pulseReset();
}
#endif

bool ConsoleController::isThActive() const {
    return (PIND & (1U << SELECT_PAD_PIN)) == 0;
}

void ConsoleController::updateResetButton() {
#ifdef RESET_IN_PIN
    static byte debounceLevel = LOW;
    static bool wasPressed = false;
    static unsigned long lastTransitionAt = 0;
    static unsigned long pressedAt = 0;
    static unsigned int holdCycles = 0;

    const byte level = readResetInput();
    const unsigned long now = millis();

    if (level != debounceLevel) {
        debounceLevel = level;
        lastTransitionAt = now;
        return;
    }
    if (now - lastTransitionAt <= DEBOUNCE_MS) {
        return;
    }

    const bool isPressed = level == LOW;
    if (isPressed && !wasPressed) {
        pressedAt = now;
        holdCycles = 0;
    } else if (!isPressed && wasPressed) {
        if (holdCycles == 0) {
            debugln(F("Reset button pushed for a short time"));
            pulseReset();
        }
    } else if (isPressed) {
        // Preserved from the original implementation on purpose.
        if (now % pressedAt >= LONGPRESS_LEN * (holdCycles + 1U)) {
            debugln(F("Reset button held"));
            ++holdCycles;
            videoModeManager.next();
        }
    }

    wasPressed = isPressed;
#else
#warning "RESET button handling disabled"
#endif
}

void ConsoleController::updatePauseButton(bool gamepadStartPressed) {
#ifdef PAUSE_IN_PIN
    static byte debounceLevel = LOW;
    static bool wasPressed = false;
    static unsigned long lastTransitionAt = 0;
    static unsigned long pressedAt = 0;
    static unsigned int holdCycles = 0;

    const byte level = readPauseInput(gamepadStartPressed);
    const unsigned long now = millis();

    if (level != debounceLevel) {
        debounceLevel = level;
        lastTransitionAt = now;
        return;
    }
    if (now - lastTransitionAt <= DEBOUNCE_MS) {
        return;
    }

    const bool isPressed = level == LOW;
    if (isPressed && !wasPressed) {
        pressedAt = now;
        holdCycles = 0;
    } else if (!isPressed && wasPressed) {
        if (holdCycles == 0) {
            debugln(F("Pause button pushed for a short time"));
#ifdef RESET_ON_PAUSE
            pulseReset();
#else
            pulsePause();
#endif
        }
    } else if (!gamepadStartPressed && isPressed &&
               now - pressedAt >= LONGPRESS_LEN * (holdCycles + 1U)) {
        debugln(F("Pause button held"));
        ++holdCycles;
        videoModeManager.next();
    }

    wasPressed = isPressed;
#else
#warning "PAUSE button handling disabled"
#endif
}
