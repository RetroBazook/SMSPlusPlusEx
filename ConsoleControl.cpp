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
#include "ConsoleControl.h"
#include "Debug.h"
#include "VideoMode.h"

namespace {
#ifdef FMSOUND_OUT_PIN
SwitchMode currentFmMode = PSG;
#endif

#ifdef RESET_IN_PIN
byte readResetInput() {
#ifdef ARDUINO_NANO
    return analogRead(RESET_IN_PIN) > ANALOG_IN_THRESHOLD ? HIGH : LOW;
#else
    return digitalRead(RESET_IN_PIN);
#endif
}
#endif

#ifdef PAUSE_IN_PIN
byte readPauseInput(bool gamepadStartPressed) {
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
}  // namespace

void assertReset() {
#ifdef RESET_OUT_PIN
    // RESET is active-low. OUTPUT drives the already-low pin; INPUT releases it
    // to the console pull-up, emulating an open-collector output.
    pinMode(RESET_OUT_PIN, OUTPUT);
#endif
}

void releaseReset() {
#ifdef RESET_OUT_PIN
    pinMode(RESET_OUT_PIN, INPUT);
#endif
}

void assertPause() {
#ifdef PAUSE_OUT_PIN
    pinMode(PAUSE_OUT_PIN, OUTPUT);
#endif
}

void releasePause() {
#ifdef PAUSE_OUT_PIN
    pinMode(PAUSE_OUT_PIN, INPUT);
#endif
}

#ifdef FMSOUND_OUT_PIN
void initializeFmSound() {
    pinMode(FMSOUND_OUT_PIN, OUTPUT);
    pinMode(JAP_FMSOUND_OUT_PIN, OUTPUT);

    currentFmMode = static_cast<SwitchMode>(EEPROM.read(FM_MOD_OFFSET));
    if (currentFmMode > JAP_FM) {
        currentFmMode = PSG;
    }

    digitalWrite(FMSOUND_OUT_PIN, LOW);
    digitalWrite(JAP_FMSOUND_OUT_PIN, LOW);
    delayMicroseconds(100);

    switch (currentFmMode) {
        case FM:
            digitalWrite(FMSOUND_OUT_PIN, HIGH);
            break;
        case JAP_FM:
            digitalWrite(JAP_FMSOUND_OUT_PIN, HIGH);
            break;
        case PSG:
        default:
            break;
    }
}

void switchFmSoundAndReset(SwitchMode mode) {
    if (currentFmMode == mode) {
        return;
    }

    EEPROM.write(FM_MOD_OFFSET, static_cast<uint8_t>(mode));
    pulseReset();
}
#endif

bool isThActive() {
    return (PIND & (1U << SELECT_PAD_PIN)) == 0;  // TH is active-low.
}

void updateResetButton() {
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
        // Intentionally preserves the original SMS++ expression/behaviour.
        if (now % pressedAt >= LONGPRESS_LEN * (holdCycles + 1U)) {
            debugln(F("Reset button held"));
            ++holdCycles;
            nextVideoMode();
        }
    }

    wasPressed = isPressed;
#else
    #warning "RESET button handling disabled"
#endif
}

void updatePauseButton(bool gamepadStartPressed) {
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
        nextVideoMode();
    }

    wasPressed = isPressed;
#else
    #warning "PAUSE button handling disabled"
#endif
}

void pulseReset() {
    debugln(F("Resetting console"));
    assertReset();
    delay(PULSE_LEN);
    releaseReset();
}

void pulsePause() {
    debugln(F("Pausing console"));
    assertPause();
    delay(PULSE_LEN);
    releasePause();
}
