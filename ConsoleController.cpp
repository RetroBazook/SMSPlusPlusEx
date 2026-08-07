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

using namespace FirmwareConfig;

ConsoleController::ConsoleController(VideoModeManager& videoMode)
    : video_(videoMode),
      resetButton_(
          Timing::DebounceMs,
          Timing::LongPressMs,
          DebouncedButton::LongPressClock::LegacyModulo),
      pauseButton_(Timing::DebounceMs, Timing::LongPressMs) {}

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
    delay(Timing::ConsolePulseMs);
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
    delay(Timing::ConsolePulseMs);
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

    currentFmMode_ = static_cast<SwitchMode>(EEPROM.read(EepromAddress::FmMode));
    if (currentFmMode_ > JAP_FM) {
        currentFmMode_ = PSG;
    }

    digitalWrite(FMSOUND_OUT_PIN, LOW);
    digitalWrite(JAP_FMSOUND_OUT_PIN, LOW);
    delayMicroseconds(100);

    if (currentFmMode_ == FM) {
        digitalWrite(FMSOUND_OUT_PIN, HIGH);
    } else if (currentFmMode_ == JAP_FM) {
        digitalWrite(JAP_FMSOUND_OUT_PIN, HIGH);
    }
}

void ConsoleController::switchFmSoundAndReset(SwitchMode mode) {
    if (currentFmMode_ == mode) {
        return;
    }

    EEPROM.write(EepromAddress::FmMode, static_cast<uint8_t>(mode));
    pulseReset();
}
#endif

void ConsoleController::updateResetButton() {
#ifdef RESET_IN_PIN
    switch (resetButton_.update(readResetInput())) {
        case DebouncedButton::Event::Released:
            if (resetButton_.holdCycles() == 0) {
                debugln(F("Reset button pushed for a short time"));
                pulseReset();
            }
            break;

        case DebouncedButton::Event::LongPress:
            debugln(F("Reset button held"));
            video_.next();
            break;

        default:
            break;
    }
#else
#warning "RESET button handling disabled"
#endif
}

void ConsoleController::updatePauseButton(bool gamepadStartPressed) {
#ifdef PAUSE_IN_PIN
    const auto event = pauseButton_.update(
        readPauseInput(gamepadStartPressed),
        !gamepadStartPressed);

    switch (event) {
        case DebouncedButton::Event::Released:
            if (pauseButton_.holdCycles() == 0) {
                debugln(F("Pause button pushed for a short time"));
#ifdef RESET_ON_PAUSE
                pulseReset();
#else
                pulsePause();
#endif
            }
            break;

        case DebouncedButton::Event::LongPress:
            debugln(F("Pause button held"));
            video_.next();
            break;

        default:
            break;
    }
#else
#warning "PAUSE button handling disabled"
#endif
}
