/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "StatusLed.h"

StatusLed statusLed;

void StatusLed::blinkBlocking(uint8_t blinkCount) {
    for (uint8_t i = 0; i < blinkCount * 2U; ++i) {
        ledState_ = !ledState_;
        digitalWrite(LED_BUILTIN, ledState_ ? HIGH : LOW);
        delay(kBlinkDurationMs);
    }

    ledState_ = HIGH;
    digitalWrite(LED_BUILTIN, HIGH);
}

void StatusLed::startContinuousBlink() {
    if (remainingToggles_ != 0) {
        return;
    }

    ledState_ = LOW;
    digitalWrite(LED_BUILTIN, LOW);
    remainingToggles_ = -1;
    lastBlinkAt_ = millis();
}

void StatusLed::stopBlink() {
    remainingToggles_ = 0;
    ledState_ = HIGH;
    digitalWrite(LED_BUILTIN, HIGH);
}

void StatusLed::update() {
    if (remainingToggles_ == 0 || millis() - lastBlinkAt_ < kBlinkDurationMs) {
        return;
    }

    lastBlinkAt_ = millis();
    ledState_ = !ledState_;
    digitalWrite(LED_BUILTIN, ledState_ ? HIGH : LOW);

    if (remainingToggles_ > 0 && --remainingToggles_ == 0) {
        ledState_ = HIGH;
        digitalWrite(LED_BUILTIN, HIGH);
    }
}
