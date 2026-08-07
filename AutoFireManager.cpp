/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "AutoFireManager.h"

namespace {
constexpr uint8_t kHitsPerSecond[AF_MODES_NO] = {5, 10, 15, 20};
}

bool AutoFireManager::isOn(ButtonState& state) {
    const unsigned long intervalMs = 1000UL / kHitsPerSecond[state.rate];

    if (state.pressStartedAt == 0) {
        state.pressStartedAt = millis();
        return false;  // Preserves the original first-call behaviour.
    }

    return ((millis() - state.pressStartedAt) / intervalMs) % 2U == 0;
}

void AutoFireManager::cycle(ButtonState& state) {
    state.rate = static_cast<AutoFireRate>((state.rate + 1) % AF_MODES_NO);
}

uint8_t AutoFireManager::convertToMasterSystem(
    uint16_t megaDrivePad,
    const ButtonMapping& mapping) {

    uint8_t smsPad = 0;

    if (megaDrivePad & MD_BTN_UP)    smsPad |= SMS_BTN_UP;
    if (megaDrivePad & MD_BTN_DOWN)  smsPad |= SMS_BTN_DOWN;
    if (megaDrivePad & MD_BTN_LEFT)  smsPad |= SMS_BTN_LEFT;
    if (megaDrivePad & MD_BTN_RIGHT) smsPad |= SMS_BTN_RIGHT;

    const bool autoLeftPressed =
        megaDrivePad & (mapping.autoBoth() | mapping.autoLeft());
    const bool autoRightPressed =
        megaDrivePad & (mapping.autoBoth() | mapping.autoRight());

    if (autoLeftPressed) {
        if (isOn(left_)) {
            smsPad |= SMS_BTN_B1;
        }
    } else {
        if (megaDrivePad & (mapping.left() | mapping.both())) {
            smsPad |= SMS_BTN_B1;
        }
        left_.pressStartedAt = 0;
    }

    if (autoRightPressed) {
        if (isOn(right_)) {
            smsPad |= SMS_BTN_B2;
        }
    } else {
        if (megaDrivePad & (mapping.right() | mapping.both())) {
            smsPad |= SMS_BTN_B2;
        }
        right_.pressStartedAt = 0;
    }

    return smsPad;
}

void AutoFireManager::cycleLeft() {
    cycle(left_);
}

void AutoFireManager::cycleRight() {
    cycle(right_);
}

void AutoFireManager::cycleBoth() {
    cycle(left_);
    cycle(right_);
}
