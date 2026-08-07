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

#include "AutoFire.h"
#include "Remapping.h"
#include "Types.h"

namespace {
struct AutoFireState {
    AutoFireRate rate = AF_MEDIUM;
    unsigned long pressStartedAt = 0;
};

constexpr byte kHitsPerSecond[AF_MODES_NO] = {5, 10, 15, 20};
AutoFireState leftAutoFire;
AutoFireState rightAutoFire;

bool isAutoFireOn(AutoFireState& state) {
    const unsigned long intervalMs = 1000UL / kHitsPerSecond[state.rate];

    if (state.pressStartedAt == 0) {
        state.pressStartedAt = millis();
        return false;  // Preserves original first-call behaviour.
    }

    return ((millis() - state.pressStartedAt) / intervalMs) % 2U == 0;
}

void cycleRate(AutoFireState& state) {
    state.rate = static_cast<AutoFireRate>((state.rate + 1) % AF_MODES_NO);
}
}  // namespace

byte convertMegaDriveToMasterSystem(word megaDrivePad) {
    byte smsPad = 0;

    if (megaDrivePad & MD_BTN_UP)    smsPad |= SMS_BTN_UP;
    if (megaDrivePad & MD_BTN_DOWN)  smsPad |= SMS_BTN_DOWN;
    if (megaDrivePad & MD_BTN_LEFT)  smsPad |= SMS_BTN_LEFT;
    if (megaDrivePad & MD_BTN_RIGHT) smsPad |= SMS_BTN_RIGHT;

    const bool autoLeftPressed =
        megaDrivePad & (getMappedAutoBothButton() | getMappedAutoLeftButton());
    const bool autoRightPressed =
        megaDrivePad & (getMappedAutoBothButton() | getMappedAutoRightButton());

    if (autoLeftPressed) {
        if (isAutoFireOn(leftAutoFire)) smsPad |= SMS_BTN_B1;
    } else {
        if (megaDrivePad & (getMappedLeftButton() | getMappedBothButton())) {
            smsPad |= SMS_BTN_B1;
        }
        leftAutoFire.pressStartedAt = 0;
    }

    if (autoRightPressed) {
        if (isAutoFireOn(rightAutoFire)) smsPad |= SMS_BTN_B2;
    } else {
        if (megaDrivePad & (getMappedRightButton() | getMappedBothButton())) {
            smsPad |= SMS_BTN_B2;
        }
        rightAutoFire.pressStartedAt = 0;
    }

    return smsPad;
}

void cycleAutoFireLeft() {
    cycleRate(leftAutoFire);
}

void cycleAutoFireRight() {
    cycleRate(rightAutoFire);
}

void cycleAutoFireBoth() {
    cycleRate(leftAutoFire);
    cycleRate(rightAutoFire);
}
