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
#include "PadProtocol.h"
#include "Remapping.h"

namespace {
constexpr uint8_t kMappingSize = 6;
constexpr uint8_t kThreeButtonMappingSize = 3;
constexpr unsigned long kRemapDebounceMs = 100;
constexpr unsigned long kBlinkDurationMs = 250;

const MdButton kDefaultMapping[kMappingSize] = {
    MD_BTN_B, MD_BTN_C, MD_BTN_A,
    MD_BTN_Y, MD_BTN_Z, MD_BTN_X,
};

MdButton mapping[kMappingSize] = {
    MD_BTN_B, MD_BTN_C, MD_BTN_A,
    MD_BTN_Y, MD_BTN_Z, MD_BTN_X,
};

int8_t remapIndex = -1;
bool threeButtonMode = false;
unsigned long remapBlockedUntil = 0;

unsigned long lastBlinkAt = 0;
bool ledState = HIGH;
int remainingToggles = 0;  // Negative means blink indefinitely.

BtnNumber buttonToNumber(MdButton button) {
    switch (button) {
        case MD_BTN_A: return BTN_NB_A;
        case MD_BTN_B: return BTN_NB_B;
        case MD_BTN_C: return BTN_NB_C;
        case MD_BTN_X: return BTN_NB_X;
        case MD_BTN_Y: return BTN_NB_Y;
        case MD_BTN_Z: return BTN_NB_Z;
        default:       return static_cast<BtnNumber>(0xFF);
    }
}

MdButton numberToButton(BtnNumber number) {
    switch (number) {
        case BTN_NB_A: return MD_BTN_A;
        case BTN_NB_B: return MD_BTN_B;
        case BTN_NB_C: return MD_BTN_C;
        case BTN_NB_X: return MD_BTN_X;
        case BTN_NB_Y: return MD_BTN_Y;
        case BTN_NB_Z: return MD_BTN_Z;
        default:       return static_cast<MdButton>(0);
    }
}

void saveMapping() {
    for (uint8_t i = 0; i < kMappingSize; ++i) {
        EEPROM.update(REMAP_OFFSET + i, static_cast<uint8_t>(buttonToNumber(mapping[i])));
    }
}

void restoreDefaultMapping() {
    for (uint8_t i = 0; i < kMappingSize; ++i) {
        mapping[i] = kDefaultMapping[i];
    }
    saveMapping();
}

MdButton getCompatibleButton(word padStatus) {
    switch (padStatus) {
        case MD_BTN_A: return MD_BTN_A;
        case MD_BTN_B: return MD_BTN_B;
        case MD_BTN_C: return MD_BTN_C;
        case MD_BTN_X: return threeButtonMode ? static_cast<MdButton>(0) : MD_BTN_X;
        case MD_BTN_Y: return threeButtonMode ? static_cast<MdButton>(0) : MD_BTN_Y;
        case MD_BTN_Z: return threeButtonMode ? static_cast<MdButton>(0) : MD_BTN_Z;
        default:       return static_cast<MdButton>(0);
    }
}

bool mappingAlreadyContains(MdButton button, uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        if (mapping[i] == button) {
            return true;
        }
    }
    return false;
}

void stopBlinkIndicator() {
    remainingToggles = 0;
    ledState = HIGH;
    digitalWrite(LED_BUILTIN, HIGH);
}

void startBlinkIndicator(int blinkCount = 0) {
    if (remainingToggles != 0) {
        return;
    }

    ledState = LOW;
    digitalWrite(LED_BUILTIN, LOW);
    remainingToggles = blinkCount > 0 ? blinkCount * 2 : -1;
    lastBlinkAt = millis();
}

void finishRemap() {
    saveMapping();
    stopBlinkIndicator();
    remapIndex = -1;
    threeButtonMode = false;
}

void mirrorThreeButtonMapping(MdButton button, uint8_t index) {
    switch (button) {
        case MD_BTN_A: mapping[index + 3] = MD_BTN_X; break;
        case MD_BTN_B: mapping[index + 3] = MD_BTN_Y; break;
        case MD_BTN_C: mapping[index + 3] = MD_BTN_Z; break;
        default: break;
    }
}
}  // namespace

void initializeMapping() {
    for (uint8_t i = 0; i < kMappingSize; ++i) {
        const auto stored = static_cast<BtnNumber>(EEPROM.read(REMAP_OFFSET + i));
        const MdButton button = numberToButton(stored);

        if (button == static_cast<MdButton>(0)) {
            restoreDefaultMapping();
            return;
        }

        mapping[i] = button;
    }
}

void beginFullRemap() {
    for (auto& button : mapping) {
        button = static_cast<MdButton>(0);
    }

    remapIndex = 0;
    threeButtonMode = false;
    remapBlockedUntil = 0;
    startBlinkIndicator();
}

void beginThreeButtonRemap() {
    beginFullRemap();
    threeButtonMode = true;
}

void updateRemapping() {
    if (remapIndex < 0 || millis() < remapBlockedUntil) {
        return;
    }

    const MdButton button = getCompatibleButton(readMegaDrivePad());
    if (button == static_cast<MdButton>(0)) {
        return;
    }

    const uint8_t duplicateCheckCount = threeButtonMode
        ? kThreeButtonMappingSize
        : static_cast<uint8_t>(remapIndex);
    if (mappingAlreadyContains(button, duplicateCheckCount)) {
        return;
    }

    mapping[remapIndex] = button;
    if (threeButtonMode) {
        mirrorThreeButtonMapping(button, remapIndex);
    }

    ++remapIndex;
    const uint8_t requiredButtons = threeButtonMode ? kThreeButtonMappingSize : kMappingSize;
    if (remapIndex >= requiredButtons) {
        finishRemap();
        return;
    }

    remapBlockedUntil = millis() + kRemapDebounceMs;
}

bool isRemapping() {
    return remapIndex >= 0;
}

void blinkBuiltInLed(uint8_t blinkCount) {
    for (uint8_t i = 0; i < blinkCount * 2U; ++i) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
        delay(kBlinkDurationMs);
    }

    ledState = HIGH;
    digitalWrite(LED_BUILTIN, HIGH);
}

void updateBlinkIndicator() {
    if (remainingToggles == 0 || millis() - lastBlinkAt < kBlinkDurationMs) {
        return;
    }

    lastBlinkAt = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

    if (remainingToggles > 0 && --remainingToggles == 0) {
        ledState = HIGH;
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

MdButton getMappedLeftButton()      { return mapping[0]; }
MdButton getMappedRightButton()     { return mapping[1]; }
MdButton getMappedBothButton()      { return mapping[2]; }
MdButton getMappedAutoLeftButton()  { return mapping[3]; }
MdButton getMappedAutoRightButton() { return mapping[4]; }
MdButton getMappedAutoBothButton()  { return mapping[5]; }
