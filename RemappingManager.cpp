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
#include "PadController.h"
#include "RemappingManager.h"
#include "StatusLed.h"

RemappingManager remappingManager;

BtnNumber RemappingManager::buttonToNumber(MdButton button) {
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

MdButton RemappingManager::numberToButton(BtnNumber number) {
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

void RemappingManager::save() const {
    for (uint8_t i = 0; i < kMappingSize; ++i) {
        EEPROM.update(REMAP_OFFSET + i, static_cast<uint8_t>(buttonToNumber(mapping_[i])));
    }
}

void RemappingManager::restoreDefaults() {
    static const MdButton defaults[kMappingSize] = {
        MD_BTN_B, MD_BTN_C, MD_BTN_A,
        MD_BTN_Y, MD_BTN_Z, MD_BTN_X,
    };

    for (uint8_t i = 0; i < kMappingSize; ++i) {
        mapping_[i] = defaults[i];
    }
    save();
}

void RemappingManager::begin() {
    for (uint8_t i = 0; i < kMappingSize; ++i) {
        const BtnNumber stored = static_cast<BtnNumber>(EEPROM.read(REMAP_OFFSET + i));
        const MdButton button = numberToButton(stored);

        if (button == static_cast<MdButton>(0)) {
            restoreDefaults();
            return;
        }

        mapping_[i] = button;
    }
}

void RemappingManager::startFullRemap() {
    for (MdButton& button : mapping_) {
        button = static_cast<MdButton>(0);
    }

    remapIndex_ = 0;
    threeButtonMode_ = false;
    blockedUntil_ = 0;
    statusLed.startContinuousBlink();
}

void RemappingManager::startThreeButtonRemap() {
    startFullRemap();
    threeButtonMode_ = true;
}

MdButton RemappingManager::compatibleButton(word padStatus) const {
    switch (padStatus) {
        case MD_BTN_A: return MD_BTN_A;
        case MD_BTN_B: return MD_BTN_B;
        case MD_BTN_C: return MD_BTN_C;
        case MD_BTN_X: return threeButtonMode_ ? static_cast<MdButton>(0) : MD_BTN_X;
        case MD_BTN_Y: return threeButtonMode_ ? static_cast<MdButton>(0) : MD_BTN_Y;
        case MD_BTN_Z: return threeButtonMode_ ? static_cast<MdButton>(0) : MD_BTN_Z;
        default:       return static_cast<MdButton>(0);
    }
}

bool RemappingManager::alreadyContains(MdButton button, uint8_t count) const {
    for (uint8_t i = 0; i < count; ++i) {
        if (mapping_[i] == button) {
            return true;
        }
    }
    return false;
}

void RemappingManager::mirrorThreeButtonMapping(MdButton button, uint8_t index) {
    switch (button) {
        case MD_BTN_A: mapping_[index + 3] = MD_BTN_X; break;
        case MD_BTN_B: mapping_[index + 3] = MD_BTN_Y; break;
        case MD_BTN_C: mapping_[index + 3] = MD_BTN_Z; break;
        default: break;
    }
}

void RemappingManager::finish() {
    save();
    statusLed.stopBlink();
    remapIndex_ = -1;
    threeButtonMode_ = false;
}

void RemappingManager::update() {
    if (!isActive() || millis() < blockedUntil_) {
        return;
    }

    const MdButton button = compatibleButton(padController.readMegaDrivePad());
    if (button == static_cast<MdButton>(0)) {
        return;
    }

    const uint8_t duplicateCheckCount = threeButtonMode_
        ? kThreeButtonMappingSize
        : static_cast<uint8_t>(remapIndex_);
    if (alreadyContains(button, duplicateCheckCount)) {
        return;
    }

    mapping_[remapIndex_] = button;
    if (threeButtonMode_) {
        mirrorThreeButtonMapping(button, static_cast<uint8_t>(remapIndex_));
    }

    ++remapIndex_;
    const uint8_t requiredButtons = threeButtonMode_ ? kThreeButtonMappingSize : kMappingSize;
    if (remapIndex_ >= requiredButtons) {
        finish();
        return;
    }

    blockedUntil_ = millis() + kDebounceMs;
}

bool RemappingManager::setButton(uint8_t index, MdButton button) {
    if (index >= kMappingSize) {
        return false;
    }
    mapping_[index] = button;
    return true;
}
