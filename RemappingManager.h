/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#pragma once

#include <Arduino.h>
#include "Types.h"

class RemappingManager {
public:
    void begin();
    void startFullRemap();
    void startThreeButtonRemap();
    void update();

    bool isActive() const { return remapIndex_ >= 0; }

    MdButton leftButton() const      { return mapping_[0]; }
    MdButton rightButton() const     { return mapping_[1]; }
    MdButton bothButton() const      { return mapping_[2]; }
    MdButton autoLeftButton() const  { return mapping_[3]; }
    MdButton autoRightButton() const { return mapping_[4]; }
    MdButton autoBothButton() const  { return mapping_[5]; }

    // Kept as a public capability because the original firmware exposed
    // remapButton(index, button), even though it was not used internally.
    bool setButton(uint8_t index, MdButton button);

private:
    static constexpr uint8_t kMappingSize = 6;
    static constexpr uint8_t kThreeButtonMappingSize = 3;
    static constexpr unsigned long kDebounceMs = 100UL;

    MdButton mapping_[kMappingSize] = {
        MD_BTN_B, MD_BTN_C, MD_BTN_A,
        MD_BTN_Y, MD_BTN_Z, MD_BTN_X,
    };

    int8_t remapIndex_ = -1;
    bool threeButtonMode_ = false;
    unsigned long blockedUntil_ = 0;

    static BtnNumber buttonToNumber(MdButton button);
    static MdButton numberToButton(BtnNumber number);

    void save() const;
    void restoreDefaults();
    MdButton compatibleButton(word padStatus) const;
    bool alreadyContains(MdButton button, uint8_t count) const;
    void finish();
    void mirrorThreeButtonMapping(MdButton button, uint8_t index);
};

extern RemappingManager remappingManager;
