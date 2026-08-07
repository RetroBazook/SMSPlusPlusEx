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
#include "ButtonMapping.h"
#include "Types.h"

class PadController;
class StatusLed;

class RemappingManager {
public:
    RemappingManager(PadController& pad, StatusLed& led) : pad_(pad), led_(led) {}
    void begin();
    void startFullRemap();
    void startThreeButtonRemap();
    void update();
    bool isActive() const { return remapIndex_ >= 0; }
    const ButtonMapping& mapping() const { return mapping_; }
    bool setButton(uint8_t index, MdButton button) { return mapping_.set(index, button); }

private:
    static constexpr uint8_t kThreeButtonMappingSize = 3;
    PadController& pad_;
    StatusLed& led_;
    ButtonMapping mapping_;
    int8_t remapIndex_ = -1;
    bool threeButtonMode_ = false;
    unsigned long blockedUntil_ = 0;
    static BtnNumber buttonToNumber(MdButton button);
    static MdButton numberToButton(BtnNumber number);
    void save() const;
    void restoreDefaults();
    MdButton compatibleButton(uint16_t padStatus) const;
    void finish();
    void mirrorThreeButtonMapping(MdButton button, uint8_t index);
};
