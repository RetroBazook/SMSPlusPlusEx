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

class DebouncedButton {
public:
    enum class Event : uint8_t { None, Pressed, Released, LongPress };
    enum class LongPressClock : uint8_t { Elapsed, LegacyModulo };

    DebouncedButton(unsigned long debounceMs, unsigned long longPressMs,
                    LongPressClock clock = LongPressClock::Elapsed);

    Event update(byte level, bool allowLongPress = true);
    bool isPressed() const { return wasPressed_; }
    unsigned int holdCycles() const { return holdCycles_; }

private:
    unsigned long debounceMs_;
    unsigned long longPressMs_;
    LongPressClock clock_;
    byte stableLevel_ = LOW;
    bool wasPressed_ = false;
    unsigned long lastTransitionAt_ = 0;
    unsigned long pressedAt_ = 0;
    unsigned int holdCycles_ = 0;
};
