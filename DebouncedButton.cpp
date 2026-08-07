/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "DebouncedButton.h"

DebouncedButton::DebouncedButton(unsigned long debounceMs, unsigned long longPressMs,
                                 LongPressClock clock)
    : debounceMs_(debounceMs), longPressMs_(longPressMs), clock_(clock) {}

DebouncedButton::Event DebouncedButton::update(byte level, bool allowLongPress) {
    const unsigned long now = millis();
    if (level != stableLevel_) {
        stableLevel_ = level;
        lastTransitionAt_ = now;
        return Event::None;
    }
    if (now - lastTransitionAt_ <= debounceMs_) return Event::None;

    const bool pressed = level == LOW;
    if (pressed && !wasPressed_) {
        wasPressed_ = true;
        pressedAt_ = now;
        holdCycles_ = 0;
        return Event::Pressed;
    }
    if (!pressed && wasPressed_) {
        wasPressed_ = false;
        return Event::Released;
    }
    if (!pressed || !allowLongPress) return Event::None;

    const unsigned long threshold = longPressMs_ * (holdCycles_ + 1U);
    const unsigned long heldFor = clock_ == LongPressClock::LegacyModulo
        ? (pressedAt_ == 0 ? 0 : now % pressedAt_)
        : now - pressedAt_;
    if (heldFor >= threshold) {
        ++holdCycles_;
        return Event::LongPress;
    }
    return Event::None;
}
