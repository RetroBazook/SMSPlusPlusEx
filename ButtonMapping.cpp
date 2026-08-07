/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "ButtonMapping.h"

namespace {
constexpr MdButton kDefaultMapping[ButtonMapping::Size] = {
    MD_BTN_B, MD_BTN_C, MD_BTN_A,
    MD_BTN_Y, MD_BTN_Z, MD_BTN_X,
};
}

ButtonMapping::ButtonMapping() { restoreDefaults(); }

MdButton ButtonMapping::get(Slot slot) const {
    return buttons_[static_cast<uint8_t>(slot)];
}

bool ButtonMapping::set(uint8_t index, MdButton button) {
    if (index >= Size) return false;
    buttons_[index] = button;
    return true;
}

void ButtonMapping::clear() {
    for (uint8_t i = 0; i < Size; ++i) buttons_[i] = static_cast<MdButton>(0);
}

void ButtonMapping::restoreDefaults() {
    for (uint8_t i = 0; i < Size; ++i) buttons_[i] = kDefaultMapping[i];
}

bool ButtonMapping::contains(MdButton button, uint8_t count) const {
    if (count > Size) count = Size;
    for (uint8_t i = 0; i < count; ++i) if (buttons_[i] == button) return true;
    return false;
}
