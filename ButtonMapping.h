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

class ButtonMapping {
public:
    enum class Slot : uint8_t { Left, Right, Both, AutoLeft, AutoRight, AutoBoth, Count };
    static constexpr uint8_t Size = static_cast<uint8_t>(Slot::Count);

    ButtonMapping();

    MdButton get(Slot slot) const;
    bool set(uint8_t index, MdButton button);
    void clear();
    void restoreDefaults();
    bool contains(MdButton button, uint8_t count) const;

    MdButton left() const { return get(Slot::Left); }
    MdButton right() const { return get(Slot::Right); }
    MdButton both() const { return get(Slot::Both); }
    MdButton autoLeft() const { return get(Slot::AutoLeft); }
    MdButton autoRight() const { return get(Slot::AutoRight); }
    MdButton autoBoth() const { return get(Slot::AutoBoth); }

private:
    MdButton buttons_[Size];
};
