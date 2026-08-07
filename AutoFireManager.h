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

class AutoFireManager {
public:
    uint8_t convertToMasterSystem(uint16_t megaDrivePad, const ButtonMapping& mapping);
    void cycleLeft(); void cycleRight(); void cycleBoth();
private:
    struct ButtonState { AutoFireRate rate=AF_MEDIUM; unsigned long pressStartedAt=0; };
    ButtonState left_, right_;
    static bool isOn(ButtonState& state);
    static void cycle(ButtonState& state);
};
