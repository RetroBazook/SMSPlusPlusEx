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

namespace FirmwareConfig {
namespace Combo {
constexpr uint16_t Trigger = MD_BTN_START;
constexpr uint16_t Remap = MD_BTN_X | MD_BTN_Y | MD_BTN_Z;
constexpr uint16_t RemapThreeButton = MD_BTN_UP | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr uint16_t Reset = MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr uint16_t Video50Hz = MD_BTN_LEFT | MD_BTN_A;
constexpr uint16_t Video60Hz = MD_BTN_RIGHT | MD_BTN_A;
constexpr uint16_t PsgSound = MD_BTN_DOWN | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr uint16_t FmSound = MD_BTN_LEFT | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr uint16_t JapaneseFmSound = MD_BTN_RIGHT | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr uint16_t AutoFireTrigger = MD_BTN_UP;
}

namespace Timing {
constexpr unsigned long VideoModeSaveDelayMs = 3000UL;
constexpr unsigned long LongPressMs = 700UL;
constexpr unsigned long ComboIgnoreMs = LongPressMs;
constexpr unsigned long DebounceMs = 20UL;
constexpr unsigned long ConsolePulseMs = 250UL;
constexpr unsigned int MegaDriveSixButtonPulseUs = 30U;
constexpr unsigned long RemapDebounceMs = 100UL;
constexpr unsigned long StatusLedBlinkMs = 250UL;
}

namespace EepromAddress {
constexpr int VideoMode = 42;
constexpr int Remapping = 43;
constexpr int FmMode = 45;
}
}
