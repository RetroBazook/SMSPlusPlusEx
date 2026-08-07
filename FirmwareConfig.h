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

// -----------------------------------------------------------------------------
// Firmware instance / PCB role
// -----------------------------------------------------------------------------
// 1 = Player 1 board: owns console-wide lines (FM, 50/60 Hz, Reset and Pause).
// 2 = Player 2 board: controller conversion only; D0..D5 remain untouched.
//
// This can also be overridden from the compiler with -DSMSPP_PLAYER=2.
#ifndef SMSPP_PLAYER
#define SMSPP_PLAYER 1
#endif

#if SMSPP_PLAYER != 1 && SMSPP_PLAYER != 2
#error "SMSPP_PLAYER must be 1 (Player 1) or 2 (Player 2)"
#endif

namespace FirmwareConfig {

enum class Player : uint8_t {
    One = 1,
    Two = 2
};

#if SMSPP_PLAYER == 1
constexpr Player ActivePlayer = Player::One;
#else
constexpr Player ActivePlayer = Player::Two;
#endif

namespace Feature {
constexpr bool ConsoleControls   = SMSPP_PLAYER == 1;
constexpr bool GamepadStartPause = SMSPP_PLAYER == 1;
constexpr bool ResetControl      = SMSPP_PLAYER == 1;
constexpr bool VideoModeControl  = SMSPP_PLAYER == 1;
constexpr bool FmSoundControl    = SMSPP_PLAYER == 1;
}

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

}  // namespace FirmwareConfig
