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

// Kept as unscoped enums because button values are intentionally combined with
// bitwise operators throughout the timing-sensitive controller code.
enum SwitchMode : uint8_t {
    PSG = 0,
    FM,
    JAP_FM,
};

enum MdButton : uint16_t {
    MD_BTN_UP    = 1U << 0,
    MD_BTN_DOWN  = 1U << 1,
    MD_BTN_LEFT  = 1U << 2,
    MD_BTN_RIGHT = 1U << 3,
    MD_BTN_B     = 1U << 4,
    MD_BTN_C     = 1U << 5,
    MD_BTN_A     = 1U << 6,
    MD_BTN_START = 1U << 7,
    MD_BTN_Z     = 1U << 8,
    MD_BTN_Y     = 1U << 9,
    MD_BTN_X     = 1U << 10,
    MD_BTN_MODE  = 1U << 11,
};

enum SmsButton : uint8_t {
    SMS_BTN_UP    = 1U << 0,
    SMS_BTN_DOWN  = 1U << 1,
    SMS_BTN_LEFT  = 1U << 2,
    SMS_BTN_RIGHT = 1U << 3,
    SMS_BTN_TL    = 1U << 4,
    SMS_BTN_TR    = 1U << 5,
    SMS_BTN_TH    = 1U << 6,

    SMS_BTN_B1 = SMS_BTN_TL,
    SMS_BTN_B2 = SMS_BTN_TR,
};

enum VideoMode : uint8_t {
    VID_50HZ = 0,
    VID_60HZ,
    VID_MODES_NO,
};

enum PadType : uint8_t {
    PAD_NONE = 0,
    PAD_SMS,
    PAD_MD,
    PAD_MD_6BTN,
};

enum AutoFireRate : uint8_t {
    AF_VERY_SLOW = 0,
    AF_SLOW,
    AF_MEDIUM,
    AF_QUICK,
    AF_MODES_NO,
};

enum BtnNumber : uint8_t {
    BTN_NB_A = 0,
    BTN_NB_B,
    BTN_NB_C,
    BTN_NB_X,
    BTN_NB_Y,
    BTN_NB_Z,
};
