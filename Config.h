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

#include "BoardConfig.h"
#include "Types.h"

// -----------------------------------------------------------------------------
// Controller shortcuts
// -----------------------------------------------------------------------------
// COMBO_TRIGGER must be held together with one of the action combinations.
constexpr word COMBO_TRIGGER = MD_BTN_START;

constexpr word COMBO_REMAP = MD_BTN_X | MD_BTN_Y | MD_BTN_Z;
constexpr word COMBO_REMAP_3BTN = MD_BTN_UP | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr word COMBO_RESET = MD_BTN_A | MD_BTN_B | MD_BTN_C;

constexpr word COMBO_50HZ = MD_BTN_LEFT | MD_BTN_A;
constexpr word COMBO_60HZ = MD_BTN_RIGHT | MD_BTN_A;

constexpr word COMBO_PSG_SOUND = MD_BTN_DOWN | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr word COMBO_FM_SOUND = MD_BTN_LEFT | MD_BTN_A | MD_BTN_B | MD_BTN_C;
constexpr word COMBO_JAP_FM_SOUND = MD_BTN_RIGHT | MD_BTN_A | MD_BTN_B | MD_BTN_C;

constexpr word COMBO_TRIGGER_AUTOFIRE = MD_BTN_UP;

// Preserved legacy build option from SMS++.
#define PAD_USE_THIRD_BTN_AS_2BTNS

// -----------------------------------------------------------------------------
// EEPROM layout
// -----------------------------------------------------------------------------
// Keep these as macros because MODE_ROM_OFFSET is also used as a feature flag.
#define MODE_ROM_OFFSET 42
#define REMAP_OFFSET 43

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------
constexpr unsigned long MODE_SAVE_DELAY = 3000UL;
constexpr unsigned long LONGPRESS_LEN = 700UL;
constexpr unsigned long IGNORE_COMBO_MS = LONGPRESS_LEN;
constexpr unsigned long DEBOUNCE_MS = 20UL;
constexpr unsigned long PULSE_LEN = 250UL;
constexpr unsigned int SIXMD_BTN_PULSE_INTERVAL = 30U;

// -----------------------------------------------------------------------------
// Video-mode LEDs
// -----------------------------------------------------------------------------
#define MODE_LED_50HZ_COLOR {0xFF, 0x00}  // Red
#define MODE_LED_60HZ_COLOR {0x00, 0xFF}  // Green

// Uncomment for a common-anode dual LED.
// #define MODE_LED_COMMON_ANODE

// Optional single LED: one blink = 50 Hz, two blinks = 60 Hz.
// #define MODE_LED_SINGLE_PIN 1

// Optional activity LED for pad input.
// #define PAD_LED_PIN 0

// Optional verbose pad logging. Requires ENABLE_SERIAL_DEBUG.
#ifdef ENABLE_SERIAL_DEBUG
// #define DEBUG_PAD
#endif

// On SMS2, optionally treat the physical Pause button as Reset.
// #define RESET_ON_PAUSE
