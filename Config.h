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
#include "FirmwareConfig.h"
#include "Types.h"

// Optional dual-color video-mode LED settings.
#define MODE_LED_50HZ_COLOR {0xFF, 0x00}
#define MODE_LED_60HZ_COLOR {0x00, 0xFF}
// #define MODE_LED_COMMON_ANODE
// #define MODE_LED_SINGLE_PIN 1
// #define PAD_LED_PIN 0

#ifdef ENABLE_SERIAL_DEBUG
// #define DEBUG_PAD
#endif

// #define RESET_ON_PAUSE

// Historical SMS++ option. The current SMSPlusPlusEx behaviour already maps the
// third MD button through ButtonMapping, so this legacy macro had no active use
// and is intentionally not exposed as a fake build option anymore.
