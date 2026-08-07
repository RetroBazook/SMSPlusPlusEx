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
#include "Config.h"

#ifdef ENABLE_SERIAL_DEBUG
    #define debug(...)   Serial.print(__VA_ARGS__)
    #define debugln(...) Serial.println(__VA_ARGS__)
#else
    #define debug(...)
    #define debugln(...)
#endif
