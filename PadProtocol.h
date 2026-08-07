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

#include "Types.h"

void initializePadInput();
void initializeOutputTraces();
void initializeElectronicSwitch();

void detectGamepad();
PadType getPadType();

word readMegaDrivePad();
byte readMasterSystemPad();
void writeMasterSystemPad(byte padStatus);
