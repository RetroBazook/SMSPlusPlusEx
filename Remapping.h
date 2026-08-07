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

void initializeMapping();
void beginFullRemap();
void beginThreeButtonRemap();
void updateRemapping();
bool isRemapping();

void blinkBuiltInLed(uint8_t blinkCount);
void updateBlinkIndicator();

MdButton getMappedLeftButton();
MdButton getMappedRightButton();
MdButton getMappedBothButton();
MdButton getMappedAutoLeftButton();
MdButton getMappedAutoRightButton();
MdButton getMappedAutoBothButton();
