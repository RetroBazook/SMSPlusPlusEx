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
#include "Types.h"

class PadController {
public:
    void begin();
    void detect();

    PadType type() const { return detectedType_; }
    bool isDetected() const { return detectedType_ != PAD_NONE; }

    word readMegaDrivePad();
    byte readMasterSystemPad() const;
    void writeMasterSystemPad(byte padStatus) const;

    // Retained low-level capability from the original readPadPin7() helper.
    bool readSelectPin() const;

private:
    PadType detectedType_ = PAD_NONE;

    static void setSelect(byte level);
    static byte readPort();
    static void setSelectLineOutput();
    static void setSelectLineInput();
    static bool anyButtonPressed(byte port);
    static bool leftAndRightPressed(byte port);

    void selectMegaDrivePad();
    void selectMasterSystemPad();

#ifdef DEBUG_PAD
    static void debugMegaDriveButtons(word status);
    static void debugMasterSystemButtons(byte status);
#endif
};

extern PadController padController;
