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
#include "PadPort.h"
#include "Types.h"

class PadController {
public:
    explicit PadController(PadPort& port) : port_(port) {}
    void begin();
    void detect();
    PadType type() const { return detectedType_; }
    bool isDetected() const { return detectedType_ != PAD_NONE; }
    uint16_t readMegaDrivePad();
    uint8_t readMasterSystemPad() const;
    void writeMasterSystemPad(uint8_t padStatus) const;
    bool readSelectPin() const { return port_.readSelectPin(); }

private:
    PadPort& port_;
    PadType detectedType_ = PAD_NONE;
    static bool anyButtonPressed(uint8_t port);
    static bool leftAndRightPressed(uint8_t port);
    void selectMegaDrivePad();
    void selectMasterSystemPad();
#ifdef DEBUG_PAD
    static void debugMegaDriveButtons(uint16_t status);
    static void debugMasterSystemButtons(uint8_t status);
#endif
};
