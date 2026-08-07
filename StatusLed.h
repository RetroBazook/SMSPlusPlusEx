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

class StatusLed {
public:
    void blinkBlocking(uint8_t blinkCount);
    void startContinuousBlink();
    void stopBlink();
    void update();

private:
    unsigned long lastBlinkAt_ = 0;
    bool ledState_ = HIGH;
    int remainingToggles_ = 0;  // Negative means blink indefinitely.
};

