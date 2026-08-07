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

class PadPort {
public:
    void begin();
    void setSelect(byte level) const;
    void setSelectAsOutput() const;
    void setSelectAsInput() const;
    byte read() const;
    void writeMasterSystem(byte padStatus) const;
    bool readSelectPin() const;
    bool isThActive() const;
};
