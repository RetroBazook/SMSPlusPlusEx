/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "BoardConfig.h"
#include "PadPort.h"

void PadPort::begin() {
    setSelectAsInput();

    // Controller data lines are inputs with pull-ups.
    PDREG_PAD_PORT &= ~PDREG_PAD_BITS;
    POREG_PAD |= PDREG_PAD_BITS;

    // Output traces emulate the SMS controller lines.
    PDREG_TRACES_PORT |= PDREG_TRACES_BITS;
#ifdef PDREG_TRACE7_PORT
    PDREG_TRACE7_PORT |= 1U << PDREG_TRACE7_BIT;
#endif
    writeMasterSystem(0x00);

    pinMode(TI4066_CONTROL_PIN, OUTPUT);
    digitalWrite(TI4066_CONTROL_PIN, LOW);
}

void PadPort::setSelect(byte level) const {
    if (level != LOW) {
        POREG_SELECT |= (1U << PDREG_SELECT_BIT);
    } else {
        POREG_SELECT &= ~(1U << PDREG_SELECT_BIT);
    }
}

void PadPort::setSelectAsOutput() const {
    pinMode(SELECT_PAD_PIN, OUTPUT);
}

void PadPort::setSelectAsInput() const {
    pinMode(SELECT_PAD_PIN, INPUT_PULLUP);
}

byte PadPort::read() const {
    return PIREG_PAD & PDREG_PAD_BITS;
}

void PadPort::writeMasterSystem(byte padStatus) const {
    // SMS controller lines are active-low.
    POREG_TRACES = ~padStatus & PDREG_TRACES_BITS;
}

bool PadPort::readSelectPin() const {
#ifdef PIREG_SELECT
    return (PIREG_SELECT & (1U << PDREG_SELECT_BIT)) != 0;
#else
    return false;
#endif
}

bool PadPort::isThActive() const {
    return (PIND & (1U << SELECT_PAD_PIN)) == 0;
}
