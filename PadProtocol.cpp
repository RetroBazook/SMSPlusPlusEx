/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include <Arduino.h>

#include "Config.h"
#include "ConsoleControl.h"
#include "Debug.h"
#include "PadProtocol.h"
#include "Remapping.h"

namespace {
PadType detectedPadType = PAD_NONE;

void setSelect(byte level) {
    if (level != LOW) {
        POREG_SELECT |= (1U << PDREG_SELECT_BIT);
    } else {
        POREG_SELECT &= ~(1U << PDREG_SELECT_BIT);
    }
}

byte readPadPort() {
    return PIREG_PAD & PDREG_PAD_BITS;
}

void setSelectLineAsOutput() {
    pinMode(SELECT_PAD_PIN, OUTPUT);
}

void setSelectLineAsInput() {
    pinMode(SELECT_PAD_PIN, INPUT_PULLUP);
}

bool anyButtonPressed(byte port) {
    return port != 0b00111111;
}

bool leftAndRightPressed(byte port) {
    return (port & 0x0C) == 0;
}

void selectMegaDrivePad() {
    detectedPadType = PAD_MD;

    // 6-button detection sequence. Timing is intentionally kept identical to
    // the original implementation.
    setSelect(HIGH);
    delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
    setSelect(LOW);
    delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
    setSelect(HIGH);
    delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
    setSelect(LOW);
    delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

    byte port = readPadPort();
#ifdef DEBUG_PAD
    debug(F("Port Read #2 = "));
    debugln(port, BIN);
#endif

    if ((port & 0x0F) == 0x00) {
        setSelect(HIGH);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
        setSelect(LOW);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

        port = readPadPort();
#ifdef DEBUG_PAD
        debug(F("Port Read #3 = "));
        debugln(port, BIN);
#endif
        if ((port & 0x0F) == 0x0F) {
            detectedPadType = PAD_MD_6BTN;
        }
    }

    setSelect(HIGH);
}

void selectMasterSystemPad() {
    setSelectLineAsInput();
    delay(10);
    digitalWrite(TI4066_CONTROL_PIN, HIGH);
    detectedPadType = PAD_SMS;
}

#ifdef DEBUG_PAD
void debugMegaDriveButtons(word status) {
    debug(F("Pressed: "));
    if (status & MD_BTN_UP) debug(F("Up "));
    if (status & MD_BTN_DOWN) debug(F("Down "));
    if (status & MD_BTN_LEFT) debug(F("Left "));
    if (status & MD_BTN_RIGHT) debug(F("Right "));
    if (status & MD_BTN_A) debug(F("A "));
    if (status & MD_BTN_B) debug(F("B "));
    if (status & MD_BTN_C) debug(F("C "));
    if (status & MD_BTN_X) debug(F("X "));
    if (status & MD_BTN_Y) debug(F("Y "));
    if (status & MD_BTN_Z) debug(F("Z "));
    if (status & MD_BTN_MODE) debug(F("Mode "));
    if (status & MD_BTN_START) debug(F("Start "));
    debugln();
}

void debugMasterSystemButtons(byte status) {
    debug(F("Pressed: "));
    if (status & SMS_BTN_UP) debug(F("Up "));
    if (status & SMS_BTN_DOWN) debug(F("Down "));
    if (status & SMS_BTN_LEFT) debug(F("Left "));
    if (status & SMS_BTN_RIGHT) debug(F("Right "));
    if (status & SMS_BTN_B1) debug(F("B1 "));
    if (status & SMS_BTN_B2) debug(F("B2 "));
    if (status & SMS_BTN_TH) debug(F("TH "));
    debugln();
}
#endif
}  // namespace

void initializePadInput() {
    setSelectLineAsInput();
    PDREG_PAD_PORT &= ~PDREG_PAD_BITS;
    POREG_PAD |= PDREG_PAD_BITS;  // Enable pull-ups.
}

void initializeOutputTraces() {
    PDREG_TRACES_PORT |= PDREG_TRACES_BITS;
#ifdef PDREG_TRACE7_PORT
    PDREG_TRACE7_PORT |= 1U << PDREG_TRACE7_BIT;
#endif
    writeMasterSystemPad(0x00);
}

void initializeElectronicSwitch() {
    pinMode(TI4066_CONTROL_PIN, OUTPUT);
    digitalWrite(TI4066_CONTROL_PIN, LOW);
}

void detectGamepad() {
    if (isThActive()) {
        // TH held low indicates the Light Phaser path, which uses SMS wiring.
        selectMasterSystemPad();
        return;
    }

    setSelectLineAsOutput();
    delay(10);

    setSelect(HIGH);
    delay(10);
    const byte portHigh = readPadPort();
    const bool buttonPressedWithSelectHigh = anyButtonPressed(portHigh);

    setSelect(LOW);
    delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
    const byte portLow = readPadPort();

#ifdef DEBUG_PAD
    debug(F("Port Read #1 = "));
    debugln(portLow, BIN);
#endif

    if (leftAndRightPressed(portLow)) {
        selectMegaDrivePad();
        return;
    }

    if (buttonPressedWithSelectHigh) {
        selectMasterSystemPad();
        return;
    }

    setSelectLineAsInput();
    delay(10);
}

PadType getPadType() {
    return detectedPadType;
}

word readMegaDrivePad() {
    static word status = 0x0000;

    setSelect(HIGH);
    delay(10);

    byte port = readPadPort();
    status = (status & 0xFFC0) | (~port & 0x3F);

    setSelect(LOW);
    delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

    port = readPadPort();
    status = (status & 0xFF3F) | ((~port & 0x30) << 2);

    if (detectedPadType == PAD_MD_6BTN) {
        setSelect(HIGH);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
        setSelect(LOW);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
        setSelect(HIGH);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
        setSelect(LOW);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
        setSelect(HIGH);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

        port = readPadPort();
        status = (status & 0xF0FF) | ((((word)~port) & 0x000F) << 8);

        setSelect(LOW);
        delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
    }

    setSelect(HIGH);
    status &= 0x0FFF;

#ifdef DEBUG_PAD
    debugMegaDriveButtons(status);
#endif
    return status;
}

byte readMasterSystemPad() {
    const byte status = static_cast<byte>(~readPadPort()) & 0x7F;
#ifdef DEBUG_PAD
    debugMasterSystemButtons(status);
#endif
    return status;
}

void writeMasterSystemPad(byte padStatus) {
    if (isRemapping()) {
        return;
    }

#ifdef DEBUG_PAD
    debug(F("Sending SMS pad status: "));
    debugln(padStatus, BIN);
#endif

    // SMS lines are active-low: 0 means pressed on the physical output.
    POREG_TRACES = ~padStatus & PDREG_TRACES_BITS;
}
