/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "Config.h"
#include "Debug.h"
#include "PadController.h"

using namespace FirmwareConfig;

bool PadController::anyButtonPressed(uint8_t port) { return port != 0b00111111; }
bool PadController::leftAndRightPressed(uint8_t port) { return (port & 0x0C) == 0; }

void PadController::begin() { port_.begin(); }

void PadController::selectMasterSystemPad() {
    port_.setSelectAsInput();
    delay(10);
    digitalWrite(TI4066_CONTROL_PIN, HIGH);
    detectedType_ = PAD_SMS;
}

void PadController::selectMegaDrivePad() {
    detectedType_ = PAD_MD;
    port_.setSelect(HIGH); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
    port_.setSelect(LOW);  delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
    port_.setSelect(HIGH); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
    port_.setSelect(LOW);  delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);

    uint8_t port = port_.read();
#ifdef DEBUG_PAD
    debug(F("Port Read #2 = ")); debugln(port, BIN);
#endif
    if ((port & 0x0F) == 0x00) {
        port_.setSelect(HIGH); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port_.setSelect(LOW);  delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port = port_.read();
#ifdef DEBUG_PAD
        debug(F("Port Read #3 = ")); debugln(port, BIN);
#endif
        if ((port & 0x0F) == 0x0F) detectedType_ = PAD_MD_6BTN;
    }
    port_.setSelect(HIGH);
}

void PadController::detect() {
    if (port_.isThActive()) { selectMasterSystemPad(); return; }
    port_.setSelectAsOutput(); delay(10);
    port_.setSelect(HIGH); delay(10);
    const bool highPressed = anyButtonPressed(port_.read());
    port_.setSelect(LOW); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
    const uint8_t low = port_.read();
#ifdef DEBUG_PAD
    debug(F("Port Read #1 = ")); debugln(low, BIN);
#endif
    if (leftAndRightPressed(low)) { selectMegaDrivePad(); return; }
    if (highPressed) { selectMasterSystemPad(); return; }
    port_.setSelectAsInput(); delay(10);
}

uint16_t PadController::readMegaDrivePad() {
    static uint16_t status = 0;
    port_.setSelect(HIGH); delay(10);
    uint8_t port = port_.read();
    status = (status & 0xFFC0U) | (~port & 0x3FU);
    port_.setSelect(LOW); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
    port = port_.read();
    status = (status & 0xFF3FU) | ((~port & 0x30U) << 2);
    if (detectedType_ == PAD_MD_6BTN) {
        port_.setSelect(HIGH); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port_.setSelect(LOW);  delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port_.setSelect(HIGH); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port_.setSelect(LOW);  delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port_.setSelect(HIGH); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
        port = port_.read();
        status = (status & 0xF0FFU) | ((static_cast<uint16_t>(~port) & 0x000FU) << 8);
        port_.setSelect(LOW); delayMicroseconds(Timing::MegaDriveSixButtonPulseUs);
    }
    port_.setSelect(HIGH);
    status &= 0x0FFFU;
#ifdef DEBUG_PAD
    debugMegaDriveButtons(status);
#endif
    return status;
}

uint8_t PadController::readMasterSystemPad() const {
    const uint8_t status = static_cast<uint8_t>(~port_.read()) & 0x7FU;
#ifdef DEBUG_PAD
    debugMasterSystemButtons(status);
#endif
    return status;
}

void PadController::writeMasterSystemPad(uint8_t padStatus) const {
#ifdef DEBUG_PAD
    debug(F("Sending SMS pad status: ")); debugln(padStatus, BIN);
#endif
    port_.writeMasterSystem(padStatus);
}

#ifdef DEBUG_PAD
void PadController::debugMegaDriveButtons(uint16_t status) {
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

void PadController::debugMasterSystemButtons(uint8_t status) {
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
