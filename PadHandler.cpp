/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#include "AutoFireManager.h"
#include "ComboHandler.h"
#include "Config.h"
#include "ConsoleController.h"
#include "PadController.h"
#include "PadHandler.h"
#include "RemappingManager.h"
#include "FirmwareConfig.h"

void PadHandler::updateMasterSystemPad() {
    const uint8_t padStatus = pad_.readMasterSystemPad();

#if SMSPP_PLAYER == 1
    // Also services the physical Pause input on the Player 1 board.
    console_.updatePauseButton(false);
#endif

    pad_.writeMasterSystemPad(padStatus);
}

void PadHandler::updateMegaDrivePad() {
    const uint16_t padStatus = pad_.readMegaDrivePad();

#if SMSPP_PLAYER == 1
    console_.updatePauseButton((padStatus & MD_BTN_START) != 0);
#endif

#ifdef PAD_LED_PIN
    digitalWrite(PAD_LED_PIN, padStatus);
#endif

    combos_.update(padStatus);

    // The original write_sms_pad() stops writing immediately when a remap
    // starts, including the same loop iteration that detected the combo.
    if (remap_.isActive()) {
        return;
    }

    const uint8_t smsPad = autoFire_.convertToMasterSystem(
        padStatus,
        remap_.mapping());
    pad_.writeMasterSystemPad(smsPad);
}

void PadHandler::update() {
    switch (pad_.type()) {
        case PAD_SMS:
            updateMasterSystemPad();
            break;

        case PAD_MD:
        case PAD_MD_6BTN:
            updateMegaDrivePad();
            break;

        case PAD_NONE:
        default:
            break;
    }
}
