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
#include "Config.h"
#include "ConsoleController.h"
#include "Debug.h"
#include "PadController.h"
#include "PadHandler.h"
#include "RemappingManager.h"
#include "VideoModeManager.h"

PadHandler padHandler;

bool PadHandler::comboPressed(word padStatus, word combo) {
    return (padStatus & combo) == combo;
}

void PadHandler::markComboHandled() {
    lastComboAt_ = millis();
}

void PadHandler::handleSpecialCombos(word padStatus) {
    if (millis() - lastComboAt_ <= IGNORE_COMBO_MS || !comboPressed(padStatus, COMBO_TRIGGER)) {
        return;
    }

#ifdef FMSOUND_OUT_PIN
    if (comboPressed(padStatus, COMBO_JAP_FM_SOUND)) {
        debugln(F("Enable JAP FM Sound"));
        consoleController.switchFmSoundAndReset(JAP_FM);
        return;
    }
    if (comboPressed(padStatus, COMBO_FM_SOUND)) {
        debugln(F("Enable FM Sound"));
        consoleController.switchFmSoundAndReset(FM);
        return;
    }
    if (comboPressed(padStatus, COMBO_PSG_SOUND)) {
        debugln(F("Enable PSG Sound"));
        consoleController.switchFmSoundAndReset(PSG);
        return;
    }
#endif

    if (comboPressed(padStatus, COMBO_REMAP_3BTN)) {
        debugln(F("Remap combo detected"));
        remappingManager.startThreeButtonRemap();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_REMAP)) {
        debugln(F("Remap combo detected"));
        remappingManager.startFullRemap();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_RESET)) {
        debugln(F("Reset combo detected"));
        consoleController.pulseReset();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_50HZ)) {
        debugln(F("50 Hz combo detected"));
        videoModeManager.set(VID_50HZ);
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_60HZ)) {
        debugln(F("60 Hz combo detected"));
        videoModeManager.set(VID_60HZ);
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_TRIGGER_AUTOFIRE | remappingManager.autoLeftButton())) {
        autoFireManager.cycleLeft();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_TRIGGER_AUTOFIRE | remappingManager.autoRightButton())) {
        autoFireManager.cycleRight();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_TRIGGER_AUTOFIRE | remappingManager.autoBothButton())) {
        autoFireManager.cycleBoth();
        markComboHandled();
    }
}

void PadHandler::updateMasterSystemPad() {
    const byte padStatus = padController.readMasterSystemPad();
    consoleController.updatePauseButton(false);
    padController.writeMasterSystemPad(padStatus);
}

void PadHandler::updateMegaDrivePad() {
    const word padStatus = padController.readMegaDrivePad();
    consoleController.updatePauseButton((padStatus & MD_BTN_START) != 0);

#ifdef PAD_LED_PIN
    digitalWrite(PAD_LED_PIN, padStatus);
#endif

    handleSpecialCombos(padStatus);
    padController.writeMasterSystemPad(autoFireManager.convertToMasterSystem(padStatus));
}

void PadHandler::update() {
    switch (padController.type()) {
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
