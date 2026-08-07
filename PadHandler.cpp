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

#include "AutoFire.h"
#include "Config.h"
#include "ConsoleControl.h"
#include "Debug.h"
#include "PadHandler.h"
#include "PadProtocol.h"
#include "Remapping.h"
#include "VideoMode.h"

namespace {
unsigned long lastComboAt = 0;

bool comboPressed(word padStatus, word combo) {
    return (padStatus & combo) == combo;
}

void markComboHandled() {
    lastComboAt = millis();
}

void handleSpecialCombos(word padStatus) {
    if (millis() - lastComboAt <= IGNORE_COMBO_MS || !comboPressed(padStatus, COMBO_TRIGGER)) {
        return;
    }

#ifdef FMSOUND_OUT_PIN
    if (comboPressed(padStatus, COMBO_JAP_FM_SOUND)) {
        debugln(F("Enable JAP FM Sound"));
        switchFmSoundAndReset(JAP_FM);
        return;
    }
    if (comboPressed(padStatus, COMBO_FM_SOUND)) {
        debugln(F("Enable FM Sound"));
        switchFmSoundAndReset(FM);
        return;
    }
    if (comboPressed(padStatus, COMBO_PSG_SOUND)) {
        debugln(F("Enable PSG Sound"));
        switchFmSoundAndReset(PSG);
        return;
    }
#endif

    if (comboPressed(padStatus, COMBO_REMAP_3BTN)) {
        debugln(F("Remap combo detected"));
        beginThreeButtonRemap();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_REMAP)) {
        debugln(F("Remap combo detected"));
        beginFullRemap();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_RESET)) {
        debugln(F("Reset combo detected"));
        pulseReset();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_50HZ)) {
        debugln(F("50 Hz combo detected"));
        setVideoMode(VID_50HZ);
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_60HZ)) {
        debugln(F("60 Hz combo detected"));
        setVideoMode(VID_60HZ);
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_TRIGGER_AUTOFIRE | getMappedAutoLeftButton())) {
        cycleAutoFireLeft();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_TRIGGER_AUTOFIRE | getMappedAutoRightButton())) {
        cycleAutoFireRight();
        markComboHandled();
    } else if (comboPressed(padStatus, COMBO_TRIGGER_AUTOFIRE | getMappedAutoBothButton())) {
        cycleAutoFireBoth();
        markComboHandled();
    }
}

void updateMasterSystemPad() {
    const byte padStatus = readMasterSystemPad();
    updatePauseButton(false);
    writeMasterSystemPad(padStatus);
}

void updateMegaDrivePad() {
    const word padStatus = readMegaDrivePad();
    updatePauseButton((padStatus & MD_BTN_START) != 0);

#ifdef PAD_LED_PIN
    digitalWrite(PAD_LED_PIN, padStatus);
#endif

    handleSpecialCombos(padStatus);
    writeMasterSystemPad(convertMegaDriveToMasterSystem(padStatus));
}
}  // namespace

void updatePad() {
    switch (getPadType()) {
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
