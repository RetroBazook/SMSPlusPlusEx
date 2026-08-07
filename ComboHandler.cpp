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
#include "Debug.h"
#include "RemappingManager.h"
#include "VideoModeManager.h"

using namespace FirmwareConfig;

void ComboHandler::update(uint16_t padStatus) {
    if (millis() - lastHandledAt_ <= Timing::ComboIgnoreMs ||
        !pressed(padStatus, Combo::Trigger)) {
        return;
    }

#if SMSPP_PLAYER == 1
#ifdef FMSOUND_OUT_PIN
    if (pressed(padStatus, Combo::JapaneseFmSound)) {
        debugln(F("Enable JAP FM Sound"));
        console_.switchFmSoundAndReset(JAP_FM);
        return;
    }
    if (pressed(padStatus, Combo::FmSound)) {
        debugln(F("Enable FM Sound"));
        console_.switchFmSoundAndReset(FM);
        return;
    }
    if (pressed(padStatus, Combo::PsgSound)) {
        debugln(F("Enable PSG Sound"));
        console_.switchFmSoundAndReset(PSG);
        return;
    }
#endif
#endif

    const ButtonMapping& mapping = remap_.mapping();

    // Remapping and autofire are local controller features and are available
    // on both P1 and P2. Start is only used here as an internal combo modifier;
    // P2 never forwards it to the console Pause line.
    if (pressed(padStatus, Combo::RemapThreeButton)) {
        debugln(F("Remap combo detected"));
        remap_.startThreeButtonRemap();
        handled();
    } else if (pressed(padStatus, Combo::Remap)) {
        debugln(F("Remap combo detected"));
        remap_.startFullRemap();
        handled();
#if SMSPP_PLAYER == 1
    } else if (pressed(padStatus, Combo::Reset)) {
        debugln(F("Reset combo detected"));
        console_.pulseReset();
        handled();
    } else if (pressed(padStatus, Combo::Video50Hz)) {
        debugln(F("50 Hz combo detected"));
        video_.set(VID_50HZ);
        handled();
    } else if (pressed(padStatus, Combo::Video60Hz)) {
        debugln(F("60 Hz combo detected"));
        video_.set(VID_60HZ);
        handled();
#endif
    } else if (pressed(padStatus, Combo::AutoFireTrigger | mapping.autoLeft())) {
        autoFire_.cycleLeft();
        handled();
    } else if (pressed(padStatus, Combo::AutoFireTrigger | mapping.autoRight())) {
        autoFire_.cycleRight();
        handled();
    } else if (pressed(padStatus, Combo::AutoFireTrigger | mapping.autoBoth())) {
        autoFire_.cycleBoth();
        handled();
    }
}
