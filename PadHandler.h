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
class AutoFireManager; class ComboHandler; class ConsoleController; class PadController; class RemappingManager;
class PadHandler {
public:
    PadHandler(PadController& pad, ConsoleController& console, RemappingManager& remap, AutoFireManager& autoFire, ComboHandler& combos)
      : pad_(pad),console_(console),remap_(remap),autoFire_(autoFire),combos_(combos){}
    void update();
private:
    PadController& pad_; ConsoleController& console_; RemappingManager& remap_; AutoFireManager& autoFire_; ComboHandler& combos_;
    void updateMasterSystemPad(); void updateMegaDrivePad();
};
