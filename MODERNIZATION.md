/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

# Modernization notes

This pass focuses on readability, testability and flexibility without intentionally changing SMSPlusPlusEx behaviour.

Key changes:
- constructor injection replaces cross-module global singleton access;
- hardware register access moved from `PadController` to `PadPort`;
- mapping represented by `ButtonMapping` instead of six unrelated getters;
- combo dispatch moved out of `PadHandler` into `ComboHandler`;
- Reset/Pause debounce logic shares `DebouncedButton`;
- configuration values grouped under `FirmwareConfig::{Combo,Timing,EepromAddress}`;
- Arduino `word`/`byte` are progressively replaced by fixed-width integer types in feature APIs;
- the unused `PAD_USE_THIRD_BTN_AS_2BTNS` macro is removed instead of pretending to be an active option.

The Reset long-press still uses the original modulo-based timing through `DebouncedButton::LongPressClock::LegacyModulo`; changing it should be a separate behavioural fix.
