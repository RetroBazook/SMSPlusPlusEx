/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

# Architecture

SMSPlusPlusEx is composed in `SMSPlusPlusEx.ino`. Dependencies are passed explicitly to constructors; feature classes no longer reach into global singleton objects.

- `PadPort`: AVR register and electrical I/O only.
- `PadController`: SMS / Mega Drive controller detection and protocol.
- `ButtonMapping`: typed six-slot mapping value object.
- `RemappingManager`: EEPROM persistence and interactive remapping workflow.
- `AutoFireManager`: autofire state and MD-to-SMS button conversion.
- `DebouncedButton`: reusable active-low debounce / long-press state machine.
- `ConsoleController`: Reset/Pause/FM console I/O; uses `DebouncedButton`.
- `VideoModeManager`: 50/60 Hz state, LEDs and EEPROM persistence.
- `ComboHandler`: interprets controller shortcuts and dispatches feature actions.
- `PadHandler`: high-level per-frame pad routing only.
- `StatusLed`: status/remapping LED animation.

There is no dynamic allocation, inheritance, RTTI or virtual dispatch.
