# SMSPlusPlusEx refactored layout

Open `SMSPlusPlusEx.ino` in the Arduino IDE. The sketch remains targeted at the board selected in `BoardConfig.h`.

Configuration is intentionally split in two:

- `BoardConfig.h`: physical pins and AVR register mappings.
- `Config.h`: button combos, EEPROM offsets, timings and optional features.

See `ARCHITECTURE.md` and `MODERNIZATION.md` for the class/module layout and compatibility notes.
