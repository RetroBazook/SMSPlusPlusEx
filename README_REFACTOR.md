# SMSPlusPlusEx – modernized C++ layout

This version reorganizes the original monolithic Arduino sketch into small C++ modules while preserving the original runtime behaviour and AVR register-level pad timing.

## Modules

- `SMSPlusPlusEx.ino`: Arduino entry points and high-level orchestration only.
- `Config.h`: board pinout, compile-time options and user-tunable settings.
- `Types.h`: shared enums and button bitmasks.
- `ConsoleControl.*`: reset, pause, TH detection and FM switch.
- `VideoMode.*`: 50/60 Hz state, LEDs and EEPROM persistence.
- `PadProtocol.*`: SMS/MD detection, low-level protocol reads and SMS output.
- `Remapping.*`: mapping persistence and remap UI state.
- `AutoFire.*`: MD-to-SMS conversion and autofire state.
- `PadHandler.*`: high-level pad processing and special combos.
- `Debug.h`: debug macros.

## Refactor policy

The hardware protocol, delays, EEPROM offsets and controller combos are intentionally preserved. Internal helpers are file-local (`namespace { ... }`) and public APIs use consistent descriptive names.

One suspicious legacy expression in reset long-press handling (`now % pressedAt`) is intentionally retained to avoid an implicit behaviour change; it is marked in source for a separate bug-fix pass.
