# Modernization notes

This pass focuses on readability and maintainability without intentionally changing the original SMSPlusPlusEx behaviour.

## Main changes

- Consistent camelCase public API (`detectGamepad`, `readMegaDrivePad`, `setVideoMode`, etc.).
- File-local helpers and state moved into anonymous namespaces.
- Large functions split into intention-revealing helpers.
- Duplicate remapping paths consolidated into one state machine.
- `Config.h` split from hardware-specific `BoardConfig.h`.
- Shared enums use explicit underlying integer widths.
- Internal fixed values use `constexpr` where preprocessor feature flags are not required.
- GPL attribution restored to source/header files touched by the refactor.
- `SMSPlusPlusEx.ino` now contains only orchestration and Arduino `setup()`/`loop()`.

## Behaviour intentionally preserved

- AVR register-level I/O and MD 6-button pulse timings.
- SMS / MD / MD 6-button / Light Phaser detection logic.
- EEPROM offsets 42, 43 and 45.
- Controller combos and autofire rates.
- Reset/Pause pulse timing and video-mode save delay.

## Legacy expression left unchanged

The original Reset long-press code contains `millis() % last_pressed`. This looks suspicious compared with the Pause path (`millis() - last_pressed`), but it is deliberately preserved in this modernization pass to avoid silently changing behaviour. It is marked in `ConsoleControl.cpp` for a dedicated bug-fix pass.
