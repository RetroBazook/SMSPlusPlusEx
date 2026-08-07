# Refactor changelog

## Architecture
- Added constructor-based dependency injection.
- Added `PadPort` as the AVR/electrical I/O boundary.
- Added `ButtonMapping` as the mapping value object.
- Added `ComboHandler` for shortcut dispatch.
- Added `DebouncedButton` for shared Reset/Pause input handling.
- Removed cross-module singleton access; objects are composed in `SMSPlusPlusEx.ino`.

## Configuration
- Added `FirmwareConfig.h` with grouped `Combo`, `Timing` and `EepromAddress` constants.
- `BoardConfig.h` now defaults to Nano but permits an Uno build override.
- Added the missing Uno `TI4066_CONTROL_PIN` definition from the documented D7 wiring.
- Removed the inactive `PAD_USE_THIRD_BTN_AS_2BTNS` build macro.

## Compatibility
- EEPROM addresses remain 42 (video), 43 (remap) and 45 (FM).
- Existing combo masks and timing values are unchanged.
- Reset long-press keeps the legacy modulo timing mode intentionally.
- Output remains suppressed immediately when remapping starts.
