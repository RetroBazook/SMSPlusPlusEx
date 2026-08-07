# Modernization notes

This revision keeps the firmware statically allocated and AVR-friendly while making ownership explicit.

## Design rules

- One stateful component class per `.cpp` file.
- No inheritance, virtual methods, heap allocation, STL containers or exceptions.
- AVR register aliases remain macros in `BoardConfig.h` for clarity and timing-sensitive access.
- Shared bit-mask enums remain unscoped because the firmware intentionally uses bitwise operations on them.
- `SMSPlusPlusEx.ino` only coordinates startup and the main loop.

## Stateful components

- `ConsoleController`: reset, pause, physical buttons, TH and FM switch.
- `VideoModeManager`: video mode state and EEPROM persistence.
- `PadController`: controller detection and wire protocol.
- `RemappingManager`: mapping state and EEPROM persistence.
- `AutoFireManager`: autofire timers/rates and MD-to-SMS conversion.
- `StatusLed`: built-in LED animation state.
- `PadHandler`: high-level pad handling and combo debounce state.

## Compatibility choices

The implementation intentionally keeps the original controller timing sequence, EEPROM offsets, combo masks and the historical Reset long-press expression. The unused original `remapButton()` capability is represented by `RemappingManager::setButton()`, and `readPadPin7()` by `PadController::readSelectPin()`.
