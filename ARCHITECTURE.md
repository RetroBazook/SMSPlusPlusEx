# SMSPlusPlusEx architecture

The firmware is split by responsibility. Stateful components are classes with one implementation file each; low-level board constants remain in headers/macros because they map directly to AVR registers.

- `ConsoleController`: Reset/Pause lines, physical buttons, TH state and FM switching.
- `VideoModeManager`: 50/60 Hz state, LEDs and EEPROM persistence.
- `PadController`: controller detection and SMS/Mega Drive wire protocol.
- `RemappingManager`: button mapping, remap workflow and EEPROM persistence.
- `AutoFireManager`: autofire state/rates and Mega Drive to SMS conversion.
- `StatusLed`: built-in LED feedback state.
- `PadHandler`: high-level pad processing and special combos.
- `BoardConfig.h`: board wiring/register aliases only.
- `Config.h`: user-facing behaviour/timing/combo settings.
- `Types.h`: shared enums and button bit masks.
- `SMSPlusPlusEx.ino`: startup order and top-level loop only.

No inheritance or heap allocation is used. Objects are statically allocated, which keeps the design suitable for ATmega328P-class targets.
