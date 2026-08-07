# SMSPlusPlusEx – refactor C++

Cette version remplace le découpage multi-`.ino` par de vrais modules `.h/.cpp`.

## Organisation
- `SMSPlusPlusEx.ino` : uniquement `setup()` / `loop()` et orchestration.
- `Config.h` : sélection de plateforme, pins, combos et constantes de configuration.
- `Types.h` : enums/types partagés.
- `ConsoleControl.*` : reset, pause, sélection FM/PSG.
- `VideoMode.*` : 50/60 Hz, LEDs et persistance EEPROM.
- `PadProtocol.*` : détection et protocole SMS / Mega Drive 3/6 boutons.
- `Remapping.*` : mapping EEPROM, mode remap et feedback LED.
- `AutoFire.*` : conversion MD→SMS et autofire.
- `PadHandler.*` : traitement du pad et combos.

## Encapsulation
Les états internes `padType`, `current_mode`, `buttonsMap`, `remapIndex`, `remap3btnMod`, et les états d'autofire sont maintenant privés de leur module et accessibles via une petite API.

L'objectif de cette passe est de conserver le comportement fonctionnel du code fourni tout en supprimant la dépendance à la concaténation automatique de plusieurs fichiers `.ino`.
