This is a fork from https://github.com/SukkoPera/SMSPlusPlus

Link to the installation page: https://github.com/SukkoPera/SMSPlusPlus/wiki/Installation

# SMSPlusPlusEx

## Additional Features in the EX Mod Version

> ⚠️ The EX hardware has been tested with Arduino Nano boards.
> Arduino Uno builds are supported by the firmware, but the current P1/P2 PCB wiring documented below is based on Arduino Nano sockets.

## Player Profiles

SMSPlusPlusEx now supports two compile-time hardware profiles:

```cpp
#define SMSPP_PLAYER 1
```

or:

```cpp
#define SMSPP_PLAYER 2
```

### Player 1

Player 1 has access to all console-control features:

- Start button -> Pause
- In-game Reset
- 50/60 Hz switching
- PSG / FM / Japanese FM switching
- Button remapping
- Autofire

### Player 2

Player 2 only handles controller input features:

- Normal SMS / Mega Drive pad input
- Button remapping
- Autofire

Player 2 does **not** control:

- Pause / Start line
- Console Reset
- 50/60 Hz switching
- PSG / FM / Japanese FM switching

On the Player 2 Nano, pins **D0 to D5 are unused by SMSPlusPlusEx**.

---

## Hardware / Arduino Nano Pinout

### Player 1 Arduino Nano

```text
                               +-----+
                  +------------| USB |------------+
                  |            +-----+            |
    P1_OUT_PIN9   | [X]D13/SCK        MISO/D12[X] | P1_OUT_PIN6
                  | [ ]3.3V           MOSI/D11[X] | P1_OUT_PIN4
                  | [ ]V.ref             SS/D10[X] | P1_OUT_PIN3
     P1_IN_PIN1   | [X]A0                    D9[X] | P1_OUT_PIN2
     P1_IN_PIN2   | [X]A1                    D8[X] | P1_OUT_PIN1
     P1_IN_PIN3   | [X]A2                    D7[X] | P1_TH_4066_IN
     P1_IN_PIN4   | [X]A3                    D6[X] | P1_EXT_TH
     P1_IN_PIN6   | [X]A4/SDA                D5[X] | PAUSE_IN
     P1_IN_PIN9   | [X]A5/SCL                D4[X] | OUT_PAUSE
                  | [ ]A6                    D3[X] | OUT_RESET
                  | [ ]A7                    D2[X] | VIDEO_MODE
              +5V | [X]5V                   GND[X] | GND
                  | [ ]RST                  RST[ ] |
                  | [ ]GND   5V MOSI GND    TX1[X] | JFMS_4066_IN
                  | [ ]Vin   [ ] [ ] [ ]    RX0[X] | FMS_4066_IN
                  |          [ ] [ ] [ ]           |
                  |          MISO SCK RST          |
                  | NANO-V3                        |
                  +--------------------------------+
```

Player 1 pin summary:

| Nano pin | Function |
|---|---|
| D0 / RX | FMS_4066_IN |
| D1 / TX | JFMS_4066_IN |
| D2 | VIDEO_MODE |
| D3 | OUT_RESET |
| D4 | OUT_PAUSE |
| D5 | PAUSE_IN |
| D6 | P1_EXT_TH |
| D7 | P1_TH_4066_IN |
| D8 | P1_OUT_PIN1 |
| D9 | P1_OUT_PIN2 |
| D10 | P1_OUT_PIN3 |
| D11 | P1_OUT_PIN4 |
| D12 | P1_OUT_PIN6 |
| D13 | P1_OUT_PIN9 |
| A0 | P1_IN_PIN1 |
| A1 | P1_IN_PIN2 |
| A2 | P1_IN_PIN3 |
| A3 | P1_IN_PIN4 |
| A4 | P1_IN_PIN6 |
| A5 | P1_IN_PIN9 |

### Player 2 Arduino Nano

```text
                               +-----+
                  +------------| USB |------------+
                  |            +-----+            |
    P2_OUT_PIN9   | [X]D13/SCK        MISO/D12[X] | P2_OUT_PIN6
                  | [ ]3.3V           MOSI/D11[X] | P2_OUT_PIN4
                  | [ ]V.ref             SS/D10[X] | P2_OUT_PIN3
     P2_IN_PIN1   | [X]A0                    D9[X] | P2_OUT_PIN2
     P2_IN_PIN2   | [X]A1                    D8[X] | P2_OUT_PIN1
     P2_IN_PIN3   | [X]A2                    D7[X] | P2_TH_4066_IN
     P2_IN_PIN4   | [X]A3                    D6[X] | P2_EXT_TH
     P2_IN_PIN6   | [X]A4/SDA                D5[ ] | N/C
     P2_IN_PIN9   | [X]A5/SCL                D4[ ] | N/C
                  | [ ]A6                    D3[ ] | N/C
                  | [ ]A7                    D2[ ] | N/C
              +5V | [X]5V                   GND[X] | GND
                  | [ ]RST                  RST[ ] |
                  | [ ]GND   5V MOSI GND    TX1[ ] | N/C
                  | [ ]Vin   [ ] [ ] [ ]    RX0[ ] | N/C
                  |          [ ] [ ] [ ]           |
                  |          MISO SCK RST          |
                  | NANO-V3                        |
                  +--------------------------------+
```

Player 2 pin summary:

| Nano pin | Function |
|---|---|
| D0-D5 | Not used by SMSPlusPlusEx |
| D6 | P2_EXT_TH |
| D7 | P2_TH_4066_IN |
| D8 | P2_OUT_PIN1 |
| D9 | P2_OUT_PIN2 |
| D10 | P2_OUT_PIN3 |
| D11 | P2_OUT_PIN4 |
| D12 | P2_OUT_PIN6 |
| D13 | P2_OUT_PIN9 |
| A0 | P2_IN_PIN1 |
| A1 | P2_IN_PIN2 |
| A2 | P2_IN_PIN3 |
| A3 | P2_IN_PIN4 |
| A4 | P2_IN_PIN6 |
| A5 | P2_IN_PIN9 |

---

## 1) Restored Light Phaser Compatibility

Light Phaser compatibility has been restored by adding an electronic switch using a 74HC4066 IC (tested and working).

The TH-related wiring remains:

- Pad Port Trace 7 (towards the SMS I/O controller) is routed through the 74HC4066.
- Arduino D7 controls the relevant 74HC4066 switch.
- Pad Port Pin 7 remains connected to Arduino D6 and to the 74HC4066 input/output path.

## 2) FM Sound Mode Control with etim FMSound Installed

Adds sound mode control when an etim FMSound board is installed.

The 74HC4066 IC is also required to control the FMSound board (tested on a French PAL M4jr model).

Current Player 1 wiring:

```text
                                                        74HC4066
                                                      _____________
                                                     |             |
                     Arduino Nano D6 =>       IN/OUT1| 1         14| VDD       <= 5V
SMS IO Controller (Pad Port Trace 7) =>        OUT/IN1| 2         13| CONT 1    <= Arduino Nano D7
                                             OUT/IN2 | 3         12| CONT 4
                                             IN/OUT2 | 4         11| IN/OUT4   <= JAP_FM FMSOUND (FMSound board)
                                 GND =>       CONT2   | 5         10| OUT/IN4   <= GND FMSOUND (FMSound board)
                     Arduino Nano D0 =>       CONT3   | 6          9| OUT/IN3   <= GND FMSOUND (FMSound board)
                                 GND =>         VSS   | 7          8| IN/OUT3   <= FM FMSOUND (FMSound board)
                                                     |_____________|

Arduino Nano D1 / TX -> control input used for Japanese FM selection.
```

> ⚠️ Important

There are installation differences from the original SMS++ mod:

- Pad Port Trace 7 (to the SMS I/O Controller IC) should no longer be connected directly to the Arduino. It must be connected through the 74HC4066.
- Arduino D7 is used for the TH / Light Phaser 74HC4066 control.
- Pad Port Pin 7 remains connected to Arduino Nano D6.
- Player 1 Arduino D0 is used for FM control.
- Player 1 Arduino D1 / TX is used for Japanese FM control.
- Player 2 does not use the FM switching circuitry.

For the etim FMSound board, the cables that were originally soldered to the 3-position switch must now be routed through the 74HC4066.

## 3) Improved Bluetooth Receiver Compatibility

Fully compatible with all tested Bluetooth dongles (8BitDo, Retro-Bit, and likely others).

The gamepad type detection is no longer performed during Arduino setup. Instead, the system waits for controller activity to determine whether it is a Master System or a Mega Drive controller.

This gives slower Bluetooth dongles enough time to synchronize before controller detection.

## 4) New Button Combos

### a) Sound mode — Player 1 only

- Enable FM Sound and reset: **Start + Left + A + B + C**
- Enable Japanese FM Sound and reset: **Start + Right + A + B + C**
- Enable PSG Sound and reset: **Start + Down + A + B + C**

### b) Remap MD buttons — 6-button mode

Enter remap mode:

**Start + X + Y + Z**

After entering remap mode, press the keys you want to map in this order:

1. Button 1
2. Button 2
3. Button 1 + Button 2
4. Button 1 autofire
5. Button 2 autofire
6. Button 1 autofire + Button 2 autofire

### c) Remap MD buttons — 3-button mode

Enter 3-button remap mode:

**Start + Up + A + B + C**

After entering remap mode, press the keys you want to map in this order:

1. Button 1
2. Button 2
3. Button 1 + Button 2

Remapping is available on both Player 1 and Player 2 firmware profiles.

## 5) Updated Old Combos

### a) In-Game Reset — Player 1 only

**Start + A + B + C**

### b) Switch Video Mode — Player 1 only

- 50 Hz: **Start + A + Left**
- 60 Hz: **Start + A + Right**

## 6) Start Button Cycling Fixed

Previously, holding down the Start button for too long could cause a loop of pause/unpause in the game.

This behavior has been corrected: pressing Start now pauses/resumes only once, even if the button is held.

On the Player 2 firmware profile, Start is never forwarded to the SMS Pause line. It remains available internally as a modifier for features such as remapping.

---

# Original Message

## SMS++

**SMS++** is a modchip for the **Sega Master System** gaming console. It is basically a port of my [MegaDrive++](https://github.com/SukkoPera/MegaDrivePlusPlus) project.

Since the Master System controller only has a few buttons, it makes it impossible to perform "special combos" to control the modchip features. Thus, SMS++ requires the use of an unmodified **Mega Drive/Genesis Control Pad**, either 3- or 6-button.

SMS++ has the following features:

- **Pause your games from the control pad**: In my opinion, a huge design flaw of the Master System is the lack of a Pause button on the control pad. This forces players to stand up and run to the console to pause the game, which often leads to game playing errors. This has always scared me away from the SMS, so I had to do something to remedy: SMS++ allows you to pause your games just by **pressing the Start button**, as you would do on any other modern console!
- **50/60 Hz mode switching**: If your console is PAL, you will also be able to **run most games at 60 Hz**, which means **full-speed** and **full-screen**! Get rid of those **black bars**!
  - The mod is **switchless**, so you don't need to modify the aesthetics of your console installing ugly switches, but rather you will be able to change the video mode:
    - Through the **Pause/Reset button: Keep pushed** to cycle through modes.
    - From the Player 1 controller pad: Press **Start + B + Left** for 50 Hz or **Start + B + Right** for 60 Hz.
  - The last used mode is saved automatically after 5 seconds and reused at power-up.
- **Reset-From-Pad** (AKA **In-Game-Reset** AKA **IGR**): Press **Start + A + B + C**.
- **Make _Wonder Boy in Monster Land_** and other games that normally don't work with the MegaDrive pad **compatible** with it!
- **Turn the Pause button on your Master System II to a Reset button**: Now that you can pause your games straight from your controller, the Pause button on the console is pretty useless. Turning it into a Reset button (which the SMS2 lacks) might make it useful again!
- **Use A/B instead of B/C**: Normally, when you connect a Mega Drive controller to a Master System console, you have to use B as Button 1 and C as Button 2. SMS++ follows this convention, but as this is something more I have never liked, it also allows you to map B1/B2 to A/B: **Keep A pressed at power-up**.
- **Use A as B1+B2**: This makes *Double Dragon* much more playable, for instance. If using A/B for B1/B2, this maps to C, of course.
- **Controller autodetection** at power-up:
  - When using a Master System pad, SMS++ disables all special functions, as if it just wasn't there. This also means that it should also be compatible with the Light Phaser (Though this is still **untested**!).
  - When using a 6-button pad, **X/Y/Z map to autofire versions of A/B/C**: Autofire rate is configurabile between 3 different settings (+ Off), and **every button can have a different rate**. Switch among them with **Start + X/Y/Z**.
- **Supports a single or dual LED** (either common-anode or common-cathode) to indicate the current mode and other settings (Colors can be set to any value when PWM pins are available).
- Uses **cheap *Atmel AVR* microcontrollers**.
  - Can be **flashed on different chips** (ATtiny's, ATmega's, or **even a full Arduino** board), but please note that **not all features are supported on all chips**, depending on the number of available I/O pins, please read on for details.
- Even though default settings are recommended, **everything can be customized** to taste.
- Uses the popular **Arduino environment**, allowing for easy development, testing and modifications.
- Last but not least, it is **Open Source and Free Software**!

If you are interested in modding your console with SMS++, please head to the [wiki](https://github.com/SukkoPera/SMSPlusPlus/wiki). There you will find full instructions about what chip to buy, how to put SMS++ on it and how to install it, with a full wiring guide for a few different Master System models that were sold.
