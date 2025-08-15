This is a fork from https://github.com/SukkoPera/SMSPlusPlus

Link to the installation page : https://github.com/SukkoPera/SMSPlusPlus/wiki/Installation

## Additional Features in the EX Mod Version

```

⚠️ Works with an Arduino Nano. Untested with an Arduino Uno ⚠️


1) Restored Light Phaser compatibility

Light Phaser compatibility has been restored by adding an electronic switch using a 74HC4066 IC (tested and working).

2) FM Sound mode control with etim FMSound installed

Adds sound mode control when an etim FMSound board is installed. 
The 74HC4066 IC is also required to control the FMSound board (tested on a French PAL M4jr model).

                                                        74HC4066
                                                      _____________
                                                     |             |
                     Arduino Nano D6 =>       IN/OUT1| 1         14| VDD       <= 5V
SMS IO Controller (Pad Port Trace 7) =>       OUT/IN1| 2         13| CONT 1    <= Arduino Nano D7
                                             OUT/IN2 | 3         12| CONT 4    <= Arduino Nano TX1
                                             IN/OUT2 | 4         11| IN/OUT4   <= JAP_FM FMSOUND (FMSound board)
                                 GND =>       CONT2  | 5         10| OUT/IN4   <= GND FMSOUND (FMSound board)
                     Arduino Nano D5 =>       CONT3  | 6          9| OUT/IN3   <= GND FMSOUND (FMSound board)
                                 GND =>         VSS  | 7          8| IN/OUT3   <= FM FMSOUND (FMSound board)
                                                     |_____________|

⚠️ Important:

There are installation differences from the original mod:

- Pad Port Trace 7 (to IO Controller IC) should no longer be connected to the Arduino. It must be connected to pin 2 of the 74HC4066.
- Arduino D7 should be connected to 74HC4066 CONT1 (pin 13).
- Pad Port Pin 7 remains connected to Arduino Nano D6, but is also connected to pin 1 of the 74HC4066.
- Arduino D5 (FMSOUND_OUT_PIN) should be connected to 74HC4066 CONT3 (pin 6).
- Arduino TX1 (JAP_FMSOUND_OUT_PIN) should be connected to 74HC4066 CONT4 (pin 11).

For the etim FMSound board, the cables that were originally soldered to the 3-position switch must now be soldered to the 74HC4066 (see diagram above).


2) Improved Bluetooth Receiver Compatibility

Fully compatible with all tested Bluetooth dongles (8BitDo, Retro-Bit, and likely others).
The gamepad type detection is no longer performed during Arduino setup.
Instead, the system waits for a button press to determine whether it's a Master System or a Mega Drive controller.
This gives enough time for even the slowest dongles to sync before the check.

3) New Button Combos

  a) Sound mode
  Enable FM Sound and reset: Start + Left + A + B + C
  Enable Japanese FM Sound and reset: Start + Right + A + B + C
  Enable PSG Sound and reset: Start + Down + A + B + C

  b) Remap MD buttons (6 buttons mode)
  Use BC: Start + X + Y + Z

  ℹ️ After entering remap mode, press the keys you want to map in the following order:
	. Btn 1
	. Btn 2
	. Btn 1 + Btn 2
	. Btn 1 autofire
	. Btn 2 autofire
	. Btn 1 autofire + Btn 2 autofire
  
  c) Remap MD buttons (3 buttons mode)
  Swap ON: Start + Up + A + B + C
  
  ℹ️ After entering remap mode, press the keys you want to map in the following order:
	. Btn 1
	. Btn 2
	. Btn 1 + Btn 2

4) Updated Old Combos

  a) In-Game Reset
  Reset: Start + A + B + C
  
  b) Switch Video Mode
  50Hz: Start + A + Left
  60Hz: Start + A + Right

5) Start Button Cycling Fixed

Previously, holding down the "Start" button for too long would cause a loop of pause/unpause in the game.
This behavior has been corrected — now, pressing "Start" only pauses/resumes once, even if held down.

```

Original message :

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
