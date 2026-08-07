#pragma once

#include <Arduino.h>
#include "Types.h"

/*******************************************************************************
 * PLATFORM SELECTION
 ******************************************************************************/

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__)

/*
 * Arduino Uno/Nano, but these have different configurations, so the
 * board type must be defined MANUALLY
 */
//~ #define ARDUINO_UNO
#define ARDUINO_NANO

#if defined(ARDUINO_UNO)

#warning "Compiling for Arduino Uno"

/*
 *                    +----[PWR]-------------------| USB |--+
 *                    |                            +-----+  |
 *                    |         GND/RST2  [ ][ ]            |
 *                    |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |
 *                    |          5V/MISO2 [ ][ ]  A4/SDA[ ] |
 *                    |                             AREF[ ] |
 *                    |                              GND[ ] |
 *                    | [ ]N/C                    SCK/13[X] | Pad Port Trace 9
 *                    | [ ]IOREF                 MISO/12[X] | Pad Port Trace 6
 *                    | [ ]RST                   MOSI/11[X]~| Pad Port Trace 4
 *                    | [ ]3V3    +---+               10[X]~| Pad Port Trace 3
 *                +5V | [X]5v    -| A |-               9[X]~| Pad Port Trace 2
 *                GND | [X]GND   -| R |-               8[X] | Pad Port Trace 1
 *                    | [ ]GND   -| D |-                    |
 *                    | [ ]Vin   -| U |-               7[X] | Pad Port Trace 7 => replaced by the TH line Control (going to 74HC4066 pin 13)
 *                    |          -| I |-               6[X]~| Pad Port Pin 7
 * Pad Port Pin 1 PC0 | [X]A0    -| N |-               5[X]~| Pause/Reset In
 * Pad Port Pin 2 PC1 | [X]A1    -| O |-               4[X] | Pause Out
 * Pad Port Pin 3 PC2 | [X]A2     +---+           INT1/3[X]~| Reset Out
 * Pad Port Pin 4 PC3 | [X]A3                     INT0/2[X] | Video Mode
 * Pad Port Pin 6 PC4 | [X]A4/SDA  RST SCK MISO     TX>1[ ] | (Led Green) => replaced by Japanese FM Sound Control (going to 74HC4066 pin 14)
 * Pad Port Pin 9 PC5 | [X]A5/SCL  [ ] [ ] [ ]      RX<0[ ] | (Led Red) => replaced by FM Sound Control (going to 74HC4066 pin 6)
 *                    |            [ ] [ ] [ ]              |
 *                    |  UNO_R3    GND MOSI 5V  ____________/
 *                    \_______________________/
 */

/* We don't have enough pins to connect both the Reset and Pause buttons. Anyway
 * we don't really need them both. Actually we only need one of them to switch
 * between 50/60 Hz modes from the console itself. If switching from the
 * controller is enough, don't enable/connect any of them.
 *
 * By default we expect Pause to be connected, since it is the only physical
 * button available on the SMS2. If it is connected, it can be turned into a
 * Reset button enabling RESET_ON_PAUSE below.
 */
#define PAUSE_IN_PIN 5
//#define RESET_IN_PIN 5

// Other pin definitions
#define PAUSE_OUT_PIN 4
#define RESET_OUT_PIN 3
#define VIDEOMODE_PIN 2
#define SELECT_PAD_PIN 6

/* If leds are enabled, the serial console (useful for debugging) will be
 * disabled
 */
//#define MODE_LED_R_PIN 0
//#define MODE_LED_G_PIN 1  //Should not be enabled with fm sound ctrl pin at the same time

// Controller port
#define PDREG_PAD_PORT DDRC
#define PDREG_PAD_BITS ((1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0))
#define PIREG_PAD PINC
#define POREG_PAD PORTC

// Select signal
#define PDREG_SELECT_PORT DDRD
#define PDREG_SELECT_BIT DDD6
#define POREG_SELECT PORTD

// Select signal is on a different port
#define PIREG_SELECT PIND

// Traces port
#define PDREG_TRACES_PORT DDRB
#define PDREG_TRACES_BITS ((1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0))
#define POREG_TRACES PORTB

// Select trace is on a different port
// #define PDREG_TRACE7_PORT DDRD
// #define PDREG_TRACE7_BIT DDD7
// #define POREG_TRACE7 PORTD

//Set FM Sound output at the place of Pad Type
#define FMSOUND_OUT_PIN 0   // RX0
#define JAP_FMSOUND_OUT_PIN 1   // TX1
#define FM_MOD_OFFSET 45

// #if !defined(MODE_LED_R_PIN) && !defined(MODE_LED_G_PIN)
// #define ENABLE_SERIAL_DEBUG
// #else
// #warning "Serial debugging disabled"
// #endif
//~ #define DEBUG_PAD

#elif defined(ARDUINO_NANO)

#warning "Compiling for Arduino Nano"

/*
 * This configuration is almost identical to that of the Uno, except that we use
 * the Nano extra pin A6 to emulate a digital input to sample the Pause/Reset
 * button. This frees up pin 5, which we can then use as Controller Type Out.
 *
 * Note that you will have to install an external pull-up resistor (1k-10k
 * should be fine) to properly register button presses this way.
 *
 *                               +-----+
 *                  +------------| USB |------------+
 *                  |            +-----+            |
 * Pad Port Trace 9 | [X]D13/SCK        MISO/D12[X] | Pad Port Trace 6
 *                  | [ ]3.3V           MOSI/D11[X]~| Pad Port Trace 4
 *                  | [ ]V.ref     ___    SS/D10[X]~| Pad Port Trace 3
 *   Pad Port Pin 1 | [X]A0       / N \       D9[X]~| Pad Port Trace 2
 *   Pad Port Pin 2 | [X]A1      /  A  \      D8[X] | Pad Port Trace 1
 *   Pad Port Pin 3 | [X]A2      \  N  /      D7[X] | Pad Port Trace 7 => replaced by the TH line Control (going to 74HC4066 pin 13)
 *   Pad Port Pin 4 | [X]A3       \_0_/       D6[X]~| Pad Port Pin 7
 *   Pad Port Pin 6 | [X]A4/SDA               D5[X]~| Controller Type Out => replaced by FM Sound Control (going to 74HC4066 pin 6)
 *   Pad Port Pin 9 | [X]A5/SCL               D4[X] | Pause Out
 *   Pause/Reset In | [X]A6              INT1/D3[X]~| Reset Out
 *                  | [ ]A7              INT0/D2[X] | Video Mode
 *              +5V | [X]5V                  GND[X] | GND
 *                  | [ ]RST                 RST[ ] |
 *                  | [ ]GND   5V MOSI GND   TX1[X] | (Led Green) => replaced by Japanese FM Sound Control (going to 74HC4066 pin 14)
 *                  | [ ]Vin   [ ] [ ] [ ]   RX0[X] | (Led Red) => unused
 *                  |          [ ] [ ] [ ]          |
 *                  |          MISO SCK RST         |
 *                  | NANO-V3                       |
 *                  +-------------------------------+
 */

/* We don't have enough pins to connect both the Reset and Pause buttons. Anyway
 * we don't really need them both. Actually we only need one of them to switch
 * between 50/60 Hz modes from the console itself. If switching from the
 * controller is enough, don't enable/connect any of them.
 *
 * By default we expect Pause to be connected, since it is the only physical
 * button available on the SMS2. If it is connected, it can be turned into a
 * Reset button enabling RESET_ON_PAUSE below.
 *
 * REMEMBER THE EXTERNAL PULL-UP!
 */
#define PAUSE_IN_PIN A6
//~ #define RESET_IN_PIN A6

// Threshold to read analog inputs as HIGH
#define ANALOG_IN_THRESHOLD 950

#define PAUSE_OUT_PIN 4
#define RESET_OUT_PIN 3
#define VIDEOMODE_PIN 2
#define SELECT_PAD_PIN 6
#define TI4066_CONTROL_PIN 7

/* If leds are enabled, the serial console (useful for debugging) will be
 * disabled
 */
//#define MODE_LED_R_PIN 0
//#define MODE_LED_G_PIN 1

// Controller port
#define PDREG_PAD_PORT DDRC
#define PDREG_PAD_BITS ((1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0))
#define PIREG_PAD PINC
#define POREG_PAD PORTC

// Select signal
#define PDREG_SELECT_PORT DDRD
#define PDREG_SELECT_BIT DDD6
#define POREG_SELECT PORTD

// Select signal is on a different por
#define PIREG_SELECT PIND

// Traces port
#define PDREG_TRACES_PORT DDRB
#define PDREG_TRACES_BITS ((1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0))
#define POREG_TRACES PORTB

// Select trace is on a different port (REMOVED FOR LIGHT PHASER COMPATIBILITY)
// #define PDREG_TRACE7_PORT DDRD
// #define PDREG_TRACE7_BIT DDD7
// #define POREG_TRACE7 PORTD

//Set FM Sound output at the place of Pad Type
#define FMSOUND_OUT_PIN 5  //D5
#define JAP_FMSOUND_OUT_PIN 1   // TX1
#define FM_MOD_OFFSET 45


// #if !defined(MODE_LED_R_PIN) && !defined(MODE_LED_G_PIN)
// #define ENABLE_SERIAL_DEBUG
// #else
// #warning "Serial debugging disabled"
// #endif

#else
#error "Unsupported Arduino platform!"
#endif

#else
#error "Unsupported platform!"
#endif


/* Button combo that enables the other combos
 *
 * Note: That vertical bar ("pipe") means that the buttons must be pressed
 *       together.
 */
#define COMBO_TRIGGER MD_BTN_START

#define COMBO_REMAP (MD_BTN_X | MD_BTN_Y | MD_BTN_Z)
#define COMBO_REMAP_3BTN (MD_BTN_UP | MD_BTN_A | MD_BTN_B | MD_BTN_C)

/* Button combos to perform other actions. These are to be considered in
 * addition to TRIGGER_COMBO.
 */
#define COMBO_RESET (MD_BTN_A | MD_BTN_B | MD_BTN_C)

// Combos for video modes
#define COMBO_50HZ (MD_BTN_LEFT | MD_BTN_A)
#define COMBO_60HZ (MD_BTN_RIGHT | MD_BTN_A)

// Combos for fmsound
#define COMBO_PSG_SOUND (MD_BTN_DOWN | MD_BTN_A | MD_BTN_B | MD_BTN_C)
#define COMBO_FM_SOUND (MD_BTN_LEFT | MD_BTN_A | MD_BTN_B | MD_BTN_C)
#define COMBO_JAP_FM_SOUND (MD_BTN_RIGHT | MD_BTN_A | MD_BTN_B | MD_BTN_C)

// Combos for autofire
#define COMBO_TRIGGER_AUTOFIRE (MD_BTN_UP)

// Define this to use A as B+C. When padUseAB is enabled, C = A+B.
#define PAD_USE_THIRD_BTN_AS_2BTNS


/*******************************************************************************
 * ADVANCED SETTINGS
 ******************************************************************************/

/* Offset in the EEPROM at which the current mode should be saved. Undefine to
 * disable mode saving.
 */
#define MODE_ROM_OFFSET 42
#define REMAP_OFFSET 43

// Time to wait after mode change before saving the new mode (milliseconds)
#define MODE_SAVE_DELAY 3000L

/* Colors to use to indicate the video mode, in 8-bit RGB componentes. You can
 * use any value here if your led is connected to PWM-capable pins, otherwise
 * values specified here will be interpreted as either fully off (if 0) or fully
 * on (if anything else).
 *
 * Note that using PWM-values here sometimes causes unpredictable problems. This
 * happened to me on an ATtiny861, and it's probably due to how pins and timers
 * interact. It seems to work fine on a full Arduino, but unless you really want
 * weird colors, use only 0x00 and 0xFF.
 *
 * We only have two LED pins, so let's use a dual-color led.
 */

#define MODE_LED_50HZ_COLOR \
	{ 0xFF, 0x00 }  // Red
#define MODE_LED_60HZ_COLOR \
	{ 0x00, 0xFF }  // Green

// Define this if your led is common-anode, comment out for common-cathode
//#define MODE_LED_COMMON_ANODE

/* Use a single led to indicate the video mode. Since this does NOT disable the
 * dual led, it can be used together with it, provided that you have a free pin.
 *
 * Basically, the single led is blinked 1-2 times according to which mode is set
 * (1 is 50 Hz, see VideoMode below).
 */
//#define MODE_LED_SINGLE_PIN 1

/* Use a led to indicate when a button press is detected. Useful for making sure
 * that all button presses are registered correctly.
 */
//#define PAD_LED_PIN 0

// Print the controller status on serial. Only useful for debugging.
#ifdef ENABLE_SERIAL_DEBUG
//~ #define DEBUG_PAD
#endif

/* Reset the console when the pause button on the console itself is pressed.
 * This might be useful on the SMS2, since it has no RESET button. Now that you
 * can trigger PAUSE from your controller, the PAUSE button on the console is
 * pretty useless, isn't it?
 */
//#define RESET_ON_PAUSE

/* Presses of the reset button longer than this amount of milliseconds will
 * switch to the next mode, shorter presses will reset the console.
 */
#define LONGPRESS_LEN 700

/* Time to ignore combos for after one has been detected. Soft of acts as a
 * debouncing mechanism for combos.
 */
#define IGNORE_COMBO_MS LONGPRESS_LEN

// Debounce duration for the reset/pause button
#define DEBOUNCE_MS 20

// Duration of the reset/pause pulse (milliseconds)
#define PULSE_LEN 250

// Interval between pulses for reading the 6-button pad (microseconds)
#define SIXMD_BTN_PULSE_INTERVAL 30

/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/
