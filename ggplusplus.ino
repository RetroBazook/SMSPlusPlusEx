/*******************************************************************************
 * This file is part of SMS++Ex.    Update by Retro Bazook :)                  *
 *                                                                             *
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>                    *
 *                                                                             *
 * SMS++ is free software: you can redistribute it and/or modify               *
 * it under the terms of the GNU General Public License as published by        *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * SMS++ is distributed in the hope that it will be useful,                    *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with SMS++. If not, see <http://www.gnu.org/licenses/>.               *
 *******************************************************************************
 *
 * SMS++ - 50/60 Hz switch and In-Game-Reset (IGR) for Sega Master System.
 *
 * Please refer to the GitHub page and wiki for any information:
 * https://github.com/SukkoPera/SMSPlusPlus
 */


// http://www.smspower.org/Development/PeripheralPorts


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

#if defined(ARDUINO_NANO)

#warning "Compiling for Arduino Nano"

/*
 *
 *                               +-----+
 *                  +------------| USB |------------+
 *                  |            +-----+            |
 *                  | [ ]D13/SCK        MISO/D12[ ] | 
 *                  | [ ]3.3V           MOSI/D11[X]~| Pad Port Pin 7 (Pad Out TH)
 *                  | [ ]V.ref     ___    SS/D10[X]~| Out Start
 *   Pad Port Pin 1 | [X]A0       / N \       D9[X]~| Out RBtn
 *   Pad Port Pin 2 | [X]A1      /  A  \      D8[X] | Out LBtn
 *   Pad Port Pin 3 | [X]A2      \  N  /      D7[X] | Out Down
 *   Pad Port Pin 4 | [X]A3       \_0_/       D6[X]~| Out Up
 *   Pad Port Pin 6 | [X]A4/SDA               D5[X]~| Out Right
 *   Pad Port Pin 9 | [X]A5/SCL               D4[X] | Out Left
 *                  | [ ]A6              INT1/D3[ ]~| 
 *                  | [ ]A7              INT0/D2[ ] | 
 *              +5V | [X]5V                  GND[X] | GND
 *                  | [ ]RST                 RST[ ] |
 *                  | [ ]GND   5V MOSI GND   TX1[ ] | 
 *                  | [ ]Vin   [ ] [ ] [ ]   RX0[ ] | 
 *                  |          [ ] [ ] [ ]          |
 *                  |          MISO SCK RST         |
 *                  | NANO-V3                       |
 *                  +-------------------------------+
 */



// Threshold to read analog inputs as HIGH
#define ANALOG_IN_THRESHOLD 950

// Controller port
#define PDREG_PAD_PORT DDRC
#define PDREG_PAD_BITS ((1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0))
#define PIREG_PAD PINC
#define POREG_PAD PORTC

// Select signal
#define PDREG_SELECT_PORT DDRB 
#define PDREG_SELECT_BIT  DDB3
#define POREG_SELECT      PORTB

// Select signal is on a different por
#define PIREG_SELECT PIND

#else
#error "Unsupported Arduino platform!"
#endif

#else
#error "Unsupported platform!"
#endif


/*******************************************************************************
 * BUTTON COMBO SETTINGS
 ******************************************************************************/

/* DON'T TOUCH THIS! Just look at it for the button names you can use below!
 *
 * Technical note: This has been organized (together with the controller port
 * wiring) to minimize bit twiddling in the controller reading function.
 */
enum MdButton {
	MD_BTN_MODE = 1 << 11,
	MD_BTN_X = 1 << 10,
	MD_BTN_Y = 1 << 9,
	MD_BTN_Z = 1 << 8,
	MD_BTN_START = 1 << 7,
	MD_BTN_A = 1 << 6,
	MD_BTN_C = 1 << 5,
	MD_BTN_B = 1 << 4,
	MD_BTN_RIGHT = 1 << 3,
	MD_BTN_LEFT = 1 << 2,
	MD_BTN_DOWN = 1 << 1,
	MD_BTN_UP = 1 << 0
};

enum GgPinOut {
	GG_OUT_LEFT = 4,
	GG_OUT_RIGHT = 5,
	GG_OUT_UP = 6,
	GG_OUT_DOWN = 7,
	GG_OUT_LBTN = 8,
	GG_OUT_RBTN = 9,
	GG_OUT_START = 10
};

static int remapIndex = -1;
static bool remap3btnMod = false;

/* Button combo that enables the other combos
 *
 * Note: That vertical bar ("pipe") means that the buttons must be pressed
 *       together.
 */
#define COMBO_TRIGGER MD_BTN_START

#define COMBO_REMAP (MD_BTN_X | MD_BTN_Y | MD_BTN_Z)
#define COMBO_REMAP_3BTN (MD_BTN_UP | MD_BTN_A | MD_BTN_B | MD_BTN_C)

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


#ifdef MODE_ROM_OFFSET
#include <EEPROM.h>
#endif

enum PadType {
	PAD_NONE,
	PAD_SMS,     // Master System
	PAD_MD,      // Mega Drive/Genesis
	PAD_MD_6BTN  // Mega Drive/Genesis 6-Button
};


enum AutoFireRate {
	AF_VERY_SLOW = 0,
	AF_SLOW,
	AF_MEDIUM,
	AF_QUICK,
	AF_MODES_NO  // Leave at end
};

const byte autofireHitsPerSec[AF_MODES_NO] = {
	5,
	10,
	15,
	20
};

struct AutoFireButton {
	AutoFireRate rate;
	unsigned long pressStart;
};

PadType padType = PAD_NONE;

AutoFireButton afStatusL = { AF_MEDIUM, 0 };
AutoFireButton afStatusR = { AF_MEDIUM, 0 };

enum BtnNumber
{
  BTN_NB_A = 0, BTN_NB_B, BTN_NB_C, BTN_NB_X, BTN_NB_Y, BTN_NB_Z
};

MdButton buttonsMap[] = {
	MD_BTN_B,
	MD_BTN_C,
	MD_BTN_A,
	MD_BTN_Y,
	MD_BTN_Z,
	MD_BTN_X
};

MdButton& btnL = buttonsMap[0];
MdButton& btnR = buttonsMap[1];
MdButton& btnLandR = buttonsMap[2];
MdButton& btnAutoL = buttonsMap[3];
MdButton& btnAutoR = buttonsMap[4];
MdButton& btnAutoLandR = buttonsMap[5];

const int BTN_MAP_SIZE = sizeof(buttonsMap) / sizeof(buttonsMap[0]);


bool startPreviouslyPressed = false;

unsigned long mode_last_changed_time;

void save_remap() {
	for (int i = 0; i < BTN_MAP_SIZE; i++) {
		EEPROM.update(REMAP_OFFSET + i, (uint8_t)getCorrespondingKeyNb(buttonsMap[i]));
	}
}

BtnNumber getCorrespondingKeyNb(MdButton mdBtn) {
    switch (mdBtn) {
        case MD_BTN_A: return BTN_NB_A;
        case MD_BTN_B: return BTN_NB_B;
        case MD_BTN_C: return BTN_NB_C;
        case MD_BTN_X: return BTN_NB_X;
        case MD_BTN_Y: return BTN_NB_Y;
        case MD_BTN_Z: return BTN_NB_Z;
    }
    return (BtnNumber)-1; // Valeur invalide
}

MdButton getCorrespondingButton(BtnNumber btnNb) {
    switch (btnNb) {
        case BTN_NB_A: return MD_BTN_A;
        case BTN_NB_B: return MD_BTN_B;
        case BTN_NB_C: return MD_BTN_C;
        case BTN_NB_X: return MD_BTN_X;
        case BTN_NB_Y: return MD_BTN_Y;
        case BTN_NB_Z: return MD_BTN_Z;
    }
    return (MdButton)0; // Valeur invalide
}

inline void loadMapping() {

	for (int i = 0; i < BTN_MAP_SIZE; i++) {
			BtnNumber btnNb = (BtnNumber)EEPROM.read(REMAP_OFFSET + i);
			MdButton mdBtn = getCorrespondingButton(btnNb);

			if (mdBtn == 0) //If save corruped restore default mapping and save
			{
				MdButton defaultMap[BTN_MAP_SIZE] = {
					MD_BTN_B,
					MD_BTN_C,
					MD_BTN_A,
					MD_BTN_Y,
					MD_BTN_Z,
					MD_BTN_X
				};

				for (int i = 0; i < BTN_MAP_SIZE; i++) {
					buttonsMap[i] = defaultMap[i];
				}
				save_remap();
				return;
			}

			buttonsMap[i] = mdBtn;
	}
}

void remapButton(int index, MdButton mdButton) {
    if (index >= 0 && index < BTN_MAP_SIZE) {
        buttonsMap[index] = mdButton;
    }
}

// void pause_console() {
// 	enablePause();
// 	delay(PULSE_LEN);
// 	disablePause();
// }

// Set the level of the SELECT signal of the first controller port
inline void setSelect(byte level) {
	if (level)
		POREG_SELECT |= (1 << PDREG_SELECT_BIT);
	else
		POREG_SELECT &= ~(1 << PDREG_SELECT_BIT);
}

// Returns the state of the first controller port
inline byte readPadPort() {
	return PIREG_PAD & PDREG_PAD_BITS;
}

#ifdef PIREG_SELECT
inline boolean readPadPin7() {
	//~ debugln (PIREG_SELECT, BIN);
	return PIREG_SELECT & (1 << PDREG_SELECT_BIT);
}
#endif

void setSelectLineAsOutput() {
	PDREG_SELECT_PORT |= (1 << PDREG_SELECT_BIT);
}

static unsigned long previousMillis = 0;
static bool ledState = HIGH;
static int remainingToggles = 0;  // <0 : infini
const unsigned long BLINK_DURATION = 250; // ms

inline void blinkBuiltInLed(int nbBlink) {
	for (int i = 0; i < (nbBlink * 2); i++) 
	{
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
		delay(BLINK_DURATION);
	}
	ledState = HIGH;
	digitalWrite(LED_BUILTIN, ledState);
}

inline void startBlinkAsync(int nbBlink = 0) {
	if (remainingToggles == 0) {
		ledState = LOW;
		digitalWrite(LED_BUILTIN, ledState);
		remainingToggles = (nbBlink > 0) ? nbBlink * 2 : -1;
		previousMillis = millis();
	}
}

inline void updateBlinkAsync() {
	if (remainingToggles != 0 && millis() - previousMillis >= BLINK_DURATION) 
	{
		previousMillis = millis();
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
		if (remainingToggles > 0) 
		{
			remainingToggles--;
			if (remainingToggles == 0) {
					ledState = HIGH;
					digitalWrite(LED_BUILTIN, ledState);
			}
		}
	}
}

void startRemapButtons() {
	// Clear the mapping table
	for (int i = 0; i < BTN_MAP_SIZE; i++) {
			buttonsMap[i] = (MdButton)0; // 0 means "not mapped"
	}

	remapIndex = 0;
	startBlinkAsync();
}

void startRemapButtons_3btn() {
	remap3btnMod = true;
	startRemapButtons();
}

MdButton getCompatibleKey(word pad, bool threebtnmod = false) {
    switch (pad) {
			case MD_BTN_A: return MD_BTN_A;
			case MD_BTN_B: return MD_BTN_B;
			case MD_BTN_C: return MD_BTN_C;
			case MD_BTN_X: return threebtnmod ? (MdButton)0 : MD_BTN_X;
			case MD_BTN_Y: return threebtnmod ? (MdButton)0 : MD_BTN_Y;
			case MD_BTN_Z: return threebtnmod ? (MdButton)0 : MD_BTN_Z;
			default:       return (MdButton)0; // Invalid
    }
}

void updateRemapButtons() 
{
	static uint32_t blockUntil = 0;

	if (millis() < blockUntil) { return; }

	MdButton newKeyPressed = getCompatibleKey(read_md_pad());

	// Only accept if exactly one bit is set
	if(newKeyPressed == (MdButton)0) { return; }

	// Avoid duplicates
	for (int i = 0; i < remapIndex; i++) 
	{
			if (buttonsMap[i] == newKeyPressed) return;
	}

	buttonsMap[remapIndex] = newKeyPressed;
	remapIndex++;

	// If mapping complete
	if (remapIndex >= BTN_MAP_SIZE) {
			save_remap();
			stopBlinkAsync();
			remapIndex = -1;
			//asm volatile ("  jmp 0"); //Reset
	} else {
		// Start debounce block period (100 ms)
		blockUntil = millis() + 100;
	}
}

void updateRemapButtons_3btn() 
{
	static uint32_t blockUntil = 0;

	if (millis() < blockUntil) { return; }

	MdButton newKeyPressed = getCompatibleKey(read_md_pad(), true);

	// Only accept if exactly one bit is set
	if(newKeyPressed == (MdButton)0) { return; }

	// Avoid duplicates
	for (int i = 0; i < 3; i++) 
	{
			if (buttonsMap[i] == newKeyPressed) return;
	}

	buttonsMap[remapIndex] = newKeyPressed;
	if(newKeyPressed == MD_BTN_A) {
		buttonsMap[remapIndex + 3] = MD_BTN_X;
	} else if(newKeyPressed == MD_BTN_B) {
		buttonsMap[remapIndex + 3] = MD_BTN_Y;
	} else if(newKeyPressed == MD_BTN_C) {
		buttonsMap[remapIndex + 3] = MD_BTN_Z;
	}
	remapIndex++;

	// If mapping complete
	if (remapIndex >= 3) {
			save_remap();
			stopBlinkAsync();
			remapIndex = -1;
			remap3btnMod = false;
	} else {
		// Start debounce block period (100 ms)
		blockUntil = millis() + 100;
	}
}

inline void stopBlinkAsync() {
	remainingToggles = 0;
	ledState = HIGH;
	digitalWrite(LED_BUILTIN, ledState);
}
inline bool anyButtonPressed(byte port) {
	return port != 0b00111111;
}

inline bool leftAndRightButtonPressed(byte port) {
	return (port & 0x0C) == 0;
}

void check_gamepad() 
{
	delay(10);

	// Guess pad type - start with select line high for a while
	setSelect(HIGH);
	delay(10);
	byte port_high = readPadPort();
	bool highSelectBtnPressed = anyButtonPressed(port_high);

	// Bring select line low 1st time
	setSelect(LOW);
	delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

	byte port_low = readPadPort();

#ifdef DEBUG_PAD
	debug(F("Port Read #1 = "));
	debugln(port_low, BIN);
#endif

	if (leftAndRightButtonPressed(port_low)) {
		/* Left and right are both pressed (active low).
		* This usually doesn't happen unless the controller is worn out,
		* so assume it's a Mega Drive pad.
		*
		* Now let's check if it has 3 or 6 buttons
		*/
		setup_md_pad();
		return;

	} else if (highSelectBtnPressed) {
		setup_sms_pad();
		return;
	}

	delay(10);
}

void setup_sms_pad() {
	delay(10);

	padType = PAD_SMS;
}

void setup_md_pad() {
	// Assume 3 buttons for a start
	padType = PAD_MD;

	/* Now follow the protocol described at
		* https://applause.elfmimi.jp/md6bpad-e.html
		*/
	setSelect(HIGH);  // High again (1st time)
	delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
	setSelect(LOW);  // Low again (2nd time)
	delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

	setSelect(HIGH);  // High again (2nd time)
	delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
	setSelect(LOW);  // Low (3rd time)
	delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

	// We should have all 0s now for the 6-button pad
	byte port = readPadPort();
#ifdef DEBUG_PAD
	debug(F("Port Read #2 = "));
	debugln(port, BIN);
#endif
	if ((port & 0x0F) == 0x00) {
		setSelect(HIGH);  // High again (3rd time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
		// Have Z Y X MD here
		setSelect(LOW);  // Low (4th time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

		// We should have all 1s now for the 6-button pad
		port = readPadPort();
#ifdef DEBUG_PAD
		debug(F("Port Read #3 = "));
		debugln(port, BIN);
#endif
		if ((port & 0x0F) == 0x0F) {
			// This is a 6-button pad
			padType = PAD_MD_6BTN;
		}
	}

	// Bring select line high again
	setSelect(HIGH);
}

void setup_pin() {
	for (int pin = 4; pin <= 11; pin++) {
		pinMode(pin, OUTPUT);
	}

	for (int pin = A0; pin <= A5; pin++) {
		pinMode(pin, INPUT_PULLUP);
	}

	// Make sure no buttons are pressed at start
	update_gg_outputs(0x00);
}


inline word read_md_pad() {
	static word pad_status = 0x0000;
	byte port;

	// Start with select line high for a while
	setSelect(HIGH);
	delay(10);

	// We can read up, down, left, right, C & B
	port = readPadPort();
	pad_status = (pad_status & 0xFFC0) | (~port & 0x3F);

	// Bring select line low 1st time
	setSelect(LOW);
	delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

	// We can read Start & A
	port = readPadPort();
	pad_status = (pad_status & 0xFF3F) | ((~port & 0x30) << 2);

	if (padType == PAD_MD_6BTN) {
		setSelect(HIGH);  // High again (1st time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
		setSelect(LOW);  // Low again (2nd time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

		setSelect(HIGH);  // High again (2nd time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);
		setSelect(LOW);  // Low (3rd time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

		// All 0s at this point

		setSelect(HIGH);  // High again (3rd time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

		// We can read Z, Y, X & Mode
		port = readPadPort();
		pad_status = (pad_status & 0xF0FF)
		             | ((((word)~port) & 0x000F) << 8);

		setSelect(LOW);  // Low (4th time)
		delayMicroseconds(SIXMD_BTN_PULSE_INTERVAL);

		// All 1s at this point
	}

	// Finally bring select line high again
	setSelect(HIGH);

	// Mask away bits representing non-existing buttons
	pad_status &= 0x0FFF;

#ifdef DEBUG_PAD
	debug(F("Pressed: "));
	if (pad_status & MD_BTN_UP)
		debug(F("Up "));
	if (pad_status & MD_BTN_DOWN)
		debug(F("Down "));
	if (pad_status & MD_BTN_LEFT)
		debug(F("Left "));
	if (pad_status & MD_BTN_RIGHT)
		debug(F("Right "));
	if (pad_status & MD_BTN_A)
		debug(F("A "));
	if (pad_status & MD_BTN_B)
		debug(F("B "));
	if (pad_status & MD_BTN_C)
		debug(F("C "));
	if (pad_status & MD_BTN_X)
		debug(F("X "));
	if (pad_status & MD_BTN_Y)
		debug(F("Y "));
	if (pad_status & MD_BTN_Z)
		debug(F("Z "));
	if (pad_status & MD_BTN_MODE)
		debug(F("Mode "));
	if (pad_status & MD_BTN_START)
		debug(F("Start "));
	debugln();
#endif

	return pad_status;
}

inline byte read_sms_pad() {
	byte pad_status = ~readPadPort();

	// Mask away bits representing non-existing buttons
	pad_status &= 0x7F;

	return pad_status;
}

inline void update_gg_outputs(byte pad_status) 
{
	if (remapIndex != -1) { return; } //Dont write during remapping

	WriteOutCtrl(pad_status, GG_OUT_UP);
	WriteOutCtrl(pad_status, GG_OUT_DOWN);
	WriteOutCtrl(pad_status, GG_OUT_LEFT);
	WriteOutCtrl(pad_status, GG_OUT_RIGHT);
	WriteOutCtrl(pad_status, GG_OUT_LBTN);
	WriteOutCtrl(pad_status, GG_OUT_RBTN);
	WriteOutCtrl(pad_status, GG_OUT_START);
}

inline bool WriteOutCtrl(byte pad_status, int pin_num)
{
	digitalWrite(pin_num, pad_status & pin_num ? HIGH : LOW);
}

bool checkAutoFire(AutoFireButton& btn) 
{
	bool result = false;
	unsigned long intv = 1000 / autofireHitsPerSec[btn.rate];  // ms between presses

	if (btn.pressStart != 0) {
		// Button was pressed before
		result = ((millis() - btn.pressStart) / intv) % 2 == 0;
	} else {
		// Just pressed
		btn.pressStart = millis();
	}

	return result;
}

inline byte mdPadToGg(word mdPad) 
{
	byte ggButtons = 0x00;

	ggButtons |= (mdPad & MD_BTN_UP) ? GG_OUT_UP : 0x00;
	ggButtons |= (mdPad & MD_BTN_DOWN) ? GG_OUT_DOWN : 0x00;
	ggButtons |= (mdPad & MD_BTN_LEFT) ? GG_OUT_LEFT : 0x00;
	ggButtons |= (mdPad & MD_BTN_RIGHT) ? GG_OUT_RIGHT : 0x00;
	ggButtons |= (mdPad & MD_BTN_START) ? GG_OUT_START : 0x00;
	
	bool autoFireL_pressed = mdPad & (btnAutoLandR | btnAutoL);
	bool autoFireR_pressed = mdPad & (btnAutoLandR | btnAutoR);

	if(!autoFireL_pressed)
	{
		ggButtons |= (mdPad & (btnL | btnLandR)) ? GG_OUT_LBTN : 0x00;
		afStatusL.pressStart = 0;
	} 
	else 
	{
		ggButtons |= checkAutoFire(afStatusL) ? GG_OUT_LBTN : 0x00;
	}
	if(!autoFireR_pressed)
	{
		ggButtons |= (mdPad & (btnR | btnLandR)) ? GG_OUT_RBTN : 0x00;
		afStatusR.pressStart = 0;
	} 
	else 
	{
		ggButtons |= checkAutoFire(afStatusR) ? GG_OUT_RBTN : 0x00;
	}

	return ggButtons;
}

void cycleAutoFire(AutoFireButton& btn) {
	btn.rate = static_cast<AutoFireRate>((btn.rate + 1) % AF_MODES_NO);
}

void handle_pad() {
	static long last_combo_time = 0;

	switch (padType) {
		case PAD_SMS:
			{
				byte pad_status = read_sms_pad();
				update_gg_outputs(pad_status);
				break;
			}

		case PAD_MD:
		case PAD_MD_6BTN:
			{
				word pad_status = read_md_pad();

				//Handle combos
				if (millis() - last_combo_time > IGNORE_COMBO_MS) {
					// Look for special combos
					if ((pad_status & COMBO_TRIGGER) == COMBO_TRIGGER) {
						if ((pad_status & COMBO_REMAP_3BTN) == COMBO_REMAP_3BTN) {
							startRemapButtons_3btn();
							last_combo_time = millis();
						} else if ((pad_status & COMBO_REMAP) == COMBO_REMAP) {
							startRemapButtons();
							last_combo_time = millis();
						} else if ((pad_status & (COMBO_TRIGGER_AUTOFIRE | btnAutoL)) == (COMBO_TRIGGER_AUTOFIRE | btnAutoL)) {
							cycleAutoFire(afStatusL);
							last_combo_time = millis();
						} else if ((pad_status & (COMBO_TRIGGER_AUTOFIRE | btnAutoR)) == (COMBO_TRIGGER_AUTOFIRE | btnAutoR)) {
							cycleAutoFire(afStatusR);
							last_combo_time = millis();
						} else if ((pad_status & (COMBO_TRIGGER_AUTOFIRE | btnAutoLandR)) == (COMBO_TRIGGER_AUTOFIRE | btnAutoLandR)) {
							cycleAutoFire(afStatusL);
							cycleAutoFire(afStatusR);
							last_combo_time = millis();
						}
					}
				}

				// Send pad status to SMS
				byte ggButtons = mdPadToGg(pad_status);
				update_gg_outputs(ggButtons);

				break;
			}
	}
}

void setup() 
{
	loadMapping();
	setup_pin();
}

void loop() {
	if (padType == PAD_NONE) {
		check_gamepad();
		if (padType == PAD_NONE) {
			return;
		}
		switch(padType)
		{
			case PAD_SMS:
			blinkBuiltInLed(2);
			break;

			case PAD_MD:
			blinkBuiltInLed(3);
			break;

			case PAD_MD_6BTN:
			blinkBuiltInLed(6);
			break;
		}
	}

	if (remapIndex == -1) //Check no remap is occuring
	{
		handle_pad();
	} else {
		if (remap3btnMod) 
		{
			updateRemapButtons_3btn();
		} else {
			updateRemapButtons();
		}
	}

	updateBlinkAsync();
}