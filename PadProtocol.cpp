#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "PadProtocol.h"
#include "ConsoleControl.h"
#include "Remapping.h"

namespace { PadType padType = PAD_NONE; }
PadType getPadType() { return padType; }

// Set the level of the SELECT signal of the first controller port
void setSelect(byte level) {
	if (level)
		POREG_SELECT |= (1 << PDREG_SELECT_BIT);
	else
		POREG_SELECT &= ~(1 << PDREG_SELECT_BIT);
}

// Returns the state of the first controller port
byte readPadPort() {
	return PIREG_PAD & PDREG_PAD_BITS;
}

#ifdef PIREG_SELECT
boolean readPadPin7() {
	//~ debugln (PIREG_SELECT, BIN);
	return PIREG_SELECT & (1 << PDREG_SELECT_BIT);
}
#endif

void setSelectLineAsOutput() {
	pinMode(SELECT_PAD_PIN, OUTPUT);
}

void setSelectLineAsInput() {
	pinMode(SELECT_PAD_PIN, INPUT_PULLUP);
}


void setup_pad() {
	// Set port directions
	setSelectLineAsInput();
	PDREG_PAD_PORT &= ~(PDREG_PAD_BITS);  // Other lines are INPUTs...
	POREG_PAD |= PDREG_PAD_BITS;          // ... with pull-ups
}


bool anyButtonPressed(byte port) {
	return port != 0b00111111;
}

bool leftAndRightButtonPressed(byte port) {
	return (port & 0x0C) == 0;
}

void check_gamepad() 
{
	
	if (isThActive()) {
		//Light phaser detected
		setup_sms_pad();
		return;
	}

	setSelectLineAsOutput();
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

	//Otherwise preapre next loop
	setSelectLineAsInput();
	delay(10);
}

void setup_sms_pad() {
	// This is a SMS pad - Switch SELECT to INPUT with pull-up
	setSelectLineAsInput();
	delay(10);

	digitalWrite(TI4066_CONTROL_PIN, HIGH);

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

void setup_traces() {
	PDREG_TRACES_PORT |= PDREG_TRACES_BITS;  // Trace lines are all OUTPUTs

#ifdef PDREG_TRACE7_PORT
	// Trace 7 is on a different MCU port and has to be an OUTPUT as well
	PDREG_TRACE7_PORT |= 1 << PDREG_TRACE7_BIT;
#endif

	// Make sure no buttons are pressed at start
	write_sms_pad(0x00);
}

void setup_elec_switch_control() {
	pinMode(TI4066_CONTROL_PIN, OUTPUT);
	digitalWrite(TI4066_CONTROL_PIN, LOW);
}

/******************************************************************************/

/*
 * The basic idea here is to make up a word where each bit represents the state
 * of a button, where 1 means pressed, for commodity's sake. The bit-button
 * mapping is defined in the MdButton enum above.
 *
 * To get consistent readings, we should really read all of the pad pins at
 * once, at least with the 6-button pad, since tour source states that only data
 * read in 1.6 milli seconds from the first up-edge of Select is reliable.
 * In order to do this we try to connect all pins to a single port of our MCU.
 */
word read_md_pad() {
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

byte read_sms_pad() {
	byte pad_status = ~readPadPort();

	// Mask away bits representing non-existing buttons
	pad_status &= 0x7F;

#ifdef DEBUG_PAD
	debug(F("Pressed: "));
	if (pad_status & SMS_BTN_UP)
		debug(F("Up "));
	if (pad_status & SMS_BTN_DOWN)
		debug(F("Down "));
	if (pad_status & SMS_BTN_LEFT)
		debug(F("Left "));
	if (pad_status & SMS_BTN_RIGHT)
		debug(F("Right "));
	if (pad_status & SMS_BTN_B1)
		debug(F("B1 "));
	if (pad_status & SMS_BTN_B2)
		debug(F("B2 "));
	if (pad_status & SMS_BTN_TH)
		debug(F("TH "));
	debugln();
#endif

	return pad_status;
}

void write_sms_pad(byte pad_status) 
{
	if(isRemapping()) { return; } //Dont write during remapping
	
#ifdef DEBUG_PAD
	debug(F("Sending SMS pad status: "));
	debugln(pad_status, BIN);
#endif

	// NOTE: 0 means pressed!
	POREG_TRACES = ~pad_status & PDREG_TRACES_BITS;
}

