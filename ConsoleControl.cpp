#include <Arduino.h>
#include <EEPROM.h>
#include "Config.h"
#include "Debug.h"
#include "ConsoleControl.h"
#include "VideoMode.h"

/* These functions set the RESET line to the desired state. Note that RESET is
 * an active-low signal.
 *
 * We drive the RESET line emulating an open-collector output.
 */
#ifdef RESET_OUT_PIN
void enableReset() {
	/* No explicit setting to LOW is needed, pins are LOW by default when first
	 * set as OUTPUTs.
	 */
	pinMode(RESET_OUT_PIN, OUTPUT);
}

void disableReset() {
	/* Switch to INPUT, pin will go to HI-Z and the pull-up resistor we're
	 * soldered to will bring the line high
	 */
	pinMode(RESET_OUT_PIN, INPUT);
}
#endif

#ifdef FMSOUND_OUT_PIN

SwitchMode currentSwitchState = PSG;

void setupFmSoundSwitchState() 
{
  pinMode(FMSOUND_OUT_PIN, OUTPUT);
  pinMode(JAP_FMSOUND_OUT_PIN, OUTPUT);

  currentSwitchState = (SwitchMode)EEPROM.read(FM_MOD_OFFSET);
	
  if (currentSwitchState > JAP_FM) {
    currentSwitchState = PSG;
  }

	digitalWrite(FMSOUND_OUT_PIN, LOW);
	digitalWrite(JAP_FMSOUND_OUT_PIN, LOW);
	delayMicroseconds(100);
	switch (currentSwitchState) {
		case FM:
			digitalWrite(FMSOUND_OUT_PIN, HIGH);
			break;
		case JAP_FM:
			digitalWrite(JAP_FMSOUND_OUT_PIN, HIGH);
			break;
		case PSG:
			break;
	}
}

void setFmSoundSwitchStateAndReboot(SwitchMode mode) 
{
	if (currentSwitchState != mode) {
    EEPROM.write(FM_MOD_OFFSET, (uint8_t)mode);
		reset_console();
  }
}

#endif

bool isThActive() {
	return !(PIND & (1 << SELECT_PAD_PIN)); // TH = LOW → active
}

/* Ditto for the PAUSE line
 */
#ifdef PAUSE_OUT_PIN
void enablePause() {
	pinMode(PAUSE_OUT_PIN, OUTPUT);
}

void disablePause() {
	pinMode(PAUSE_OUT_PIN, INPUT);
}
#endif

// Reset is active low on SMS
void handle_reset_button() {
#ifdef RESET_IN_PIN
	static byte debounce_level = LOW;
	static bool pressed_before = false;
	static long last_int = 0, last_pressed = 0;
	static unsigned int hold_cycles = 0;

#ifdef ARDUINO_NANO
	/* We use A6/A7 on this platform, which are only analog inputs, so we must
	 * read them as such
	 */
	byte pressed_now = (analogRead(RESET_IN_PIN) > ANALOG_IN_THRESHOLD) ? HIGH : LOW;
#else
	byte pressed_now = digitalRead(RESET_IN_PIN);
#endif

	if (pressed_now != debounce_level) {
		// Reset debouncing timer
		last_int = millis();
		debounce_level = pressed_now;
	} else if (millis() - last_int > DEBOUNCE_MS) {
		// OK, button is stable, see if it has changed
		if (pressed_now == LOW && !pressed_before) {
			// Button just pressed
			last_pressed = millis();
			hold_cycles = 0;
		} else if (pressed_now == HIGH && pressed_before) {
			// Button released
			if (hold_cycles == 0) {
				debugln(F("Reset button pushed for a short time"));
				reset_console();
			}
		} else {
			// Button has not just been pressed/released
			if (pressed_now == LOW && millis() % last_pressed >= LONGPRESS_LEN * (hold_cycles + 1)) {
				// Reset has been hold for a while
				debugln(F("Reset button held"));
				++hold_cycles;
				next_mode();
			}
		}

		pressed_before = (pressed_now == LOW);
	}
#else
#warning "RESET button handling disabled"
#endif
}

// Pause is active low on SMS
void handle_pause_button(bool gamepad_start_pressed) {
#ifdef PAUSE_IN_PIN
	static byte debounce_level = LOW;
	static bool pressed_before = false;
	static long last_int = 0, last_pressed = 0;
	static unsigned int hold_cycles = 0;

	byte pressed_now;
	if (gamepad_start_pressed) {
		pressed_now = LOW;
	} else {
#ifdef ARDUINO_NANO
		pressed_now = (analogRead(PAUSE_IN_PIN) > ANALOG_IN_THRESHOLD) ? HIGH : LOW;
#else
		pressed_now = digitalRead(PAUSE_IN_PIN);
#endif
	}
	

	if (pressed_now != debounce_level) {
		// Reset debouncing timer
		last_int = millis();
		debounce_level = pressed_now;
	} else if (millis() - last_int > DEBOUNCE_MS) {
		// OK, button is stable, see if it has changed
		if (pressed_now == LOW && !pressed_before) {
			// Button just pressed
			last_pressed = millis();
			hold_cycles = 0;
		} else if (pressed_now == HIGH && pressed_before) {
			// Button released
			if (hold_cycles == 0) {
				debugln(F("Pause button pushed for a short time"));
#ifdef RESET_ON_PAUSE
				reset_console();
#else
				pause_console();
#endif
			}
		} else {
			// Button has not just been pressed/released
			if (!gamepad_start_pressed && pressed_now == LOW && millis() - last_pressed >= LONGPRESS_LEN * (hold_cycles + 1)) {
				// Reset has been hold for a while
				debugln(F("Pause button held"));
				++hold_cycles;
				next_mode();
			}
		}

		pressed_before = (pressed_now == LOW);
	}
#else
#warning "PAUSE button handling disabled"
#endif
}


void reset_console() {
	debugln(F("Resetting console"));

	enableReset();
	delay(PULSE_LEN);
	disableReset();
}

void pause_console() {
	debugln(F("Pausing console"));

	enablePause();
	delay(PULSE_LEN);
	disablePause();
}
