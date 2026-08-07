#include <Arduino.h>
#include <EEPROM.h>
#include "Config.h"
#include "Debug.h"
#include "VideoMode.h"

namespace {
VideoMode current_mode = VID_50HZ;
unsigned long mode_last_changed_time = 0;
#if defined(MODE_LED_R_PIN) || defined(MODE_LED_G_PIN)
#define ENABLE_MODE_LED_DUAL
const byte mode_led_colors[][VID_MODES_NO] = {
    MODE_LED_50HZ_COLOR,
    MODE_LED_60HZ_COLOR
};
#endif
}

void setupVideoMode() {
#ifdef MODE_LED_R_PIN
    pinMode(MODE_LED_R_PIN, OUTPUT);
#endif
#ifdef MODE_LED_G_PIN
    pinMode(MODE_LED_G_PIN, OUTPUT);
#endif
#ifdef MODE_LED_SINGLE_PIN
    pinMode(MODE_LED_SINGLE_PIN, OUTPUT);
#endif

    pinMode(VIDEOMODE_PIN, OUTPUT);
    current_mode = VID_50HZ;
#ifdef MODE_ROM_OFFSET
    byte tmp = EEPROM.read(MODE_ROM_OFFSET);
    debug(F("Loaded video mode from EEPROM: "));
    debugln(tmp);
    if (tmp < VID_MODES_NO) {
        current_mode = static_cast<VideoMode>(tmp);
    }
#endif
    set_mode(current_mode);
    mode_last_changed_time = 0;
}

VideoMode getCurrentVideoMode() { return current_mode; }

void update_mode_leds() {
#ifdef ENABLE_MODE_LED_DUAL
	const byte* colors = mode_led_colors[current_mode];
	byte c;

#ifdef MODE_LED_R_PIN
	c = colors[0];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite(MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
	c = colors[1];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	digitalWrite(MODE_LED_G_PIN, c);
#endif

#endif  // ENABLE_MODE_LED_DUAL

#ifdef MODE_LED_SINGLE_PIN
	// WARNING: This loop must be reasonably shorter than LONGPRESS_LEN in the worst case!
	for (int i = 0; i < current_mode + 1; ++i) {
		digitalWrite(MODE_LED_SINGLE_PIN, LOW);
		delay(40);
		digitalWrite(MODE_LED_SINGLE_PIN, HIGH);
		delay(80);
	}
#endif
}

void save_mode() {
#ifdef MODE_ROM_OFFSET
	if (mode_last_changed_time > 0 && millis() - mode_last_changed_time >= MODE_SAVE_DELAY) {
		debug(F("Saving video mode to EEPROM: "));
		debugln(current_mode);
		byte saved_mode = EEPROM.read(MODE_ROM_OFFSET);
		if (current_mode != saved_mode) {
			EEPROM.write(MODE_ROM_OFFSET, static_cast<byte>(current_mode));
		} else {
			debugln(F("Mode unchanged, not saving"));
		}
		mode_last_changed_time = 0;  // Don't save again

		// Blink led to tell the user that mode was saved
#ifdef ENABLE_MODE_LED_DUAL
		byte c = 0;

#ifdef MODE_LED_COMMON_ANODE
		c = 255 - c;
#endif

#ifdef MODE_LED_R_PIN
		digitalWrite(MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
		digitalWrite(MODE_LED_G_PIN, c);
#endif

		// Keep off for a bit
		delay(200);

		// Turn led back on
		update_mode_leds();
#endif  // ENABLE_MODE_LED_DUAL

#ifdef MODE_LED_SINGLE_PIN
		// Make one long flash
		digitalWrite(MODE_LED_SINGLE_PIN, LOW);
		delay(500);
		digitalWrite(MODE_LED_SINGLE_PIN, HIGH);
#endif
	}
#endif  // MODE_ROM_OFFSET
}

void set_mode(VideoMode m) {
	switch (m) {
		default:
		case VID_50HZ:
			digitalWrite(VIDEOMODE_PIN, HIGH);  // PAL 50Hz
			break;
		case VID_60HZ:
			digitalWrite(VIDEOMODE_PIN, LOW);  // PAL 60Hz
	}

	current_mode = m;
	update_mode_leds();

	mode_last_changed_time = millis();
}

void change_mode(int increment) {
	// This also loops in [0, VID_MODES_NO) backwards
	VideoMode new_mode = static_cast<VideoMode>((current_mode + increment + VID_MODES_NO) % VID_MODES_NO);
	set_mode(new_mode);
}

void next_mode() {
	change_mode(+1);
}

void prev_mode() {
	change_mode(-1);
}

