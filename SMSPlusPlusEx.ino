/*******************************************************************************
 * SMS++ / SMSPlusPlusEx - refactored C++ layout
 *******************************************************************************/

#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "ConsoleControl.h"
#include "VideoMode.h"
#include "PadProtocol.h"
#include "Remapping.h"
#include "PadHandler.h"

void setup() {
#ifdef ENABLE_SERIAL_DEBUG
    Serial.begin(115200);
#endif
    debugln(F("Starting up..."));

    // Keep the console in reset during initialization, as in the original code.
    enableReset();

#ifdef PAD_LED_PIN
    pinMode(PAD_LED_PIN, OUTPUT);
#endif

    setupVideoMode();
    loadMapping();
    setup_pad();
    setup_traces();
    setup_elec_switch_control();

#if defined(PAUSE_IN_PIN) && !defined(ARDUINO_NANO)
    pinMode(PAUSE_IN_PIN, INPUT_PULLUP);
#endif
    disablePause();

#if defined(RESET_IN_PIN) && !defined(ARDUINO_NANO)
    pinMode(RESET_IN_PIN, INPUT_PULLUP);
#endif

#ifdef FMSOUND_OUT_PIN
    setupFmSoundSwitchState();
#endif

    disableReset();
}

void loop() {
    if (getPadType() == PAD_NONE) {
        check_gamepad();
        if (getPadType() == PAD_NONE) {
            return;
        }

        switch (getPadType()) {
            case PAD_SMS:     blinkBuiltInLed(2); break;
            case PAD_MD:      blinkBuiltInLed(3); break;
            case PAD_MD_6BTN: blinkBuiltInLed(6); break;
            default: break;
        }
    }

    if (!isRemapping()) {
        handle_reset_button();
        handle_pad();
        save_mode();
    } else {
        updateRemapping();
    }

    updateBlinkAsync();
}
