#include <Arduino.h>
#include "Config.h"
#include "Debug.h"
#include "PadHandler.h"
#include "PadProtocol.h"
#include "ConsoleControl.h"
#include "VideoMode.h"
#include "Remapping.h"
#include "AutoFire.h"

void handle_pad() {
	static long last_combo_time = 0;

	switch (getPadType()) {
		case PAD_SMS:
			{
				// Just relay data without much thinking
				byte pad_status = read_sms_pad();
				handle_pause_button(false);
				write_sms_pad(pad_status);
				break;
			}

		case PAD_MD:
		case PAD_MD_6BTN:
			{
				word pad_status = read_md_pad();

				// Handle pause
				handle_pause_button((pad_status & MD_BTN_START) != 0);

#ifdef PAD_LED_PIN
				digitalWrite(PAD_LED_PIN, pad_status);
#endif
				//Handle combos
				if (millis() - last_combo_time > IGNORE_COMBO_MS) {
					// Look for special combos
					if ((pad_status & COMBO_TRIGGER) == COMBO_TRIGGER) {
						
#ifdef FMSOUND_OUT_PIN
						if ((pad_status & COMBO_JAP_FM_SOUND) == COMBO_JAP_FM_SOUND) {
							debugln(F("Enable JAP FM Sound"));
							setFmSoundSwitchStateAndReboot(JAP_FM);
						} else if ((pad_status & COMBO_FM_SOUND) == COMBO_FM_SOUND) {
							debugln(F("Enable FM Sound"));
							setFmSoundSwitchStateAndReboot(FM);
						} else if ((pad_status & COMBO_PSG_SOUND) == COMBO_PSG_SOUND) {
							debugln(F("Enable PSG Sound"));
							setFmSoundSwitchStateAndReboot(PSG);
						} else
#endif
						if ((pad_status & COMBO_REMAP_3BTN) == COMBO_REMAP_3BTN) {
							debugln(F("Remap combo detected"));
							startRemapButtons_3btn();
							last_combo_time = millis();
						} else if ((pad_status & COMBO_REMAP) == COMBO_REMAP) {
							debugln(F("Remap combo detected"));
							startRemapButtons();
							last_combo_time = millis();
						} else if ((pad_status & COMBO_RESET) == COMBO_RESET) {
							debugln(F("Reset combo detected"));
							reset_console();
							last_combo_time = millis();
						} else if ((pad_status & COMBO_50HZ) == COMBO_50HZ) {
							debugln(F("50 Hz combo detected"));
							set_mode(VID_50HZ);
							last_combo_time = millis();
						} else if ((pad_status & COMBO_60HZ) == COMBO_60HZ) {
							debugln(F("60 Hz combo detected"));
							set_mode(VID_60HZ);
							last_combo_time = millis();
						} else if ((pad_status & (COMBO_TRIGGER_AUTOFIRE | getMappedAutoLeftButton())) == (COMBO_TRIGGER_AUTOFIRE | getMappedAutoLeftButton())) {
							cycleAutoFireLeft();
							last_combo_time = millis();
						} else if ((pad_status & (COMBO_TRIGGER_AUTOFIRE | getMappedAutoRightButton())) == (COMBO_TRIGGER_AUTOFIRE | getMappedAutoRightButton())) {
							cycleAutoFireRight();
							last_combo_time = millis();
						} else if ((pad_status & (COMBO_TRIGGER_AUTOFIRE | getMappedAutoBothButton())) == (COMBO_TRIGGER_AUTOFIRE | getMappedAutoBothButton())) {
							cycleAutoFireBoth();
							last_combo_time = millis();
						}
					}
				}

				// Send pad status to SMS
				byte smsPad = mdPadToSms(pad_status);
				write_sms_pad(smsPad);

				break;
			}
	}
}
