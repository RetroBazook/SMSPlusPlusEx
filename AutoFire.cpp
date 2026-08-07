#include <Arduino.h>
#include "Types.h"
#include "Remapping.h"
#include "AutoFire.h"

namespace {
const byte autofireHitsPerSec[AF_MODES_NO] = {5, 10, 15, 20};
AutoFireButton afStatusL = {AF_MEDIUM, 0};
AutoFireButton afStatusR = {AF_MEDIUM, 0};
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

byte mdPadToSms(word mdPad) {
	byte smsPad = 0x00;

	smsPad |= (mdPad & MD_BTN_UP) ? SMS_BTN_UP : 0x00;
	smsPad |= (mdPad & MD_BTN_DOWN) ? SMS_BTN_DOWN : 0x00;
	smsPad |= (mdPad & MD_BTN_LEFT) ? SMS_BTN_LEFT : 0x00;
	smsPad |= (mdPad & MD_BTN_RIGHT) ? SMS_BTN_RIGHT : 0x00;
	
	bool autoFireL_pressed = mdPad & (getMappedAutoBothButton() | getMappedAutoLeftButton());
	bool autoFireR_pressed = mdPad & (getMappedAutoBothButton() | getMappedAutoRightButton());

	if(!autoFireL_pressed)
	{
		smsPad |= (mdPad & (getMappedLeftButton() | getMappedBothButton())) ? SMS_BTN_B1 : 0x00;
		afStatusL.pressStart = 0;
	} 
	else 
	{
		smsPad |= checkAutoFire(afStatusL) ? SMS_BTN_B1 : 0x00;
	}
	if(!autoFireR_pressed)
	{
		smsPad |= (mdPad & (getMappedRightButton() | getMappedBothButton())) ? SMS_BTN_B2 : 0x00;
		afStatusR.pressStart = 0;
	} 
	else 
	{
		smsPad |= checkAutoFire(afStatusR) ? SMS_BTN_B2 : 0x00;
	}

	return smsPad;
}

void cycleAutoFire(AutoFireButton& btn) {
	btn.rate = static_cast<AutoFireRate>((btn.rate + 1) % AF_MODES_NO);
}

void cycleAutoFireLeft() { cycleAutoFire(afStatusL); }
void cycleAutoFireRight() { cycleAutoFire(afStatusR); }
void cycleAutoFireBoth() { cycleAutoFire(afStatusL); cycleAutoFire(afStatusR); }
