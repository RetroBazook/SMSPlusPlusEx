#include <Arduino.h>
#include <EEPROM.h>
#include "Config.h"
#include "Debug.h"
#include "Remapping.h"
#include "PadProtocol.h"

namespace {
constexpr int BTN_MAP_SIZE = 6;
MdButton buttonsMap[BTN_MAP_SIZE] = {
    MD_BTN_B, MD_BTN_C, MD_BTN_A, MD_BTN_Y, MD_BTN_Z, MD_BTN_X
};
int remapIndex = -1;
bool remap3btnMod = false;
}

MdButton getMappedLeftButton() { return buttonsMap[0]; }
MdButton getMappedRightButton() { return buttonsMap[1]; }
MdButton getMappedBothButton() { return buttonsMap[2]; }
MdButton getMappedAutoLeftButton() { return buttonsMap[3]; }
MdButton getMappedAutoRightButton() { return buttonsMap[4]; }
MdButton getMappedAutoBothButton() { return buttonsMap[5]; }
bool isRemapping() { return remapIndex != -1; }

BtnNumber getCorrespondingKeyNb(MdButton mdBtn);
MdButton getCorrespondingButton(BtnNumber btnNb);
MdButton getCompatibleKey(word pad, bool threebtnmod = false);
void startBlinkAsync(int nbBlink = 0);
void stopBlinkAsync();
void updateRemapButtons();
void updateRemapButtons_3btn();

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

void loadMapping() {

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

static unsigned long previousMillis = 0;
static bool ledState = HIGH;
static int remainingToggles = 0;  // <0 : infini
const unsigned long BLINK_DURATION = 250; // ms

void blinkBuiltInLed(int nbBlink) {
	for (int i = 0; i < (nbBlink * 2); i++) 
	{
		ledState = !ledState;
		digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
		delay(BLINK_DURATION);
	}
	ledState = HIGH;
	digitalWrite(LED_BUILTIN, ledState);
}

void startBlinkAsync(int nbBlink) {
	if (remainingToggles == 0) {
		ledState = LOW;
		digitalWrite(LED_BUILTIN, ledState);
		remainingToggles = (nbBlink > 0) ? nbBlink * 2 : -1;
		previousMillis = millis();
	}
}

void updateBlinkAsync() {
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

MdButton getCompatibleKey(word pad, bool threebtnmod) {
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

void stopBlinkAsync() {
	remainingToggles = 0;
	ledState = HIGH;
	digitalWrite(LED_BUILTIN, ledState);
}

void updateRemapping() {
    if (remap3btnMod) {
        updateRemapButtons_3btn();
    } else {
        updateRemapButtons();
    }
}
