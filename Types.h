#pragma once

#include <Arduino.h>

enum SwitchMode : uint8_t {
    PSG = 0,
    FM = 1,
    JAP_FM = 2
};

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

// Master System Buttons - For internal use only
enum SmsButton {
	SMS_BTN_TH = 1 << 6,
	SMS_BTN_TR = 1 << 5,
	SMS_BTN_TL = 1 << 4,
	SMS_BTN_RIGHT = 1 << 3,
	SMS_BTN_LEFT = 1 << 2,
	SMS_BTN_DOWN = 1 << 1,
	SMS_BTN_UP = 1 << 0,

	// Commodity aliases
	SMS_BTN_B1 = SMS_BTN_TL,
	SMS_BTN_B2 = SMS_BTN_TR
};

enum VideoMode {
	VID_50HZ = 0,
	VID_60HZ,
	VID_MODES_NO  // Leave at end
};

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


struct AutoFireButton {
    AutoFireRate rate;
    unsigned long pressStart;
};

enum BtnNumber
{
  BTN_NB_A = 0, BTN_NB_B, BTN_NB_C, BTN_NB_X, BTN_NB_Y, BTN_NB_Z
};

