#pragma once
#include "Types.h"

void loadMapping();
void save_remap();
void remapButton(int index, MdButton mdButton);
void startRemapButtons();
void startRemapButtons_3btn();
void updateRemapping();
bool isRemapping();
void blinkBuiltInLed(int nbBlink);
void updateBlinkAsync();

MdButton getMappedLeftButton();
MdButton getMappedRightButton();
MdButton getMappedBothButton();
MdButton getMappedAutoLeftButton();
MdButton getMappedAutoRightButton();
MdButton getMappedAutoBothButton();
