#pragma once
#include "Types.h"

void setup_pad();
void setup_traces();
void setup_elec_switch_control();
void check_gamepad();
void setup_sms_pad();
void setup_md_pad();
word read_md_pad();
byte read_sms_pad();
void write_sms_pad(byte pad_status);
PadType getPadType();
