/*******************************************************************************
 * This file is part of SMS++.
 * Copyright (C) 2016 by SukkoPera <software@sukkology.net>
 *
 * SMS++ is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *******************************************************************************/

#pragma once

#include <Arduino.h>
#include "FirmwareConfig.h"

// Hardware-only configuration. Register aliases remain macros because the pad
// protocol accesses AVR registers directly in timing-sensitive code.

#if !defined(__AVR_ATmega328__) && !defined(__AVR_ATmega328P__) && \
    !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega8__)
#error "Unsupported platform!"
#endif

// Nano is the project default. The build system can override this with
// -DARDUINO_UNO.
#if !defined(ARDUINO_UNO) && !defined(ARDUINO_NANO)
#define ARDUINO_NANO
#endif

#if defined(ARDUINO_UNO)
#warning "Compiling for Arduino Uno"
#elif defined(ARDUINO_NANO)
#warning "Compiling for Arduino Nano"
#else
#error "Unsupported Arduino board!"
#endif

#if SMSPP_PLAYER == 1
#warning "SMSPlusPlusEx hardware profile: Player 1"
#else
#warning "SMSPlusPlusEx hardware profile: Player 2 (D0-D5 unused)"
#endif

// -----------------------------------------------------------------------------
// Common controller-port wiring (P1 and P2)
// -----------------------------------------------------------------------------
// A0..A5 = controller input pins 1,2,3,4,6,9.
#define PDREG_PAD_PORT DDRC
#define PDREG_PAD_BITS ((1U << DDC5) | (1U << DDC4) | (1U << DDC3) | \
                        (1U << DDC2) | (1U << DDC1) | (1U << DDC0))
#define PIREG_PAD PINC
#define POREG_PAD PORTC

// D6 = external TH / pad SELECT.
#define SELECT_PAD_PIN 6
#define PDREG_SELECT_PORT DDRD
#define PDREG_SELECT_BIT DDD6
#define POREG_SELECT PORTD
#define PIREG_SELECT PIND

// D7 = TH electronic-switch (4066) control.
#define TI4066_CONTROL_PIN 7

// D8..D13 = output traces to the console controller port.
#define PDREG_TRACES_PORT DDRB
#define PDREG_TRACES_BITS ((1U << DDB5) | (1U << DDB4) | (1U << DDB3) | \
                           (1U << DDB2) | (1U << DDB1) | (1U << DDB0))
#define POREG_TRACES PORTB

// Trace 7 intentionally remains disabled for Light Phaser compatibility.
// #define PDREG_TRACE7_PORT DDRD
// #define PDREG_TRACE7_BIT DDD7
// #define POREG_TRACE7 PORTD

// -----------------------------------------------------------------------------
// Player 1-only console wiring, matching the current PCB schematic
// -----------------------------------------------------------------------------
// P1 Nano/Uno:
//   D0 -> FMS_4066_IN
//   D1 -> JFMS_4066_IN
//   D2 -> VIDEO_MODE
//   D3 -> OUT_RESET
//   D4 -> OUT_PAUSE
//   D5 -> PAUSE_IN
//
// Player 2 has no connection on D0..D5. Do NOT define those pins in that build:
// this prevents accidental pinMode()/digitalWrite() calls on unused P2 pins.
#if SMSPP_PLAYER == 1
#define FMSOUND_OUT_PIN 0
#define JAP_FMSOUND_OUT_PIN 1
#define VIDEOMODE_PIN 2
#define RESET_OUT_PIN 3
#define PAUSE_OUT_PIN 4
#define PAUSE_IN_PIN 5
#endif

// Optional physical Reset input, not wired on the shown PCB.
// #define RESET_IN_PIN ...

// Serial debugging uses D0/D1. It is safe on P2, but conflicts with the FM
// control lines on P1 and should therefore remain disabled there.
// #define ENABLE_SERIAL_DEBUG
// #define DEBUG_PAD
