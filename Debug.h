#pragma once
#include <Arduino.h>
#include "Config.h"

#ifdef ENABLE_SERIAL_DEBUG
#define debug(...) Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#else
#define debug(...)
#define debugln(...)
#endif
