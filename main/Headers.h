#ifndef HEADERS_H
#define HEADERS_H

#include <string>
#include <cstdint>
#include <esp_wifi.h>
#include <esp_now.h>
#include "Config.h"
#include "Machine_State.h"
#include "Globals.h"
#include "OTA_Manager.h"
#include "SpindleControl.h"
#include "StatusLEDs.h"
#include "led_strip.h"   // aggiunto

extern led_strip_handle_t global_led_strip;   // dichiarazione globale

#endif // HEADERS_H