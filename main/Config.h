#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

#define ledPin 8    // pin dati WS2812B
#define ledCount 18 // numero LED

// ========== ESP-NOW FRONTEND MAC ADDRESS ==========
// Sostituisci con il MAC del tuo frontend (ESP32-S3)
//#define FRONTEND_MAC {0x10, 0x20, 0xBA, 0x40, 0x32, 0x50}
#define FRONTEND_MAC {0x80, 0xB5, 0x4E, 0xC5, 0x8E, 0x98}

struct Config {
    uint8_t frontendMac[6] = FRONTEND_MAC;
    uint8_t espNowChannel = 0;

    const uint8_t spindleDirPin = 0;
    const uint8_t resetPin = 1;
    const uint8_t ledPinReset = 2;
    const uint8_t ledPinHold = 3;
    const uint8_t ledPinStart = 4;

    const uint8_t spindleEncA = 6;
    const uint8_t spindleEncB = 5;
    const uint8_t spindleKey = 7;

    const uint8_t brakePin = 9;
    const uint8_t ledPinBrake = 10;

    const unsigned long statusUpdateInterval = 100;
    const unsigned long loopDelay = 2;

    struct OTAConfig {
        const char *ssid = "MyESPfamily";
        const char *password = "12345678";
        const char *localIP = "192.168.5.1";
        const char *gateway = "192.168.5.1";
        const char *subnet = "255.255.255.0";
        const uint16_t serverPort = 80;
        const bool enabled = true;
        const unsigned long timeout = 30 * 60 * 1000;
    } ota;
};

extern Config config;

#endif // CONFIG_H
